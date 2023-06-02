/***************************************************************************
 * @file   rtcan.c
 * @author Tim Brewis (@t-bre, tab1g19@soton.ac.uk)
 * @brief  RTOS wrapper around CAN bus
 ***************************************************************************/

#include "rtcan.h"

#include <stdbool.h>
#include <stdint.h>

#include <memory.h>

/*
 * thread constants
 */
#define RTCAN_THREAD_STACK_SIZE 1024 // TODO: this needs to be profiled

/*
 * useful macros
 */
// #define ADD_ERROR_IF(cond, error, inst) if(cond) { inst->err |= error; }
#define ADD_ERROR_IF(cond, error, inst) ;

/*
 * internal functions
 */
static rtcan_status_t create_status(rtcan_handle_t* rtcan_h);
static bool no_errors(rtcan_handle_t* rtcan_h);
static uint32_t compute_hash(const uint32_t key);
static void rtcan_tx_thread_entry(ULONG input);
static void rtcan_rx_thread_entry(ULONG input);

static rtcan_status_t transmit_internal(rtcan_handle_t* rtcan_h,
                                        uint32_t identifier,
                                        const uint8_t* data_ptr,
                                        uint32_t data_length);


//=============================================================== initialisation

/**
 * @brief       Initialises the RTCAN instance
 *
 * @details     The CAN instance of the handle should not be used by any other
 *              part of the system
 *
 * @param[in]   rtcan_h         RTCAN handle
 * @param[in]   hcan            CAN handle
 * @param[in]   priority        Service thread priority
 * @param[in]   stack_pool_ptr  Memory pool to allocate stack memory from
 */
rtcan_status_t rtcan_init(rtcan_handle_t* rtcan_h,
                          CAN_HandleTypeDef* hcan,
                          ULONG priority,
                          TX_BYTE_POOL* stack_pool_ptr)
{
    rtcan_h->hcan = hcan;
    rtcan_h->err = RTCAN_ERROR_NONE;
    atomic_store(&rtcan_h->rx_ready, true);

    // threads
    void* stack_ptr = NULL;

    UINT tx_status = tx_byte_allocate(stack_pool_ptr,
                                      &stack_ptr,
                                      RTCAN_THREAD_STACK_SIZE,
                                      TX_NO_WAIT);

    ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INIT, rtcan_h);

    if (no_errors(rtcan_h))
    {
        tx_status = tx_thread_create(&rtcan_h->tx_thread,
                                     "RTCAN Tx Thread",
                                     rtcan_tx_thread_entry,
                                     (ULONG) rtcan_h,
                                     stack_ptr,
                                     RTCAN_THREAD_STACK_SIZE,
                                     priority,
                                     priority,
                                     TX_NO_TIME_SLICE,
                                     TX_DONT_START);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INIT, rtcan_h);
    }

    if (no_errors(rtcan_h))
    {
        tx_status = tx_byte_allocate(stack_pool_ptr,
                                     &stack_ptr,
                                     RTCAN_THREAD_STACK_SIZE,
                                     TX_NO_WAIT);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INIT, rtcan_h);
    }

    if (no_errors(rtcan_h))
    {
        tx_status = tx_thread_create(&rtcan_h->rx_thread,
                                     "RTCAN Rx Thread",
                                     rtcan_rx_thread_entry,
                                     (ULONG) rtcan_h,
                                     stack_ptr,
                                     RTCAN_THREAD_STACK_SIZE,
                                     priority,
                                     priority,
                                     TX_NO_TIME_SLICE,
                                     TX_DONT_START);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INIT, rtcan_h);
    }

    // transmit queue
    if (no_errors(rtcan_h))
    {
        tx_status = tx_queue_create(&rtcan_h->tx_queue,
                                    "RTCAN Transmit Queue",
                                    RTCAN_TX_QUEUE_ITEM_SIZE,
                                    rtcan_h->tx_queue_mem,
                                    RTCAN_TX_QUEUE_SIZE * sizeof(ULONG));

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INIT, rtcan_h);
    }

    // transmit mailbox semaphore
    if (no_errors(rtcan_h))
    {
        const uint32_t mailbox_size
            = sizeof(rtcan_h->hcan->Instance->sTxMailBox)
              / sizeof(CAN_TxMailBox_TypeDef);

        tx_status
            = tx_semaphore_create(&rtcan_h->tx_mailbox_sem, NULL, mailbox_size);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    // receive notification queue
    if (no_errors(rtcan_h))
    {
        tx_status = tx_queue_create(&rtcan_h->rx_notif_queue,
                                    "RTCAN Receive Notification Queue",
                                    RTCAN_RX_NOTIF_QUEUE_ITEM_SIZE,
                                    rtcan_h->rx_notif_queue_mem,
                                    RTCAN_RX_NOTIF_QUEUE_SIZE * sizeof(ULONG));

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    // clear hash table of subscribers
    if (no_errors(rtcan_h))
    {
        for (uint32_t i = 0; i < RTCAN_HASHMAP_SIZE; i++)
        {
            rtcan_h->subscriber_map[i] = NULL;
        }
    }

    // create subscriber memory pool
    if (no_errors(rtcan_h))
    {
        UINT tx_status = tx_byte_pool_create(&rtcan_h->subscriber_pool, 
                                             "RTCAN Subscriber Pool",
                                             rtcan_h->subscriber_pool_mem,
                                             RTCAN_SUBSCRIBER_POOL_SIZE);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    // create rx message memory pool
    if (no_errors(rtcan_h))
    {
        UINT tx_status = tx_block_pool_create(&rtcan_h->rx_msg_pool,
                                              "RTCAN Rx Message Pool",
                                              sizeof(rtcan_msg_t),
                                              rtcan_h->rx_msg_pool_mem,
                                              sizeof(rtcan_msg_t) * RTCAN_RX_MSG_POOL_SIZE);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    // TODO: configure CAN filters
    if (no_errors(rtcan_h))
    {
        CAN_FilterTypeDef filter;
        filter.FilterActivation = ENABLE;
        filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter.FilterIdHigh = 0x00AA << 5U; // pm100 internal states
        filter.FilterIdLow = 0x0000 << 5U;
        filter.FilterMaskIdHigh = 0x0000 << 5U;
        filter.FilterMaskIdLow = 0x0000 << 5U;
        filter.FilterMode = CAN_FILTERMODE_IDLIST;
        filter.FilterScale = CAN_FILTERSCALE_16BIT;
        filter.FilterBank = 0;

        HAL_StatusTypeDef hal_status = HAL_CAN_ConfigFilter(rtcan_h->hcan, 
                                                            &filter);

        ADD_ERROR_IF(hal_status != HAL_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    return create_status(rtcan_h);
}

// #define IS_CAN_IT(IT) ((IT) <= (CAN_IT_TX_MAILBOX_EMPTY     | CAN_IT_RX_FIFO0_MSG_PENDING      | \
//                                 CAN_IT_RX_FIFO0_FULL        | CAN_IT_RX_FIFO0_OVERRUN          | \
//                                 CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL             | \
//                                 CAN_IT_RX_FIFO1_OVERRUN     | CAN_IT_WAKEUP                    | \
//                                 CAN_IT_SLEEP_ACK            | CAN_IT_ERROR_WARNING             | \
//                                 CAN_IT_ERROR_PASSIVE        | CAN_IT_BUSOFF                    | \
//                                 CAN_IT_LAST_ERROR_CODE      | CAN_IT_ERROR))

/**
 * @brief   Starts the RTCAN service
 *
 * @param[in]   rtcan_h     RTCAN handle
 */
rtcan_status_t rtcan_start(rtcan_handle_t* rtcan_h)
{
    TX_THREAD* threads[2] = {&rtcan_h->tx_thread, &rtcan_h->rx_thread};

    for (uint32_t i = 0; i < 2; i++)
    {
        UINT tx_status = tx_thread_resume(threads[i]);
        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INIT, rtcan_h);
    }

    // start peripheral
    if (no_errors(rtcan_h))
    {
        const uint32_t notifs = CAN_IT_TX_MAILBOX_EMPTY
                                | CAN_IT_RX_FIFO0_MSG_PENDING
                                | CAN_IT_RX_FIFO1_MSG_PENDING
                                | CAN_IT_ERROR
                                | CAN_IT_BUSOFF
                                | CAN_IT_ERROR_PASSIVE
                                | CAN_IT_ERROR_WARNING;

        HAL_StatusTypeDef hal_status
            = HAL_CAN_ActivateNotification(rtcan_h->hcan,
                                           notifs);

        ADD_ERROR_IF(hal_status != HAL_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    if (no_errors(rtcan_h))
    {
        HAL_StatusTypeDef hal_status = HAL_CAN_Start(rtcan_h->hcan);
        
        while(HAL_CAN_GetState(rtcan_h->hcan) != HAL_CAN_STATE_LISTENING)
            ;

        ADD_ERROR_IF(hal_status != HAL_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    return create_status(rtcan_h);
}

//=================================================================== tx service

/**
 * @brief       Transmits a CAN message using the RTCAN service
 *
 * @details     The message is queued for transmission in a FIFO buffer, and the
 *              contents of the message is copied (!) to the buffer
 *
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   msg_ptr     Pointer to message to transmit
 */
rtcan_status_t rtcan_transmit(rtcan_handle_t* rtcan_h, rtcan_msg_t* msg_ptr)
{
    UINT tx_status
        = tx_queue_send(&rtcan_h->tx_queue, (void*) msg_ptr, TX_NO_WAIT);

    ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_MEMORY_FULL, rtcan_h);

    return create_status(rtcan_h);
}

/**
 * @brief       Transmit mailbox callback
 *
 * @details     Increments the mailbox semaphore to allow the next message to be
 *              dispatched. This MUST be called by the user from
 *              HAL_CAN_TxMailbox<n>CompleteCallback, for all n.
 *
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_h       CAN handle passed to HAL callback
 */
rtcan_status_t rtcan_handle_tx_mailbox_callback(rtcan_handle_t* rtcan_h,
                                                const CAN_HandleTypeDef* can_h)
{
    if (rtcan_h->hcan == can_h)
    {
        UINT tx_status = tx_semaphore_put(&rtcan_h->tx_mailbox_sem);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    return create_status(rtcan_h);
}

/**
 * @brief       Internal transmit for RTCAN service thread
 *
 * @details     Blocks RTCAN service if CAN transmit mailbox unavailable
 * 
 * @param[in]   rtcan_h         RTCAN handle
 * @param[in]   identifier      CAN standard identifier
 * @param[in]   data_ptr        Pointer to data to transmit
 * @param[in]   data_length     Length of data to transmit
 */
static rtcan_status_t transmit_internal(rtcan_handle_t* rtcan_h,
                                        uint32_t identifier,
                                        const uint8_t* data_ptr,
                                        uint32_t data_length)
{
    if ((data_ptr == NULL) || (data_length == 0U))
    {
        rtcan_h->err |= RTCAN_ERROR_ARG;
    }

    if (tx_semaphore_get(&rtcan_h->tx_mailbox_sem, TX_WAIT_FOREVER)
        != TX_SUCCESS)
    {
        rtcan_h->err |= RTCAN_ERROR_INTERNAL;
    }

    // if (no_errors(rtcan_h))
    {
        // create message
        CAN_TxHeaderTypeDef header = {
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = data_length,
            .StdId = identifier,
        };

        // send it
        uint32_t tx_mailbox;

        HAL_StatusTypeDef hal_status = HAL_CAN_AddTxMessage(rtcan_h->hcan,
                                                            &header,
                                                            data_ptr,
                                                            &tx_mailbox);


        if (hal_status != HAL_OK)
        {
            rtcan_h->err |= RTCAN_ERROR_INTERNAL;
        }
    }

    return create_status(rtcan_h);
}

/**
 * @brief       Entry function for RTCAN transmit service thread
 *
 * @param[in]   input   RTCAN handle
 */
static void rtcan_tx_thread_entry(ULONG input)
{
    rtcan_handle_t* rtcan_h = (rtcan_handle_t*) input;

    while (1)
    {
        const rtcan_msg_t message;
        UINT tx_status = tx_queue_receive(&rtcan_h->tx_queue,
                                          (void*) &message,
                                          TX_WAIT_FOREVER);

        if (tx_status == TX_SUCCESS)
        {
            (void) transmit_internal(rtcan_h,
                                     message.identifier,
                                     message.data,
                                     message.length);
        }
        else
        {
            // TODO: handle error
        }
    }
}

//================================================================ subscriptions

/**
 * @brief   Computes a hash for a single word
 * 
 * @details This implements a Jenkins hash which was chosen for its balance 
 *          between speed and distribution. The input data of CAN message
 *          IDs is small.
 */
static uint32_t compute_hash(const uint32_t key)
{
    uint32_t hash = key;
    hash += (hash << 12);
    hash ^= (hash >> 22);
    hash += (hash << 4);
    hash ^= (hash >> 9);
    hash += (hash << 10);
    hash ^= (hash >> 2);
    hash += (hash << 7);
    hash ^= (hash >> 12);
    return hash;
}

/**
 * @brief   Computes an index in the hash table of subscribers
 */
static inline uint32_t hashmap_index(const uint32_t can_id)
{
    return compute_hash(can_id) % RTCAN_HASHMAP_SIZE;
}

/**
 * @brief       Creates a hashmap node
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_id      CAN ID of node
 */
static rtcan_hashmap_node_t* create_hashmap_node(rtcan_handle_t* rtcan_h,   
                                                 uint32_t can_id)
{
    rtcan_hashmap_node_t* new_node_ptr = NULL;

    ULONG status = tx_byte_allocate(&rtcan_h->subscriber_pool, 
                                    (void**) &new_node_ptr,
                                    sizeof(rtcan_hashmap_node_t),
                                    TX_NO_WAIT);

    ADD_ERROR_IF(status != TX_SUCCESS, RTCAN_ERROR_MEMORY_FULL, rtcan_h);

    if (no_errors(rtcan_h))
    {
        new_node_ptr->chained_node_ptr = NULL;
        new_node_ptr->can_id = can_id;
    }

    return new_node_ptr;
}

/**
 * @brief       Creates a subscriber node
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   queue_ptr   Pointer to subscriber's associated queue
 */
static rtcan_subscriber_t* create_subscriber(rtcan_handle_t* rtcan_h,
                                         TX_QUEUE* queue_ptr)
{
    rtcan_subscriber_t* new_subscriber_ptr = NULL;

    ULONG status = tx_byte_allocate(&rtcan_h->subscriber_pool, 
                                    (void**) &new_subscriber_ptr,
                                    sizeof(rtcan_subscriber_t),
                                    TX_NO_WAIT);

    ADD_ERROR_IF(status != TX_SUCCESS, RTCAN_ERROR_MEMORY_FULL, rtcan_h);

    if (no_errors(rtcan_h))
    {
        new_subscriber_ptr->next_subscriber_ptr = NULL;
        new_subscriber_ptr->queue_ptr = queue_ptr;
    }
    
    return new_subscriber_ptr;
}

/**
 * @brief       Returns a pointer to the hashmap node with the given CAN ID, or
 *              null if there are no nodes with that ID
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_id      CAN ID
 */
static rtcan_hashmap_node_t* find_hashmap_node(rtcan_handle_t* rtcan_h,
                                               const uint32_t can_id)
{
    const uint32_t index = hashmap_index(can_id);
    return rtcan_h->subscriber_map[index];
}

/**
 * @brief       Appends a subscriber to an existing node in the hashmap
 */

/**
 * @brief       Adds a subscriber which will receive notifications of incoming
 *              CAN messages via a TX_QUEUE
 * 
 * @details     Hash collisions are handled by collision chaining with a singly
 *              linked list. Each node in the hash map (or chain) consists
 *              of a singly linked list of subscribers for the given CAN ID.
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_id      CAN ID to receive notification for
 * @param[in]   queue_ptr   Destination to receive messages
 */
rtcan_status_t rtcan_subscribe(rtcan_handle_t* rtcan_h,
                               uint32_t can_id, 
                               TX_QUEUE* queue_ptr)
{
    const uint32_t index = hashmap_index(can_id);

    // first time for this CAN ID, no collision
    if (rtcan_h->subscriber_map[index] == NULL)
    {
        rtcan_hashmap_node_t* new_node_ptr = create_hashmap_node(rtcan_h, 
                                                                 can_id);

        if (no_errors(rtcan_h))
        {  
            new_node_ptr->first_subscriber_ptr = create_subscriber(rtcan_h, 
                                                                   queue_ptr);
        }

        if (no_errors(rtcan_h))
        {
            rtcan_h->subscriber_map[index] = new_node_ptr;
        }
    }
    // hash collision, or another subscriber for an existing ID in the map
    else
    {
        rtcan_hashmap_node_t* node_ptr = rtcan_h->subscriber_map[index];

        if (node_ptr->can_id != can_id) // hash collision, do chaining
        {
            bool id_in_chain = false;

            while (node_ptr->chained_node_ptr != NULL)
            {
                node_ptr = node_ptr->chained_node_ptr;

                if (node_ptr->can_id == can_id)
                {
                    id_in_chain = true;
                    break;
                }
            }

            if (!id_in_chain) // create new chained node
            {
                node_ptr->chained_node_ptr = create_hashmap_node(rtcan_h, 
                                                                 can_id);

                if (no_errors(rtcan_h))
                {
                    node_ptr = node_ptr->chained_node_ptr;
                    node_ptr->first_subscriber_ptr = create_subscriber(rtcan_h, 
                                                                       queue_ptr);
                }
            }
            else // add to existing node
            {
                rtcan_subscriber_t* subscriber_ptr = node_ptr->first_subscriber_ptr;

                while (subscriber_ptr->next_subscriber_ptr != NULL)
                {
                    subscriber_ptr = subscriber_ptr->next_subscriber_ptr;
                }

                subscriber_ptr->next_subscriber_ptr = create_subscriber(rtcan_h, 
                                                                        queue_ptr);
            }

        }
        else // no collision, append to this node
        {
            rtcan_subscriber_t* subscriber_ptr = node_ptr->first_subscriber_ptr;

            while (subscriber_ptr->next_subscriber_ptr != NULL)
            {
                subscriber_ptr = subscriber_ptr->next_subscriber_ptr;
            }

            subscriber_ptr->next_subscriber_ptr = create_subscriber(rtcan_h, 
                                                                    queue_ptr);
        }
    }

    return create_status(rtcan_h);
}

//=================================================================== rx service

/**
 * @brief       Handler for CAN receive interrupts
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_h       CAN handle from interrupt
 * @param[in]   rx_fifo     Receive FIFO number
 */
rtcan_status_t rtcan_handle_rx_it(rtcan_handle_t* rtcan_h, 
                                  const CAN_HandleTypeDef* can_h,
                                  const uint32_t rx_fifo)
{
    if (!atomic_load(&rtcan_h->rx_ready))
    {
        HAL_CAN_GetRxMessage(rtcan_h->hcan,
                             rx_fifo,
                             NULL,
                             NULL);
        return RTCAN_OK;
    }

    // allocate message
    rtcan_msg_t* msg_ptr = NULL;

    UINT tx_status = tx_block_allocate(&rtcan_h->rx_msg_pool,
                                       (void**) &msg_ptr,
                                       TX_NO_WAIT);

    ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_MEMORY_FULL, rtcan_h);

    // retrieve message
    if (no_errors(rtcan_h))
    {
        CAN_RxHeaderTypeDef header;

        HAL_StatusTypeDef hal_status = HAL_CAN_GetRxMessage(rtcan_h->hcan,
                                                            rx_fifo,
                                                            &header,
                                                            msg_ptr->data);

        if (hal_status == HAL_OK)
        {
            msg_ptr->identifier = header.StdId;
            msg_ptr->length = header.DLC;
            msg_ptr->reference_count = 0;
        }
        else 
        {
            tx_block_release(msg_ptr);
        }

        ADD_ERROR_IF(hal_status != HAL_OK, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    // send to Rx thread for distribution
    if (no_errors(rtcan_h))
    {
        tx_status = tx_queue_send(&rtcan_h->rx_notif_queue, 
                                  (void*) &msg_ptr,
                                  TX_NO_WAIT);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_MEMORY_FULL, rtcan_h);
    }

    return create_status(rtcan_h);
}

/**
 * @brief       Call after message received via subscription has been used
 * 
 * @note        If this is not done, RTCAN will eventually run out of memory to
 *              store CAN messages!
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   msg_ptr     Pointer to message
 */
rtcan_status_t rtcan_msg_consumed(rtcan_handle_t* rtcan_h,
                                  rtcan_msg_t* msg_ptr)
{
    (void) rtcan_h;

    msg_ptr->reference_count--;

    if (msg_ptr->reference_count == 0)
    {
        tx_block_release(msg_ptr);
    }

    return RTCAN_OK;
}

/**
 * @brief       Entry function for RTCAN receive service thread
 * 
 * @param[in]   input   RTCAN handler
 */
static void rtcan_rx_thread_entry(ULONG input)
{
    rtcan_handle_t* rtcan_h = (rtcan_handle_t*) input;

    while (1)
    {
        atomic_store(&rtcan_h->rx_ready, true);

        // wait for message
        rtcan_msg_t* msg_ptr;

        UINT tx_status = tx_queue_receive(&rtcan_h->rx_notif_queue,
                                          (void*) &msg_ptr,
                                          TX_WAIT_FOREVER);

        ADD_ERROR_IF(tx_status != TX_SUCCESS, RTCAN_ERROR_INTERNAL, rtcan_h);

        // distribute
        if (no_errors(rtcan_h))
        {
            rtcan_hashmap_node_t* node_ptr = find_hashmap_node(rtcan_h,
                                                               msg_ptr->identifier);

            if (node_ptr != NULL)
            {
                rtcan_subscriber_t* subscriber_ptr = node_ptr->first_subscriber_ptr;

                while (subscriber_ptr != NULL)
                {
                    msg_ptr->reference_count++;
                    
                    tx_status = tx_queue_send(subscriber_ptr->queue_ptr,
                                              &msg_ptr,
                                              TX_NO_WAIT);

                    if (tx_status != TX_SUCCESS)
                    {
                        msg_ptr->reference_count--;
                    }

                    subscriber_ptr = subscriber_ptr->next_subscriber_ptr;
                }

                // catch for errors sending in queue
                if (msg_ptr->reference_count == 0)
                {
                    tx_block_release(msg_ptr);
                }
            }
            else // don't care about this message
            {
                tx_block_release(msg_ptr);
            }
        }
    }
}

//======================================================================== error

/**
 * @brief       Handles HAL CAN errors
 * 
 * @note        `HAL_CAN_ERROR_BD` and `HAL_CAN_ERROR_CRC` are much more likely
 *              to happen in the actual car where the noise level is higher.
 *  
 *              Note also that it is possible for multiple errors to happen at
 *              once so the error code will be equal to a bitwise combination 
 *              of the HAL error codes.
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_h       CAN handle
 */
rtcan_status_t rtcan_handle_hal_error(rtcan_handle_t* rtcan_h,
                                      CAN_HandleTypeDef* can_h)
{
    rtcan_status_t status = RTCAN_OK;

    if (can_h == rtcan_h->hcan)
    {
        const uint32_t tx_errors[] = {
            HAL_CAN_ERROR_TX_TERR0,
            HAL_CAN_ERROR_TX_TERR1,
            HAL_CAN_ERROR_TX_TERR2,
            HAL_CAN_ERROR_TX_ALST0,
            HAL_CAN_ERROR_TX_ALST1,
            HAL_CAN_ERROR_TX_ALST2,
            HAL_CAN_ERROR_BD,
            HAL_CAN_ERROR_CRC,
            HAL_CAN_ERROR_ACK
        };

// #if (1)

        tx_semaphore_put(&rtcan_h->tx_mailbox_sem);
        HAL_CAN_ResetError(rtcan_h->hcan);
#ifdef AHDFKJSD
        uint32_t error = HAL_CAN_GetError(rtcan_h->hcan);
        bool error_handled = false;

        for (uint32_t i = 0; i < sizeof(tx_errors)/sizeof(tx_errors[0]); i++)
        {
            if (error & tx_errors[i]) // check the single bit
            {
                tx_semaphore_put(&rtcan_h->tx_mailbox_sem);
                HAL_CAN_ResetError(rtcan_h->hcan);
                error_handled = true;
                break;
            }
        }

        // (void) error_handled;
        // if (!error_handled)
        // {
        //     tx_semaphore_put(&rtcan_h->tx_mailbox_sem);
        //     HAL_CAN_ResetError(rtcan_h->hcan);
        // }

        // // unhandled/unknown errors
        // ADD_ERROR_IF(!error_handled, RTCAN_ERROR_INTERNAL, rtcan_h);
#endif
    }

    return status;
}

//====================================================================== utility

/**
 * @brief       Returns the error code
 *
 * @param[in]   rtcan_h   RTCAN handle
 */
uint32_t rtcan_get_error(rtcan_handle_t* rtcan_h)
{
    return rtcan_h->err;
}

/**
 * @brief       Returns true if the RTCAN instance has encountered an error
 *
 * @param[in]   rtcan_h   RTCAN handle
 */
static bool no_errors(rtcan_handle_t* rtcan_h)
{
    return (rtcan_h->err == RTCAN_ERROR_NONE);
}

/**
 * @brief       Create a status code based on the current error state
 *
 * @param[in]   rtcan_h   RTCAN handle
 */
static rtcan_status_t create_status(rtcan_handle_t* rtcan_h)
{
    return (no_errors(rtcan_h)) ? RTCAN_OK : RTCAN_ERROR;
}
