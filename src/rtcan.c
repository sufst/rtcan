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
#define RTCAN_THREAD_NAME        "RTCAN Thread"
#define RTCAN_LISTENER_POOL_NAME "RTCAN Listener Memory Pool"
#define RTCAN_THREAD_STACK_SIZE 512 // TODO: this needs to be profiled

/*
 * internal functions
 */
static rtcan_status_t create_status(rtcan_handle_t* rtcan_h);
static bool no_errors(rtcan_handle_t* rtcan_h);
static uint32_t compute_hash(const uint32_t key);
static void rtcan_thread_entry(ULONG input);

static rtcan_status_t transmit_internal(rtcan_handle_t* rtcan_h,
                                        uint32_t identifier,
                                        const uint8_t* data_ptr,
                                        uint32_t data_length);

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

    // thread
    void* stack_ptr = NULL;

    UINT tx_status = tx_byte_allocate(stack_pool_ptr,
                                      &stack_ptr,
                                      RTCAN_THREAD_STACK_SIZE,
                                      TX_NO_WAIT);

    if (tx_status != TX_SUCCESS)
    {
        rtcan_h->err |= RTCAN_ERROR_INIT;
    }

    if (no_errors(rtcan_h))
    {
        tx_status = tx_thread_create(&rtcan_h->thread,
                                     RTCAN_THREAD_NAME,
                                     rtcan_thread_entry,
                                     (ULONG) rtcan_h,
                                     stack_ptr,
                                     RTCAN_THREAD_STACK_SIZE,
                                     priority,
                                     priority,
                                     TX_NO_TIME_SLICE,
                                     TX_DONT_START);

        if (tx_status != TX_SUCCESS) // must have been caused by invalid arg
        {
            rtcan_h->err |= RTCAN_ERROR_INIT;
        }
    }

    // transmit queue
    if (no_errors(rtcan_h))
    {
        tx_status = tx_queue_create(&rtcan_h->tx_queue,
                                    "RTCAN Transmit Queue",
                                    RTCAN_TX_QUEUE_ITEM_SIZE,
                                    rtcan_h->tx_queue_mem,
                                    RTCAN_TX_QUEUE_SIZE);

        if (tx_status != TX_SUCCESS)
        {
            rtcan_h->err |= RTCAN_ERROR_INIT;
        }
    }

    // transmit mailbox semaphore
    if (no_errors(rtcan_h))
    {
        const uint32_t mailbox_size
            = sizeof(rtcan_h->hcan->Instance->sTxMailBox)
              / sizeof(CAN_TxMailBox_TypeDef);

        tx_status
            = tx_semaphore_create(&rtcan_h->tx_mailbox_sem, NULL, mailbox_size);

        if (tx_status != TX_SUCCESS)
        {
            rtcan_h->err |= RTCAN_ERROR_INTERNAL;
        }
    }

    // clear hash table of listeners
    if (no_errors(rtcan_h))
    {
        for (uint32_t i = 0; i < RTCAN_HASHMAP_SIZE; i++)
        {
            rtcan_h->listener_map[i] = NULL;
        }
    }

    // create listener memory pool
    if (no_errors(rtcan_h))
    {
        ULONG status = tx_byte_pool_create(&rtcan_h->listener_pool, 
                                           RTCAN_LISTENER_POOL_NAME,
                                           rtcan_h->listener_pool_mem,
                                           RTCAN_LISTENER_POOL_SIZE);

        if (status != TX_SUCCESS)
        {
            rtcan_h->err |= RTCAN_ERROR_INTERNAL;
        }
    }

    return create_status(rtcan_h);
}

/**
 * @brief   Starts the RTCAN service
 *
 * @param[in]   rtcan_h     RTCAN handle
 */
rtcan_status_t rtcan_start(rtcan_handle_t* rtcan_h)
{
    UINT tx_status = tx_thread_resume(&rtcan_h->thread);

    if (tx_status != TX_SUCCESS)
    {
        rtcan_h->err |= RTCAN_ERROR_INIT;
    }

    // start peripheral
    if (no_errors(rtcan_h))
    {
        // TODO: does this work?
        HAL_StatusTypeDef hal_status
            = HAL_CAN_ActivateNotification(rtcan_h->hcan,
                                           CAN_IT_TX_MAILBOX_EMPTY);

        if (hal_status != HAL_OK)
        {
            rtcan_h->err |= RTCAN_ERROR_INIT;
        }
    }

    if (no_errors(rtcan_h))
    {
        HAL_StatusTypeDef hal_status = HAL_CAN_Start(rtcan_h->hcan);

        if (hal_status != HAL_OK)
        {
            rtcan_h->err |= RTCAN_ERROR_INIT;
        }
    }

    return create_status(rtcan_h);
}

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

    if (tx_status != TX_SUCCESS)
    {
        rtcan_h->err |= RTCAN_ERROR_MEMORY_FULL;
    }

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

        if (tx_status != TX_SUCCESS)
        {
            rtcan_h->err |= RTCAN_ERROR_INTERNAL;
        }
    }

    return create_status(rtcan_h);
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

    ULONG status = tx_byte_allocate(&rtcan_h->listener_pool, 
                                    (void**) &new_node_ptr,
                                    sizeof(rtcan_hashmap_node_t),
                                    TX_NO_WAIT);

    if (status == TX_SUCCESS)
    {
        new_node_ptr->chained_node_ptr = NULL;
        new_node_ptr->can_id = can_id;
    }
    else
    {
        rtcan_h->err |= RTCAN_ERROR_MEMORY_FULL; // assume this is the error?
    }

    return new_node_ptr;
}

/**
 * @brief       Creates a listener node
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   queue_ptr   Pointer to listener's associated queue
 */
static rtcan_listener_t* create_listener(rtcan_handle_t* rtcan_h,
                                         TX_QUEUE* queue_ptr)
{
    rtcan_listener_t* new_listener_ptr = NULL;

    ULONG status = tx_byte_allocate(&rtcan_h->listener_pool, 
                                    (void**) &new_listener_ptr,
                                    sizeof(rtcan_listener_t),
                                    TX_NO_WAIT);

    if (status == TX_SUCCESS)
    {
        new_listener_ptr->next_listener_ptr = NULL;
        new_listener_ptr->queue_ptr = queue_ptr;
    }
    else
    {
        rtcan_h->err |= RTCAN_ERROR_MEMORY_FULL; // assume this is the error?
    }
    
    return new_listener_ptr;
}

/**
 * @brief   Appends a listener to an existing node in the hashmap
 */

/**
 * @brief       Adds a subscriber which will receive notifications of incoming
 *              CAN messages via a TX_QUEUE
 * 
 * @details     Hash collisions are handled by collision chaining with a singly
 *              linked list. Each node in the hash map (or chain) consists
 *              of a singly linked list of listeners for the given CAN ID.
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_id      CAN ID to receive notification for
 * @param[in]   queue_ptr   Destination to receive messages
 */
rtcan_status_t rtcan_subscribe(rtcan_handle_t* rtcan_h,
                               uint32_t can_id, 
                               TX_QUEUE* queue_ptr)
{
    const uint32_t index = compute_hash(can_id) % RTCAN_HASHMAP_SIZE;

    // first time for this CAN ID, no collision
    if (rtcan_h->listener_map[index] == NULL)
    {
        rtcan_hashmap_node_t* new_node_ptr = create_hashmap_node(rtcan_h, can_id);

        if (no_errors(rtcan_h))
        {  
            new_node_ptr->first_listener_ptr = create_listener(rtcan_h, queue_ptr);
        }

        if (no_errors(rtcan_h))
        {
            rtcan_h->listener_map[index] = new_node_ptr;
        }
    }
    // hash collision, or another listener for an existing ID in the map
    else
    {
        rtcan_hashmap_node_t* node_ptr = rtcan_h->listener_map[index];

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
                node_ptr->chained_node_ptr = create_hashmap_node(rtcan_h, can_id);

                if (no_errors(rtcan_h))
                {
                    node_ptr = node_ptr->chained_node_ptr;
                    node_ptr->first_listener_ptr = create_listener(rtcan_h, queue_ptr);
                }
            }
            else // add to existing node
            {
                rtcan_listener_t* listener_ptr = node_ptr->first_listener_ptr;

                while (listener_ptr->next_listener_ptr != NULL)
                {
                    listener_ptr = listener_ptr->next_listener_ptr;
                }

                listener_ptr->next_listener_ptr = create_listener(rtcan_h, queue_ptr);
            }

        }
        else // no collision, append to this node
        {
            rtcan_listener_t* listener_ptr = node_ptr->first_listener_ptr;

            while (listener_ptr->next_listener_ptr != NULL)
            {
                listener_ptr = listener_ptr->next_listener_ptr;
            }

            listener_ptr->next_listener_ptr = create_listener(rtcan_h, queue_ptr);
        }
    }

    return create_status(rtcan_h);
}

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
 * @brief       Entry function for RTCAN service thread
 *
 * @param[in]   input   RTCAN handle
 */
static void rtcan_thread_entry(ULONG input)
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

/**
 * @brief       Internal transmit for RTCAN service thread
 *
 * @param[in]   rtcan_h       RTCAN handle
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

    if (no_errors(rtcan_h))
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