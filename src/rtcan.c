/***************************************************************************
 * @file   rtcan.c
 * @author Tim Brewis (@t-bre, tab1g19@soton.ac.uk)
 *         Refactored by Antigravity (Google DeepMind team)
 * @brief  RTOS-agnostic wrapper around CAN bus
 ***************************************************************************/

#include "rtcan.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * internal functions
 */
static rtcan_status_t create_status(rtcan_handle_t* rtcan_h);
static bool no_errors(const rtcan_handle_t* rtcan_h);
static void rtcan_tx_thread_entry(void* arg);
static void rtcan_rx_thread_entry(void* arg);

static rtcan_status_t transmit_internal(rtcan_handle_t* rtcan_h,
                                        uint32_t identifier,
                                        const uint8_t* data_ptr,
                                        uint32_t data_length,
                                        bool extended);

/**
 * @brief       Static inline helper to safely set error code if condition is met
 */
static inline void add_error_if(bool cond, uint32_t error, rtcan_handle_t* inst)
{
    if (cond)
    {
        atomic_fetch_or(&inst->err, error);
    }
}

//=============================================================== initialisation

/**
 * @brief       Initialises the RTCAN instance
 *
 * @param[in]   rtcan_h   RTCAN handle
 * @param[in]   hcan      CAN handle
 * @param[in]   config    RTCAN configuration structure
 */
rtcan_status_t rtcan_init(rtcan_handle_t* rtcan_h,
                          CAN_HandleTypeDef* hcan,
                          const rtcan_config_t* config)
{
    if ((rtcan_h == NULL) || (hcan == NULL) || (config == NULL))
    {
        return RTCAN_ERROR;
    }

    if (rtcan_h->subscriber_mutex != NULL)
    {
        return RTCAN_ERROR;
    }

    rtcan_h->hcan = hcan;
    rtcan_h->err = RTCAN_ERROR_NONE;
    atomic_store(&rtcan_h->started, false);
    /* Initialize subscriber registry */
    for (uint32_t i = 0U; i < RTCAN_MAX_SUBSCRIBERS; i++)
    {
        rtcan_h->subscriber_pool[i].queue_ptr = NULL;
        rtcan_h->subscriber_pool[i].next_subscriber_ptr = NULL;
        rtcan_h->subscriber_pool[i].in_use = false;
    }
    for (uint32_t i = 0U; i < 2048U; i++)
    {
        rtcan_h->subscriber_lut[i] = NULL;
    }

    /* Create subscriber mutex (binary semaphore) */
    rtcan_osal_status_t sub_mutex_status = rtcan_os_sem_create(&rtcan_h->subscriber_mutex,
                                                               "RTCAN Subscriber Mutex",
                                                               1U,
                                                               1U);
    add_error_if(sub_mutex_status != RTCAN_OS_OK, RTCAN_ERROR_INIT, rtcan_h);

    /* Create transmit queue */
    if (no_errors(rtcan_h))
    {
        rtcan_osal_status_t os_status = rtcan_os_queue_create(&rtcan_h->tx_queue,
                                                              "RTCAN Transmit Queue",
                                                              sizeof(rtcan_msg_t),
                                                              RTCAN_TX_QUEUE_LENGTH,
                                                              rtcan_h->tx_queue_mem,
                                                              sizeof(rtcan_h->tx_queue_mem));
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    /* Create receive notification queue */
    if (no_errors(rtcan_h))
    {
        rtcan_osal_status_t os_status = rtcan_os_queue_create(&rtcan_h->rx_notif_queue,
                                                              "RTCAN Rx Notif Queue",
                                                              sizeof(rtcan_msg_t*),
                                                              RTCAN_RX_NOTIF_QUEUE_LENGTH,
                                                              rtcan_h->rx_notif_queue_mem,
                                                              sizeof(rtcan_h->rx_notif_queue_mem));
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    /* Create transmit mailbox semaphore (initial count = max count = 3 mailboxes) */
    if (no_errors(rtcan_h))
    {
        const uint32_t mailbox_size = 3U;
        rtcan_osal_status_t os_status = rtcan_os_sem_create(&rtcan_h->tx_mailbox_sem,
                                                            "RTCAN Tx Mailbox Sem",
                                                            mailbox_size,
                                                            mailbox_size);
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    /* Create Rx message block pool */
    if (no_errors(rtcan_h))
    {
        rtcan_osal_status_t os_status = rtcan_os_block_pool_create(&rtcan_h->rx_msg_pool,
                                                                   "RTCAN Rx Message Pool",
                                                                   sizeof(rtcan_msg_t),
                                                                   RTCAN_RX_MSG_POOL_SIZE,
                                                                   rtcan_h->rx_msg_pool_mem,
                                                                   sizeof(rtcan_h->rx_msg_pool_mem));
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    /* Create background service threads */
    if (no_errors(rtcan_h))
    {
        rtcan_osal_status_t os_status = rtcan_os_thread_create(&rtcan_h->tx_thread,
                                                               "RTCAN Tx Thread",
                                                               rtcan_tx_thread_entry,
                                                               (void*) rtcan_h,
                                                               config->thread_priority,
                                                               config->tx_thread_stack_size,
                                                               config->tx_thread_stack_mem);
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    if (no_errors(rtcan_h))
    {
        rtcan_osal_status_t os_status = rtcan_os_thread_create(&rtcan_h->rx_thread,
                                                               "RTCAN Rx Thread",
                                                               rtcan_rx_thread_entry,
                                                               (void*) rtcan_h,
                                                               config->thread_priority,
                                                               config->rx_thread_stack_size,
                                                               config->rx_thread_stack_mem);
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    /* Configure CAN filters from passed array */
    if (no_errors(rtcan_h) && (config->filters != NULL))
    {
        for (uint32_t i = 0U; i < config->filter_count; i++)
        {
            HAL_StatusTypeDef hal_status = HAL_CAN_ConfigFilter(rtcan_h->hcan,
                                                                &config->filters[i]);
            add_error_if(hal_status != HAL_OK, RTCAN_ERROR_INIT, rtcan_h);
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
    if (rtcan_h == NULL)
    {
        return RTCAN_ERROR;
    }

    /* Start CAN peripheral interrupts */
    if (no_errors(rtcan_h))
    {
        const uint32_t notifs = CAN_IT_TX_MAILBOX_EMPTY
                                | CAN_IT_RX_FIFO0_MSG_PENDING
                                | CAN_IT_RX_FIFO1_MSG_PENDING
                                | CAN_IT_ERROR
                                | CAN_IT_BUSOFF
                                | CAN_IT_ERROR_PASSIVE
                                | CAN_IT_ERROR_WARNING;

        HAL_StatusTypeDef hal_status = HAL_CAN_ActivateNotification(rtcan_h->hcan,
                                                                    notifs);
        add_error_if(hal_status != HAL_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    /* Start the CAN peripheral */
    if (no_errors(rtcan_h))
    {
        HAL_StatusTypeDef hal_status = HAL_CAN_Start(rtcan_h->hcan);
        add_error_if(hal_status != HAL_OK, RTCAN_ERROR_INIT, rtcan_h);
    }

    if (no_errors(rtcan_h))
    {
        uint32_t retries = 10000U;
        while ((HAL_CAN_GetState(rtcan_h->hcan) != HAL_CAN_STATE_LISTENING) && (retries > 0U))
        {
            rtcan_os_yield();
            retries--;
        }
        add_error_if(retries == 0U, RTCAN_ERROR_INIT, rtcan_h);
    }

    if (no_errors(rtcan_h))
    {
        atomic_store(&rtcan_h->started, true);
    }

    return create_status(rtcan_h);
}

//=================================================================== tx service

/**
 * @brief       Transmits a CAN message using the RTCAN service
 *
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   msg_ptr     Pointer to message to transmit
 */
rtcan_status_t rtcan_transmit(rtcan_handle_t* rtcan_h, const rtcan_msg_t* msg_ptr)
{
    if ((rtcan_h == NULL) || (msg_ptr == NULL) || (!atomic_load(&rtcan_h->started)))
    {
        return RTCAN_ERROR;
    }

    rtcan_osal_status_t os_status = rtcan_os_queue_send(rtcan_h->tx_queue,
                                                        (const void*) msg_ptr,
                                                        RTCAN_OS_NO_WAIT);

    /* Queue full is a transient flow-control condition — return error without
       poisoning the handle so subsequent transmits can succeed */
    if (os_status != RTCAN_OS_OK)
    {
        return RTCAN_ERROR;
    }

    return RTCAN_OK;
}

/**
 * @brief       Transmit mailbox callback
 *
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_h       CAN handle passed to HAL callback
 */
rtcan_status_t rtcan_handle_tx_mailbox_callback(rtcan_handle_t* rtcan_h,
                                                const CAN_HandleTypeDef* can_h)
{
    if ((rtcan_h == NULL) || (can_h == NULL))
    {
        return RTCAN_ERROR;
    }

    if (rtcan_h->hcan == can_h)
    {
        rtcan_osal_status_t os_status = rtcan_os_sem_release(rtcan_h->tx_mailbox_sem);
        add_error_if(os_status != RTCAN_OS_OK, RTCAN_ERROR_INTERNAL, rtcan_h);
    }

    return create_status(rtcan_h);
}

/**
 * @brief       Internal transmit for RTCAN service thread
 * 
 * @param[in]   rtcan_h         RTCAN handle
 * @param[in]   identifier      CAN identifier
 * @param[in]   data_ptr        Pointer to data to transmit
 * @param[in]   data_length     Length of data to transmit
 * @param[in]   extended        Flag showing whether this is an extended frame
 */
static rtcan_status_t transmit_internal(rtcan_handle_t* rtcan_h,
                                        uint32_t identifier,
                                        const uint8_t* data_ptr,
                                        uint32_t data_length,
                                        const bool extended)
{
    if ((rtcan_h == NULL) || (data_ptr == NULL) || (data_length == 0U) || (data_length > 8U))
    {
        if (rtcan_h != NULL)
        {
            atomic_fetch_or(&rtcan_h->err, RTCAN_ERROR_ARG);
        }
        return RTCAN_ERROR;
    }

    rtcan_osal_status_t os_status = rtcan_os_sem_acquire(rtcan_h->tx_mailbox_sem,
                                                         RTCAN_OS_WAIT_FOREVER);
    if (os_status != RTCAN_OS_OK)
    {
        atomic_fetch_or(&rtcan_h->err, RTCAN_ERROR_INTERNAL);
        return RTCAN_ERROR;
    }

    /* Create Tx Header */
    CAN_TxHeaderTypeDef header = {0};
    header.RTR = CAN_RTR_DATA;
    header.DLC = data_length;

    if (extended)
    {
        header.IDE = CAN_ID_EXT;
        header.ExtId = identifier;
    }
    else
    {
        header.IDE = CAN_ID_STD;
        header.StdId = identifier;
    }

    /* Send message to mailbox */
    uint32_t tx_mailbox = 0U;
    HAL_StatusTypeDef hal_status = HAL_CAN_AddTxMessage(rtcan_h->hcan,
                                                        &header,
                                                        data_ptr,
                                                        &tx_mailbox);

    if (hal_status != HAL_OK)
    {
        atomic_fetch_or(&rtcan_h->err, RTCAN_ERROR_INTERNAL);
        /* Release the mailbox semaphore since adding message failed */
        (void) rtcan_os_sem_release(rtcan_h->tx_mailbox_sem);
    }

    return create_status(rtcan_h);
}

/**
 * @brief       Entry function for RTCAN transmit service thread
 */
static void rtcan_tx_thread_entry(void* arg)
{
    rtcan_handle_t* rtcan_h = (rtcan_handle_t*) arg;

    while (1)
    {
        rtcan_msg_t message;
        rtcan_osal_status_t os_status = rtcan_os_queue_receive(rtcan_h->tx_queue,
                                                               (void*) &message,
                                                               RTCAN_OS_WAIT_FOREVER);

        if (os_status == RTCAN_OS_OK)
        {
            (void) transmit_internal(rtcan_h,
                                     message.identifier,
                                     message.data,
                                     message.length,
                                     message.extended);
        }
    }
}

//================================================================ subscriptions

/**
 * @brief       Adds a subscriber which will receive notifications of incoming
 *              CAN messages via an rtcan_queue_t
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_id      CAN ID to receive notification for (Standard 11-bit ID)
 * @param[in]   queue_ptr   Destination to receive messages
 */
rtcan_status_t rtcan_subscribe(rtcan_handle_t* rtcan_h,
                               uint32_t can_id, 
                               rtcan_queue_t queue_ptr)
{
    if ((rtcan_h == NULL) || (queue_ptr == NULL) || (can_id >= 2048U))
    {
        return RTCAN_ERROR;
    }

    rtcan_osal_status_t os_status = rtcan_os_sem_acquire(rtcan_h->subscriber_mutex,
                                                         RTCAN_OS_WAIT_FOREVER);
    if (os_status != RTCAN_OS_OK)
    {
        atomic_fetch_or(&rtcan_h->err, RTCAN_ERROR_INTERNAL);
        return RTCAN_ERROR;
    }

    /* Check if already subscribed to prevent duplicates */
    rtcan_subscriber_t* sub = rtcan_h->subscriber_lut[can_id];
    while (sub != NULL)
    {
        if (sub->queue_ptr == queue_ptr)
        {
            (void) rtcan_os_sem_release(rtcan_h->subscriber_mutex);
            return RTCAN_OK; /* Already subscribed */
        }
        sub = sub->next_subscriber_ptr;
    }

    /* Find a free subscriber node in the static pool */
    rtcan_subscriber_t* new_sub = NULL;
    for (uint32_t i = 0U; i < RTCAN_MAX_SUBSCRIBERS; i++)
    {
        if (!rtcan_h->subscriber_pool[i].in_use)
        {
            new_sub = &rtcan_h->subscriber_pool[i];
            break;
        }
    }

    if (new_sub == NULL)
    {
        atomic_fetch_or(&rtcan_h->err, RTCAN_ERROR_MEMORY_FULL);
        (void) rtcan_os_sem_release(rtcan_h->subscriber_mutex);
        return RTCAN_ERROR;
    }

    /* Configure node */
    new_sub->queue_ptr = queue_ptr;
    new_sub->next_subscriber_ptr = NULL;
    new_sub->in_use = true;

    /* Add node to standard ID lookup table */
    if (rtcan_h->subscriber_lut[can_id] == NULL)
    {
        rtcan_h->subscriber_lut[can_id] = new_sub;
    }
    else
    {
        sub = rtcan_h->subscriber_lut[can_id];
        while (sub->next_subscriber_ptr != NULL)
        {
            sub = sub->next_subscriber_ptr;
        }
        sub->next_subscriber_ptr = new_sub;
    }

    (void) rtcan_os_sem_release(rtcan_h->subscriber_mutex);
    return RTCAN_OK;
}

/**
 * @brief       Removes a subscriber, preventing memory leaks in static pools
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_id      CAN ID associated with subscriber
 * @param[in]   queue_ptr   Queue to identify subscriber
 */
rtcan_status_t rtcan_unsubscribe(rtcan_handle_t* rtcan_h,
                                 uint32_t can_id,
                                 rtcan_queue_t queue_ptr)
{
    if ((rtcan_h == NULL) || (queue_ptr == NULL) || (can_id >= 2048U))
    {
        return RTCAN_ERROR;
    }

    rtcan_osal_status_t os_status = rtcan_os_sem_acquire(rtcan_h->subscriber_mutex,
                                                         RTCAN_OS_WAIT_FOREVER);
    if (os_status != RTCAN_OS_OK)
    {
        atomic_fetch_or(&rtcan_h->err, RTCAN_ERROR_INTERNAL);
        return RTCAN_ERROR;
    }

    rtcan_subscriber_t* sub = rtcan_h->subscriber_lut[can_id];
    if (sub == NULL)
    {
        (void) rtcan_os_sem_release(rtcan_h->subscriber_mutex);
        return RTCAN_ERROR; /* Not found */
    }

    rtcan_subscriber_t* prev = NULL;
    bool found = false;

    while (sub != NULL)
    {
        if (sub->queue_ptr == queue_ptr)
        {
            found = true;
            if (prev == NULL)
            {
                rtcan_h->subscriber_lut[can_id] = sub->next_subscriber_ptr;
            }
            else
            {
                prev->next_subscriber_ptr = sub->next_subscriber_ptr;
            }

            /* Reset the pool node and mark it as free */
            sub->queue_ptr = NULL;
            sub->next_subscriber_ptr = NULL;
            sub->in_use = false;
            break;
        }
        prev = sub;
        sub = sub->next_subscriber_ptr;
    }

    (void) rtcan_os_sem_release(rtcan_h->subscriber_mutex);
    return found ? RTCAN_OK : RTCAN_ERROR;
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
    if ((rtcan_h == NULL) || (can_h == NULL) || (rtcan_h->hcan != can_h))
    {
        return RTCAN_ERROR;
    }

    rtcan_msg_t* msg_ptr = NULL;
    rtcan_osal_status_t alloc_status = rtcan_os_block_allocate(rtcan_h->rx_msg_pool,
                                                               (void**) &msg_ptr,
                                                               RTCAN_OS_NO_WAIT);
    add_error_if(alloc_status != RTCAN_OS_OK, RTCAN_ERROR_MEMORY_FULL, rtcan_h);

    /* Always drain the FIFO — leaving it non-empty re-triggers the interrupt immediately.
       If no pool block is available, read into a scratch buffer and discard. */
    CAN_RxHeaderTypeDef header = {0};
    uint8_t scratch[8];
    uint8_t* data_buf = (msg_ptr != NULL) ? msg_ptr->data : scratch;

    HAL_StatusTypeDef hal_status = HAL_CAN_GetRxMessage(rtcan_h->hcan,
                                                        rx_fifo,
                                                        &header,
                                                        data_buf);

    if (msg_ptr == NULL)
    {
        return create_status(rtcan_h);
    }

    if (hal_status == HAL_OK)
    {
        if (header.IDE == CAN_ID_EXT)
        {
            msg_ptr->identifier = header.ExtId;
            msg_ptr->extended = true;
        }
        else
        {
            msg_ptr->identifier = header.StdId;
            msg_ptr->extended = false;
        }
        msg_ptr->length = header.DLC;
        atomic_store(&msg_ptr->reference_count, 0U);
    }
    else
    {
        (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
        add_error_if(true, RTCAN_ERROR_INTERNAL, rtcan_h);
        return create_status(rtcan_h);
    }

    /* Post message address to Rx distribution queue */
    rtcan_osal_status_t send_status = rtcan_os_queue_send(rtcan_h->rx_notif_queue,
                                                          (const void*) &msg_ptr,
                                                          RTCAN_OS_NO_WAIT);

    if (send_status != RTCAN_OS_OK)
    {
        (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
    }
    add_error_if(send_status != RTCAN_OS_OK, RTCAN_ERROR_MEMORY_FULL, rtcan_h);

    return create_status(rtcan_h);
}

/**
 * @brief       Call after message received via subscription has been used
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   msg_ptr     Pointer to message
 */
rtcan_status_t rtcan_msg_consumed(rtcan_handle_t* rtcan_h,
                                  rtcan_msg_t* msg_ptr)
{
    if ((rtcan_h == NULL) || (msg_ptr == NULL))
    {
        return RTCAN_ERROR;
    }

    uint32_t prev_count = atomic_load(&msg_ptr->reference_count);
    do {
        if (prev_count == 0U)
        {
            return RTCAN_ERROR;
        }
    } while (!atomic_compare_exchange_weak(&msg_ptr->reference_count, &prev_count, prev_count - 1U));

    if (prev_count == 1U)
    {
        (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
    }

    return RTCAN_OK;
}

/**
 * @brief       Entry function for RTCAN receive service thread
 */
static void rtcan_rx_thread_entry(void* arg)
{
    rtcan_handle_t* rtcan_h = (rtcan_handle_t*) arg;

    while (1)
    {
        /* Wait for incoming message notification */
        rtcan_msg_t* msg_ptr = NULL;
        rtcan_osal_status_t os_status = rtcan_os_queue_receive(rtcan_h->rx_notif_queue,
                                                               (void*) &msg_ptr,
                                                               RTCAN_OS_WAIT_FOREVER);

        if ((os_status == RTCAN_OS_OK) && (msg_ptr != NULL))
        {
            /* Check if the message is a standard ID within bounds */
            if ((!msg_ptr->extended) && (msg_ptr->identifier < 2048U))
            {
                if (rtcan_os_sem_acquire(rtcan_h->subscriber_mutex, RTCAN_OS_WAIT_FOREVER) != RTCAN_OS_OK)
                {
                    (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
                }
                else
                {
                    rtcan_subscriber_t* subscriber_ptr = rtcan_h->subscriber_lut[msg_ptr->identifier];
                    uint32_t subscriber_count = 0U;

                    /* 1. Count subscribers first */
                    rtcan_subscriber_t* sub = subscriber_ptr;
                    while (sub != NULL)
                    {
                        subscriber_count++;
                        sub = sub->next_subscriber_ptr;
                    }

                    if (subscriber_count > 0U)
                    {
                        /* Set reference count before posting to queues to avoid race conditions */
                        atomic_store(&msg_ptr->reference_count, subscriber_count);

                        /* 2. Dispatch to subscribers */
                        sub = subscriber_ptr;
                        while (sub != NULL)
                        {
                            rtcan_osal_status_t queue_status = rtcan_os_queue_send(sub->queue_ptr,
                                                                                   &msg_ptr,
                                                                                   RTCAN_OS_NO_WAIT);

                            if (queue_status != RTCAN_OS_OK)
                            {
                                uint32_t prev_count = atomic_load(&msg_ptr->reference_count);
                                do {
                                    if (prev_count == 0U)
                                    {
                                        break;
                                    }
                                } while (!atomic_compare_exchange_weak(&msg_ptr->reference_count,
                                                                       &prev_count,
                                                                       prev_count - 1U));
                                if (prev_count == 1U)
                                {
                                    (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
                                }
                            }

                            sub = sub->next_subscriber_ptr;
                        }
                    }
                    else
                    {
                        /* No subscribers, release block */
                        (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
                    }

                    rtcan_osal_status_t rel_status = rtcan_os_sem_release(rtcan_h->subscriber_mutex);
                    add_error_if(rel_status != RTCAN_OS_OK, RTCAN_ERROR_INTERNAL, rtcan_h);
                }
            }
            else
            {
                /* Extended IDs or out-of-bounds IDs are not supported in the LUT, release block */
                (void) rtcan_os_block_release(rtcan_h->rx_msg_pool, msg_ptr);
            }
        }
    }
}

//======================================================================== error

/**
 * @brief       Handles HAL CAN errors and releases mailbox semaphore if needed
 * 
 * @param[in]   rtcan_h     RTCAN handle
 * @param[in]   can_h       CAN handle
 */
rtcan_status_t rtcan_handle_hal_error(rtcan_handle_t* rtcan_h,
                                       CAN_HandleTypeDef* can_h)
{
    if ((rtcan_h == NULL) || (can_h == NULL) || (rtcan_h->hcan != can_h))
    {
        return RTCAN_ERROR;
    }

    /* Reset the error code in the HAL handle.
       TX semaphore is released by the abort callback, not here — releasing it
       in both places causes a double-release on every NART TX failure.
       WARNING: HAL_CAN_TxMailboxAbortCallback MUST be routed to
       rtcan_handle_tx_mailbox_callback. If it is not, tx_mailbox_sem leaks
       one count per aborted transmission and will eventually deadlock the
       tx thread after three such events. */
    rtcan_h->hcan->ErrorCode = HAL_CAN_ERROR_NONE;

    return RTCAN_OK;
}

//====================================================================== utility

/**
 * @brief       Returns the error code
 *
 * @param[in]   rtcan_h   RTCAN handle
 */
uint32_t rtcan_get_error(rtcan_handle_t* rtcan_h)
{
    if (rtcan_h == NULL)
    {
        return RTCAN_ERROR_ARG;
    }
    return atomic_load(&rtcan_h->err);
}

/**
 * @brief       Returns true if the RTCAN instance has encountered no error
 */
static bool no_errors(const rtcan_handle_t* rtcan_h)
{
    return (atomic_load(&rtcan_h->err) == RTCAN_ERROR_NONE);
}

/**
 * @brief       Create a status code based on the current error state
 */
static rtcan_status_t create_status(rtcan_handle_t* rtcan_h)
{
    return (no_errors(rtcan_h)) ? RTCAN_OK : RTCAN_ERROR;
}
