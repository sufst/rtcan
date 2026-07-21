/***************************************************************************
 * @file   rtcan.h
 * @author Tim Brewis (@t-bre, tab1g19@soton.ac.uk)
 *         Refactored by Antigravity (Google DeepMind team)
 * @brief  RTOS-agnostic wrapper around CAN bus
 ***************************************************************************/

#ifndef RTCAN_H
#define RTCAN_H

#include <can.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#include "rtcan_osal.h"

/*
 * error codes
 */
#define RTCAN_ERROR_NONE        0x00000000U // no error
#define RTCAN_ERROR_INIT        0x00000001U // failed to start service
#define RTCAN_ERROR_ARG         0x00000002U // invalid argument
#define RTCAN_ERROR_MEMORY_FULL 0x00000004U // not enough memory for operation
#define RTCAN_ERROR_INTERNAL    0x80000000U // internal error

#ifndef RTCAN_RX_MSG_POOL_SIZE
#define RTCAN_RX_MSG_POOL_SIZE 1000U // default, number of items
#endif

#ifdef RTCAN_OSAL_MAX_BLOCK_POOL_BLOCKS
_Static_assert(RTCAN_RX_MSG_POOL_SIZE <= RTCAN_OSAL_MAX_BLOCK_POOL_BLOCKS,
               "RTCAN_RX_MSG_POOL_SIZE exceeds the FreeRTOS OSAL backend's "
               "static block pool capacity (RTCAN_OSAL_MAX_BLOCK_POOL_BLOCKS)");
#endif

#ifndef RTCAN_MAX_SUBSCRIBERS
#define RTCAN_MAX_SUBSCRIBERS 32U // default, maximum simultaneous subscriptions
#endif

#ifndef RTCAN_TX_MAILBOX_TIMEOUT_TICKS
#define RTCAN_TX_MAILBOX_TIMEOUT_TICKS 100U // ~100 ms at the project's TX_TIMER_TICKS_PER_SECOND (1000, see tx_user.h)
#endif

/**
 * @brief   RTCAN status
 */
typedef enum
{
    RTCAN_OK,
    RTCAN_ERROR
} rtcan_status_t;

/**
 * @brief   subscriber information for a CAN ID
 */
typedef struct _rtcan_subscriber_t
{
    /**
     * @brief   Receive queue for subscriber
     */
    rtcan_queue_t queue_ptr;

    /**
     * @brief   Next subscriber for given CAN ID
     */
    struct _rtcan_subscriber_t *next_subscriber_ptr;

    /**
     * @brief   Flag indicating whether this slot is currently in use
     */
    bool in_use;

} rtcan_subscriber_t;

/**
 * @brief   RTCAN message
 */
typedef struct
{
    /**
     * @brief   CAN standard identifier for message
     */
    uint32_t identifier;

    /**
     * @brief   Message data buffer
     */
    uint8_t data[8];

    /**
     * @brief   Length of message data in bytes
     */
    uint32_t length;

    /**
     * @brief   Reference count for dynamically allocated messages with multiple
     *          subscribers
     */
    _Atomic uint32_t reference_count;

    /**
     * @brief   Flag showing whether the message is an extended message
     */
    bool extended;

} rtcan_msg_t;

/*
 * queue sizing constants
 */
#define RTCAN_TX_QUEUE_LENGTH 10U
#define RTCAN_RX_NOTIF_QUEUE_LENGTH 32U

/**
 * @brief RTCAN configuration structure
 */
typedef struct
{
    /**
     * @brief   Priority for the background service threads
     */
    uint32_t thread_priority;

    /**
     * @brief   Stack size in bytes for the transmit thread
     */
    size_t tx_thread_stack_size;

    /**
     * @brief   Stack size in bytes for the receive thread
     */
    size_t rx_thread_stack_size;

    /**
     * @brief   Pointer to transmit thread stack memory (optional)
     */
    void* tx_thread_stack_mem;

    /**
     * @brief   Pointer to receive thread stack memory (optional)
     */
    void* rx_thread_stack_mem;

    /**
     * @brief   Array of CAN filters to configure during initialization
     */
    const CAN_FilterTypeDef* filters;

    /**
     * @brief   Number of filters in the array
     */
    uint32_t filter_count;

} rtcan_config_t;

/**
 * @brief RTCAN handle
 */
typedef struct
{
    /**
     * @brief   Transmit service thread
     */
    rtcan_thread_t tx_thread;

    /**
     * @brief   Receive service thread
     */
    rtcan_thread_t rx_thread;

    /**
     * @brief   CAN handle dedicated to this instance
     */
    CAN_HandleTypeDef *hcan;

    /**
     * @brief   Transmit message box semaphore
     */
    rtcan_sem_t tx_mailbox_sem;

    /**
     * @brief   Receive notification queue
     */
    rtcan_queue_t rx_notif_queue;

    /**
     * @brief   Receive notification queue memory area
     */
    uint32_t rx_notif_queue_mem[RTCAN_OS_QUEUE_MEM_SIZE(RTCAN_RX_NOTIF_QUEUE_LENGTH, sizeof(rtcan_msg_t*)) / sizeof(uint32_t)];

    /**
     * @brief   Transmit queue
     */
    rtcan_queue_t tx_queue;

    /**
     * @brief   Transmit queue memory area
     */
    uint32_t tx_queue_mem[RTCAN_OS_QUEUE_MEM_SIZE(RTCAN_TX_QUEUE_LENGTH, sizeof(rtcan_msg_t)) / sizeof(uint32_t)];

    /**
     * @brief   Static pool of subscriber structures
     */
    rtcan_subscriber_t subscriber_pool[RTCAN_MAX_SUBSCRIBERS];

    /**
     * @brief   Lookup Table (LUT) mapping CAN IDs to subscriber linked lists
     */
    rtcan_subscriber_t *subscriber_lut[2048];

    /**
     * @brief   Block pool for received messages
     */
    rtcan_block_pool_t rx_msg_pool;

    /**
     * @brief   Memory area for received message pool
     */
    rtcan_msg_t rx_msg_pool_mem[RTCAN_RX_MSG_POOL_SIZE];

    /**
     * @brief   Mutex protecting subscriber_lut and subscriber_pool
     */
    rtcan_sem_t subscriber_mutex;

    /**
     * @brief   Current error code
     */
    _Atomic uint32_t err;

    /**
     * @brief   Flag indicating whether the service is started
     */
    _Atomic bool started;

} rtcan_handle_t;

/*
 * function prototypes
 */
rtcan_status_t rtcan_init(rtcan_handle_t *rtcan_h,
                          CAN_HandleTypeDef *hcan,
                          const rtcan_config_t *config);

rtcan_status_t rtcan_start(rtcan_handle_t *rtcan_h);

rtcan_status_t rtcan_transmit(rtcan_handle_t *rtcan_h, const rtcan_msg_t *msg_ptr);

rtcan_status_t rtcan_handle_tx_mailbox_callback(rtcan_handle_t *rtcan_h,
                                                const CAN_HandleTypeDef *can_h);

rtcan_status_t rtcan_handle_rx_it(rtcan_handle_t *rtcan_h,
                                  const CAN_HandleTypeDef *can_h,
                                  const uint32_t rx_fifo);

rtcan_status_t rtcan_subscribe(rtcan_handle_t *rtcan_h,
                               uint32_t can_id,
                               rtcan_queue_t queue_ptr);

rtcan_status_t rtcan_unsubscribe(rtcan_handle_t *rtcan_h,
                                 uint32_t can_id,
                                 rtcan_queue_t queue_ptr);

rtcan_status_t rtcan_msg_consumed(rtcan_handle_t *rtcan_h,
                                  rtcan_msg_t *msg_ptr);

rtcan_status_t rtcan_handle_hal_error(rtcan_handle_t *rtcan_h,
                                      CAN_HandleTypeDef *can_h);

uint32_t rtcan_get_error(rtcan_handle_t *rtcan_h);

#endif