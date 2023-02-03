/***************************************************************************
 * @file   rtcan.h
 * @author Tim Brewis (@t-bre, tab1g19@soton.ac.uk)
 * @brief  RTOS wrapper around CAN bus
 ***************************************************************************/

#ifndef RTCAN_H
#define RTCAN_H

#include "tx_api.h"

#include "rtcan.h"

#include "can.h"

/*
 * error codes
 */
#define RTCAN_ERROR_NONE 0x00000000U // no error
#define RTCAN_ERROR_INIT 0x00000001U // failed to start service
#define RTCAN_ERROR_ARG  0x00000002U // invalid argument
#define RTCAN_ERROR_MEMORY_FULL 0x00000004U // not enough memory for operation
#define RTCAN_ERROR_INTERNAL 0x80000000U // internal error

#ifndef RTCAN_HASHMAP_SIZE
    #define RTCAN_HASHMAP_SIZE  100 // default, number of items
#endif

#ifndef RTCAN_RX_MSG_POOL_SIZE
    #define RTCAN_RX_MSG_POOL_SIZE 100 // default, number of items
#endif

#ifndef RTCAN_SUBSCRIBER_POOL_SIZE
    #define RTCAN_SUBSCRIBER_POOL_SIZE (250 * sizeof(ULONG))
    // in bytes, must be multiple of sizeof(ULONG)
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
typedef struct _rtcan_subscriber_t {

    /**
     * @brief   Receive queue for subscriber
     */
    TX_QUEUE* queue_ptr;

    /**
     * @brief   Next subscriber for given CAN ID
     */
    struct _rtcan_subscriber_t* next_subscriber_ptr;

} rtcan_subscriber_t;

/**
 * @brief   subscriber node for hashmap of subscribers
 */
typedef struct _rtcan_hashmap_node_t {

    /**
     * @brief   CAN identifier
     */
    uint32_t can_id;

    /**
     * @brief   Next node for separate chaining in event of collision
     */
    struct _rtcan_hashmap_node_t* chained_node_ptr;

    /**
     * @brief   Singly linked list of subscribers for given CAN ID
     */
    rtcan_subscriber_t* first_subscriber_ptr;

} rtcan_hashmap_node_t;

/**
 * @brief   RTCAN message
 *
 * @note    This must have a size that is a multiple of sizeof(ULONG) for use
 *          with TX_QUEUE
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

} rtcan_msg_t;

/**
 * @brief   Reference counted RTCAN message
 */
typedef struct {

    rtcan_msg_t message;
    volatile uint32_t reference_count;

} rtcan_refcounted_msg_t;

/*
 * queue sizing constants
 */
#define RTCAN_TX_QUEUE_LENGTH    100
#define RTCAN_TX_QUEUE_ITEM_SIZE (sizeof(rtcan_msg_t) / sizeof(ULONG))
#define RTCAN_TX_QUEUE_SIZE      (RTCAN_TX_QUEUE_LENGTH * RTCAN_TX_QUEUE_ITEM_SIZE)

#define RTCAN_RX_NOTIF_QUEUE_LENGTH     10
#define RTCAN_RX_NOTIF_QUEUE_ITEM_SIZE  1 // one pointer = 1x ULONG
#define RTCAN_RX_NOTIF_QUEUE_SIZE       (RTCAN_RX_NOTIF_QUEUE_LENGTH * RTCAN_RX_NOTIF_QUEUE_ITEM_SIZE)

/**
 * @brief RTCAN handle
 */
typedef struct
{
    /**
     * @brief   Transmit service thread
     */
    TX_THREAD tx_thread;

    /**
     * @brief   Receive service thread
     */
    TX_THREAD rx_thread;

    /**
     * @brief   CAN handle dedicated to this instance
     */
    CAN_HandleTypeDef* hcan;

    /**
     * @brief   Transmit message box semaphore
     *
     * @details This holds a count of the number of currently available
     *          transmit message boxes and should be posted to when a message
     *          box becomes available again (e.g. in interrupt). See docs for
     *          rtcan_handle_tx_mailbox_callback().
     */
    TX_SEMAPHORE tx_mailbox_sem;

    /**
     * @brief   Receive notification queue
     * 
     * @details Posted to in CAN interrupt. Items contain pointer to received
     *          message allocated from Rx byte pool
     */
    TX_QUEUE rx_notif_queue;

    /**
     * @brief   Receive notification queue memory area
     */
    ULONG rx_notif_queue_mem[RTCAN_RX_NOTIF_QUEUE_SIZE];

    /**
     * @brief   Transmit queue
     * 
     * @details Transmit queueing is currently FIFO based
     */
    TX_QUEUE tx_queue;

    /**
     * @brief   Transmit queue memory area
     */
    ULONG tx_queue_mem[RTCAN_TX_QUEUE_SIZE];

    /**
     * @brief   Hashmap of subscribers
     */
    rtcan_hashmap_node_t* subscriber_map[RTCAN_HASHMAP_SIZE];

    /**
     * @brief   Byte pool for subscriber data
     */
    TX_BYTE_POOL subscriber_pool;

    /**
     * @brief   Memory area for subscriber data pool
     */
    ULONG subscriber_pool_mem[RTCAN_SUBSCRIBER_POOL_SIZE / sizeof(ULONG)];

    /**
     * @brief   Block pool for received messages
     */
    TX_BLOCK_POOL rx_msg_pool;

    /**
     * @brief   Memory area for received message pool
     */
    rtcan_refcounted_msg_t rx_msg_pool_mem[RTCAN_RX_MSG_POOL_SIZE];

    /**
     * @brief   Current error code
     */
    uint32_t err;

} rtcan_handle_t;

/*
 * function prototypes
 */
rtcan_status_t rtcan_init(rtcan_handle_t* rtcan_h,
                          CAN_HandleTypeDef* hcan,
                          ULONG priority,
                          TX_BYTE_POOL* stack_pool_ptr);

rtcan_status_t rtcan_start(rtcan_handle_t* rtcan_h);

rtcan_status_t rtcan_transmit(rtcan_handle_t* rtcan_h, rtcan_msg_t* msg_ptr);

rtcan_status_t rtcan_handle_tx_mailbox_callback(rtcan_handle_t* rtcan_h,
                                                const CAN_HandleTypeDef* can_h);

rtcan_status_t rtcan_handle_rx_it(rtcan_handle_t* rtcan_h, 
                                  const CAN_HandleTypeDef* can_h,
                                  const uint32_t rx_fifo);

rtcan_status_t rtcan_subscribe(rtcan_handle_t* rtcan_h,
                               uint32_t can_id, 
                               TX_QUEUE* queue_ptr);

uint32_t rtcan_get_error(rtcan_handle_t* rtcan_h);

#endif