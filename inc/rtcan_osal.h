/***************************************************************************
 * @file   rtcan_osal.h
 * @author Antigravity (Google DeepMind team)
 * @brief  Operating System Abstraction Layer (OSAL) for RTCAN driver
 ***************************************************************************/

#ifndef RTCAN_OSAL_H
#define RTCAN_OSAL_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief OSAL operation status
 */
typedef enum
{
    RTCAN_OS_OK = 0,
    RTCAN_OS_ERROR = 1,
    RTCAN_OS_TIMEOUT = 2
} rtcan_osal_status_t;

/* Infinite timeout constant */
#define RTCAN_OS_WAIT_FOREVER 0xFFFFFFFFU
#define RTCAN_OS_NO_WAIT      0x00000000U

/* Opaque pointer types for OS resources */
typedef void* rtcan_thread_t;
typedef void* rtcan_queue_t;
typedef void* rtcan_sem_t;
typedef void* rtcan_block_pool_t;

/* Thread entry function pointer type */
typedef void (*rtcan_thread_entry_t)(void* arg);

/**
 * @brief           Create a new background service thread
 * 
 * @param[out]      thread      Pointer to the created thread handle
 * @param[in]       name        Thread name string
 * @param[in]       entry       Thread entry point function
 * @param[in]       arg         Argument passed to the thread entry function
 * @param[in]       priority    Thread priority
 * @param[in]       stack_size  Stack size in bytes
 * @param[in]       stack_mem   Pointer to pre-allocated stack memory (optional, can be NULL)
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_thread_create(rtcan_thread_t* thread,
                                           const char* name,
                                           rtcan_thread_entry_t entry,
                                           void* arg,
                                           uint32_t priority,
                                           size_t stack_size,
                                           void* stack_mem);

/**
 * @brief           Create a message queue
 * 
 * @param[out]      queue           Pointer to the created queue handle
 * @param[in]       name            Queue name string
 * @param[in]       item_size       Size of each message item in bytes
 * @param[in]       capacity        Maximum number of items the queue can hold
 * @param[in]       queue_mem       Pointer to pre-allocated queue storage (optional, can be NULL)
 * @param[in]       queue_mem_size  Size of the pre-allocated queue storage in bytes
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_queue_create(rtcan_queue_t* queue,
                                          const char* name,
                                          size_t item_size,
                                          size_t capacity,
                                          void* queue_mem,
                                          size_t queue_mem_size);

/**
 * @brief           Send an item to a message queue
 * 
 * @param[in]       queue       Queue handle
 * @param[in]       item        Pointer to the item to copy into the queue
 * @param[in]       timeout     Timeout duration in ticks (or RTCAN_OS_NO_WAIT/RTCAN_OS_WAIT_FOREVER)
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_queue_send(rtcan_queue_t queue,
                                        const void* item,
                                        uint32_t timeout);

/**
 * @brief           Receive an item from a message queue
 * 
 * @param[in]       queue       Queue handle
 * @param[out]      item        Pointer to destination buffer to copy item into
 * @param[in]       timeout     Timeout duration in ticks (or RTCAN_OS_NO_WAIT/RTCAN_OS_WAIT_FOREVER)
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_queue_receive(rtcan_queue_t queue,
                                           void* item,
                                           uint32_t timeout);

/**
 * @brief           Create a counting semaphore
 * 
 * @param[out]      sem             Pointer to the created semaphore handle
 * @param[in]       name            Semaphore name string
 * @param[in]       initial_count   Initial value of the semaphore
 * @param[in]       max_count       Maximum value of the semaphore
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_sem_create(rtcan_sem_t* sem,
                                        const char* name,
                                        uint32_t initial_count,
                                        uint32_t max_count);

/**
 * @brief           Acquire a semaphore (decrement count)
 * 
 * @param[in]       sem         Semaphore handle
 * @param[in]       timeout     Timeout duration in ticks (or RTCAN_OS_NO_WAIT/RTCAN_OS_WAIT_FOREVER)
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_sem_acquire(rtcan_sem_t sem,
                                         uint32_t timeout);

/**
 * @brief           Release a semaphore (increment count)
 * 
 * @param[in]       sem         Semaphore handle
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_sem_release(rtcan_sem_t sem);

/**
 * @brief           Create a fixed-size block memory pool
 * 
 * @param[out]      pool            Pointer to the created block pool handle
 * @param[in]       name            Pool name string
 * @param[in]       block_size      Size of each memory block in bytes
 * @param[in]       block_count     Number of memory blocks in the pool
 * @param[in]       pool_mem        Pointer to pre-allocated block pool storage (optional, can be NULL)
 * @param[in]       pool_mem_size   Size of the pre-allocated block pool storage in bytes
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_block_pool_create(rtcan_block_pool_t* pool,
                                               const char* name,
                                               size_t block_size,
                                               size_t block_count,
                                               void* pool_mem,
                                               size_t pool_mem_size);

/**
 * @brief           Allocate a memory block from a pool
 * 
 * @param[in]       pool        Block pool handle
 * @param[out]      block_ptr   Pointer to destination pointer where the allocated block address will be written
 * @param[in]       timeout     Timeout duration in ticks (or RTCAN_OS_NO_WAIT/RTCAN_OS_WAIT_FOREVER)
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_block_allocate(rtcan_block_pool_t pool,
                                            void** block_ptr,
                                            uint32_t timeout);

/**
 * @brief           Release a memory block back to its pool
 * 
 * @param[in]       pool        Block pool handle
 * @param[in]       block_ptr   Pointer to the memory block to release
 * 
 * @return          rtcan_osal_status_t
 */
rtcan_osal_status_t rtcan_os_block_release(rtcan_block_pool_t pool,
                                           void* block_ptr);

#endif /* RTCAN_OSAL_H */
