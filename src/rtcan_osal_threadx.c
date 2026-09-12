/***************************************************************************
 * @file   rtcan_osal_threadx.c
 * @author Antigravity (Google DeepMind team)
 * @brief  ThreadX implementation of RTCAN OSAL
 ***************************************************************************/

#include "rtcan_osal.h"
#include <tx_api.h>

#ifndef RTCAN_MAX_INSTANCES
#define RTCAN_MAX_INSTANCES 2U
#endif

#ifndef RTCAN_THREADX_DEFAULT_STACK_SIZE
#define RTCAN_THREADX_DEFAULT_STACK_SIZE 1024U
#endif

typedef struct {
    rtcan_thread_entry_t entry;
    void*                arg;
} rtcan_thread_shim_t;

typedef struct {
    TX_SEMAPHORE tx_sem;
    ULONG        ceiling;
} rtcan_sem_entry_t;

/* Static control blocks allocated for ThreadX backend */
static TX_THREAD s_threads[RTCAN_MAX_INSTANCES * 2U];
static rtcan_sem_entry_t s_sems[RTCAN_MAX_INSTANCES * 2U];
static TX_BLOCK_POOL s_pools[RTCAN_MAX_INSTANCES];
static uint8_t s_default_stacks[RTCAN_MAX_INSTANCES * 2U][RTCAN_THREADX_DEFAULT_STACK_SIZE];
static rtcan_thread_shim_t s_shims[RTCAN_MAX_INSTANCES * 2U];

static VOID thread_entry_shim(ULONG idx)
{
    if (idx >= (RTCAN_MAX_INSTANCES * 2U))
    {
        return;
    }
    s_shims[idx].entry(s_shims[idx].arg);
}

static uint32_t s_thread_count = 0U;
static uint32_t s_sem_count = 0U;
static uint32_t s_pool_count = 0U;

rtcan_osal_status_t rtcan_os_thread_create(rtcan_thread_t* thread,
                                           const char* name,
                                           rtcan_thread_entry_t entry,
                                           void* arg,
                                           uint32_t priority,
                                           size_t stack_size,
                                           void* stack_mem)
{
    if ((thread == NULL) || (entry == NULL))
    {
        return RTCAN_OS_ERROR;
    }

    if (s_thread_count >= (RTCAN_MAX_INSTANCES * 2U))
    {
        return RTCAN_OS_ERROR;
    }

    TX_THREAD* tx_thread = &s_threads[s_thread_count];

    if (stack_mem == NULL)
    {
        stack_mem = s_default_stacks[s_thread_count];
        stack_size = RTCAN_THREADX_DEFAULT_STACK_SIZE;
    }

    s_shims[s_thread_count].entry = entry;
    s_shims[s_thread_count].arg   = arg;
    ULONG shim_idx = (ULONG)s_thread_count;

    UINT status = tx_thread_create(tx_thread,
                                   (CHAR*)name,
                                   thread_entry_shim,
                                   shim_idx,
                                   stack_mem,
                                   (ULONG)stack_size,
                                   (UINT)priority,
                                   (UINT)priority,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        return RTCAN_OS_ERROR;
    }
    s_thread_count++;

    *thread = (rtcan_thread_t)tx_thread;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_queue_create(rtcan_queue_t* queue,
                                          const char* name,
                                          size_t item_size,
                                          size_t capacity,
                                          void* queue_mem,
                                          size_t queue_mem_size)
{
    if ((queue == NULL) || (queue_mem == NULL) || (item_size == 0U) || (capacity == 0U))
    {
        return RTCAN_OS_ERROR;
    }

    if (queue_mem_size <= sizeof(TX_QUEUE))
    {
        return RTCAN_OS_ERROR;
    }

    /* TX_QUEUE control block lives at the front of the caller's buffer;
       item data follows immediately after. The buffer must be uint32_t-
       aligned (guaranteed by the uint32_t[] array types used at call sites),
       which satisfies TX_QUEUE's alignment requirement on 32-bit ARM. */
    TX_QUEUE* tx_queue = (TX_QUEUE*)queue_mem;
    void*     data     = (uint8_t*)queue_mem + sizeof(TX_QUEUE);
    ULONG     data_sz  = (ULONG)(queue_mem_size - sizeof(TX_QUEUE));

    UINT message_size = (UINT)((item_size + sizeof(ULONG) - 1U) / sizeof(ULONG));
    if (message_size == 0U)
    {
        message_size = 1U;
    }

    UINT status = tx_queue_create(tx_queue,
                                  (CHAR*)name,
                                  message_size,
                                  data,
                                  data_sz);

    if (status != TX_SUCCESS)
    {
        return RTCAN_OS_ERROR;
    }

    *queue = (rtcan_queue_t)tx_queue;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_queue_send(rtcan_queue_t queue,
                                        const void* item,
                                        uint32_t timeout)
{
    if ((queue == NULL) || (item == NULL))
    {
        return RTCAN_OS_ERROR;
    }

    UINT status = tx_queue_send((TX_QUEUE*)queue, (VOID*)item, (ULONG)timeout);
    if (status == TX_SUCCESS)
    {
        return RTCAN_OS_OK;
    }
    else if (status == TX_QUEUE_FULL)
    {
        return RTCAN_OS_TIMEOUT;
    }
    else
    {
        return RTCAN_OS_ERROR;
    }
}

rtcan_osal_status_t rtcan_os_queue_receive(rtcan_queue_t queue,
                                           void* item,
                                           uint32_t timeout)
{
    if ((queue == NULL) || (item == NULL))
    {
        return RTCAN_OS_ERROR;
    }

    UINT status = tx_queue_receive((TX_QUEUE*)queue, item, (ULONG)timeout);
    if (status == TX_SUCCESS)
    {
        return RTCAN_OS_OK;
    }
    else if (status == TX_QUEUE_EMPTY)
    {
        return RTCAN_OS_TIMEOUT;
    }
    else
    {
        return RTCAN_OS_ERROR;
    }
}

rtcan_osal_status_t rtcan_os_sem_create(rtcan_sem_t* sem,
                                        const char* name,
                                        uint32_t initial_count,
                                        uint32_t max_count)
{
    (void)max_count;

    if (sem == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    if (s_sem_count >= (RTCAN_MAX_INSTANCES * 2U))
    {
        return RTCAN_OS_ERROR;
    }

    rtcan_sem_entry_t* entry = &s_sems[s_sem_count];

    UINT status = tx_semaphore_create(&entry->tx_sem, (CHAR*)name, (ULONG)initial_count);
    if (status != TX_SUCCESS)
    {
        return RTCAN_OS_ERROR;
    }
    entry->ceiling = (ULONG)max_count;
    s_sem_count++;

    *sem = (rtcan_sem_t)entry;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_sem_acquire(rtcan_sem_t sem,
                                         uint32_t timeout)
{
    if (sem == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    rtcan_sem_entry_t* entry = (rtcan_sem_entry_t*)sem;
    UINT status = tx_semaphore_get(&entry->tx_sem, (ULONG)timeout);
    if (status == TX_SUCCESS)
    {
        return RTCAN_OS_OK;
    }
    else if (status == TX_NO_INSTANCE)
    {
        return RTCAN_OS_TIMEOUT;
    }
    else
    {
        return RTCAN_OS_ERROR;
    }
}

rtcan_osal_status_t rtcan_os_sem_release(rtcan_sem_t sem)
{
    if (sem == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    rtcan_sem_entry_t* entry = (rtcan_sem_entry_t*)sem;
    UINT status = tx_semaphore_ceiling_put(&entry->tx_sem, entry->ceiling);
    if (status == TX_SUCCESS)
    {
        return RTCAN_OS_OK;
    }
    else if (status == TX_CEILING_EXCEEDED)
    {
        return RTCAN_OS_ERROR;
    }
    else
    {
        return RTCAN_OS_ERROR;
    }
}

rtcan_osal_status_t rtcan_os_block_pool_create(rtcan_block_pool_t* pool,
                                               const char* name,
                                               size_t block_size,
                                               size_t block_count,
                                               void* pool_mem,
                                               size_t pool_mem_size)
{
    (void)block_count;
    if ((pool == NULL) || (pool_mem == NULL) || (block_size == 0U))
    {
        return RTCAN_OS_ERROR;
    }

    if (s_pool_count >= RTCAN_MAX_INSTANCES)
    {
        return RTCAN_OS_ERROR;
    }

    TX_BLOCK_POOL* tx_pool = &s_pools[s_pool_count];

    UINT status = tx_block_pool_create(tx_pool,
                                       (CHAR*)name,
                                       (ULONG)block_size,
                                       pool_mem,
                                       (ULONG)pool_mem_size);

    if (status != TX_SUCCESS)
    {
        return RTCAN_OS_ERROR;
    }
    s_pool_count++;

    *pool = (rtcan_block_pool_t)tx_pool;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_block_allocate(rtcan_block_pool_t pool,
                                            void** block_ptr,
                                            uint32_t timeout)
{
    if ((pool == NULL) || (block_ptr == NULL))
    {
        return RTCAN_OS_ERROR;
    }

    UINT status = tx_block_allocate((TX_BLOCK_POOL*)pool, block_ptr, (ULONG)timeout);
    if (status == TX_SUCCESS)
    {
        return RTCAN_OS_OK;
    }
    else if (status == TX_NO_INSTANCE)
    {
        return RTCAN_OS_TIMEOUT;
    }
    else
    {
        return RTCAN_OS_ERROR;
    }
}

rtcan_osal_status_t rtcan_os_block_release(rtcan_block_pool_t pool,
                                           void* block_ptr)
{
    (void)pool;
    if (block_ptr == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    UINT status = tx_block_release(block_ptr);
    return (status == TX_SUCCESS) ? RTCAN_OS_OK : RTCAN_OS_ERROR;
}

void rtcan_os_yield(void)
{
    tx_thread_relinquish();
}
