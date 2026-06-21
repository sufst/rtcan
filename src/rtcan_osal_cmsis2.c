/***************************************************************************
 * @file   rtcan_osal_cmsis2.c
 * @author Antigravity (Google DeepMind team)
 * @brief  CMSIS-RTOS v2 implementation of RTCAN OSAL
 ***************************************************************************/

#include "rtcan_osal.h"
#include <cmsis_os2.h>

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

    osThreadAttr_t attr = {0};
    attr.name = name;
    attr.priority = (osPriority_t)priority;
    attr.stack_size = (uint32_t)stack_size;
    if (stack_mem != NULL)
    {
        attr.stack_mem = stack_mem;
    }

    osThreadId_t tid = osThreadNew((osThreadFunc_t)entry, arg, &attr);
    if (tid == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    *thread = (rtcan_thread_t)tid;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_queue_create(rtcan_queue_t* queue,
                                          const char* name,
                                          size_t item_size,
                                          size_t capacity,
                                          void* queue_mem,
                                          size_t queue_mem_size)
{
    if ((queue == NULL) || (item_size == 0U) || (capacity == 0U))
    {
        return RTCAN_OS_ERROR;
    }

    osMessageQueueAttr_t attr = {0};
    attr.name = name;
    if (queue_mem != NULL)
    {
        attr.mq_mem = queue_mem;
        attr.mq_size = (uint32_t)queue_mem_size;
    }

    osMessageQueueId_t mq = osMessageQueueNew((uint32_t)capacity, (uint32_t)item_size, &attr);
    if (mq == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    *queue = (rtcan_queue_t)mq;
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

    osStatus_t status = osMessageQueuePut((osMessageQueueId_t)queue, item, 0U, timeout);
    if (status == osOK)
    {
        return RTCAN_OS_OK;
    }
    else if (status == osErrorTimeout || status == osErrorResource)
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

    osStatus_t status = osMessageQueueGet((osMessageQueueId_t)queue, item, NULL, timeout);
    if (status == osOK)
    {
        return RTCAN_OS_OK;
    }
    else if (status == osErrorTimeout || status == osErrorResource)
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
    if (sem == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    osSemaphoreAttr_t attr = {0};
    attr.name = name;

    osSemaphoreId_t sid = osSemaphoreNew(max_count, initial_count, &attr);
    if (sid == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    *sem = (rtcan_sem_t)sid;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_sem_acquire(rtcan_sem_t sem,
                                         uint32_t timeout)
{
    if (sem == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    osStatus_t status = osSemaphoreAcquire((osSemaphoreId_t)sem, timeout);
    if (status == osOK)
    {
        return RTCAN_OS_OK;
    }
    else if (status == osErrorTimeout || status == osErrorResource)
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

    osStatus_t status = osSemaphoreRelease((osSemaphoreId_t)sem);
    return (status == osOK) ? RTCAN_OS_OK : RTCAN_OS_ERROR;
}

rtcan_osal_status_t rtcan_os_block_pool_create(rtcan_block_pool_t* pool,
                                               const char* name,
                                               size_t block_size,
                                               size_t block_count,
                                               void* pool_mem,
                                               size_t pool_mem_size)
{
    if ((pool == NULL) || (block_size == 0U) || (block_count == 0U))
    {
        return RTCAN_OS_ERROR;
    }

    osMemoryPoolAttr_t attr = {0};
    attr.name = name;
    if (pool_mem != NULL)
    {
        attr.mp_mem = pool_mem;
        attr.mp_size = (uint32_t)pool_mem_size;
    }

    osMemoryPoolId_t mp = osMemoryPoolNew((uint32_t)block_count, (uint32_t)block_size, &attr);
    if (mp == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    *pool = (rtcan_block_pool_t)mp;
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

    void* ptr = osMemoryPoolAlloc((osMemoryPoolId_t)pool, timeout);
    if (ptr == NULL)
    {
        /* osMemoryPoolAlloc returns NULL for both timeout and error; distinguish
           by timeout value: WAIT_FOREVER only returns NULL on a genuine error. */
        return (timeout == RTCAN_OS_WAIT_FOREVER) ? RTCAN_OS_ERROR : RTCAN_OS_TIMEOUT;
    }

    *block_ptr = ptr;
    return RTCAN_OS_OK;
}

rtcan_osal_status_t rtcan_os_block_release(rtcan_block_pool_t pool,
                                           void* block_ptr)
{
    if ((pool == NULL) || (block_ptr == NULL))
    {
        return RTCAN_OS_ERROR;
    }

    osStatus_t status = osMemoryPoolFree((osMemoryPoolId_t)pool, block_ptr);
    return (status == osOK) ? RTCAN_OS_OK : RTCAN_OS_ERROR;
}

void rtcan_os_yield(void)
{
    (void) osThreadYield();
}
