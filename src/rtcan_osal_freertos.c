/***************************************************************************
 * @file   rtcan_osal_freertos.c
 * @author Antigravity (Google DeepMind team)
 * @brief  FreeRTOS implementation of RTCAN OSAL (CMSIS-RTOS v2 API, using
 *         FreeRTOS's static allocation types directly; not portable to
 *         other CMSIS-RTOS v2 kernels)
 ***************************************************************************/

#include "rtcan_osal.h"
#include <cmsis_os2.h>
#include <FreeRTOS.h>
#include <queue.h>

#if !defined(configSUPPORT_STATIC_ALLOCATION) || (configSUPPORT_STATIC_ALLOCATION == 0)
#error "rtcan_osal_freertos.c requires configSUPPORT_STATIC_ALLOCATION == 1 in FreeRTOSConfig.h (static queue/control-block allocation)"
#endif

#ifndef RTCAN_MAX_INSTANCES
#define RTCAN_MAX_INSTANCES 2U
#endif

#ifndef RTCAN_OSAL_MAX_QUEUES
#define RTCAN_OSAL_MAX_QUEUES ((RTCAN_MAX_INSTANCES * 2U) + 8U)
#endif

static StaticQueue_t s_queue_cbs[RTCAN_OSAL_MAX_QUEUES];
static uint32_t      s_queue_count = 0U;

static StaticQueue_t s_pool_cbs[RTCAN_MAX_INSTANCES];
static void*         s_pool_storage[RTCAN_MAX_INSTANCES][RTCAN_OSAL_MAX_BLOCK_POOL_BLOCKS];
static uint32_t      s_pool_count = 0U;

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
    if ((queue == NULL) || (item_size == 0U) || (capacity == 0U) || (queue_mem == NULL) ||
        (queue_mem_size < (capacity * item_size)))
    {
        return RTCAN_OS_ERROR;
    }

    if (s_queue_count >= RTCAN_OSAL_MAX_QUEUES)
    {
        return RTCAN_OS_ERROR;
    }

    osMessageQueueAttr_t attr = {0};
    attr.name = name;
    attr.cb_mem = &s_queue_cbs[s_queue_count];
    attr.cb_size = (uint32_t)sizeof(s_queue_cbs[0]);
    attr.mq_mem = queue_mem;
    attr.mq_size = (uint32_t)(capacity * item_size);

    osMessageQueueId_t mq = osMessageQueueNew((uint32_t)capacity, (uint32_t)item_size, &attr);
    if (mq == NULL)
    {
        return RTCAN_OS_ERROR;
    }
    s_queue_count++;

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
    if ((pool == NULL) ||
        (block_size == 0U) ||
        (block_count == 0U) ||
        (pool_mem == NULL) ||
        (pool_mem_size < (block_size * block_count)) ||
        (block_count > RTCAN_OSAL_MAX_BLOCK_POOL_BLOCKS))
    {
        return RTCAN_OS_ERROR;
    }

    if (s_pool_count >= RTCAN_MAX_INSTANCES)
    {
        return RTCAN_OS_ERROR;
    }

    osMessageQueueAttr_t attr = {0};
    attr.name = name;
    attr.cb_mem = &s_pool_cbs[s_pool_count];
    attr.cb_size = (uint32_t)sizeof(s_pool_cbs[0]);
    attr.mq_mem = s_pool_storage[s_pool_count];
    attr.mq_size = (uint32_t)(block_count * sizeof(void*));

    osMessageQueueId_t mq = osMessageQueueNew((uint32_t)block_count, sizeof(void*), &attr);
    if (mq == NULL)
    {
        return RTCAN_OS_ERROR;
    }

    uint8_t* base = (uint8_t*) pool_mem;
    for (size_t i = 0U; i < block_count; i++)
    {
        void* block = (void*)(base + (i * block_size));
        if (osMessageQueuePut(mq, &block, 0U, 0U) != osOK)
        {
            (void) osMessageQueueDelete(mq);
            return RTCAN_OS_ERROR;
        }
    }
    s_pool_count++;

    *pool = (rtcan_block_pool_t) mq;
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

    osStatus_t status = osMessageQueueGet((osMessageQueueId_t)pool, block_ptr, NULL, timeout);
    if (status == osOK)
    {
        return RTCAN_OS_OK;
    }
    return (status == osErrorTimeout || status == osErrorResource) ? RTCAN_OS_TIMEOUT : RTCAN_OS_ERROR;
}

rtcan_osal_status_t rtcan_os_block_release(rtcan_block_pool_t pool,
                                           void* block_ptr)
{
    if ((pool == NULL) || (block_ptr == NULL))
    {
        return RTCAN_OS_ERROR;
    }

    osStatus_t status = osMessageQueuePut((osMessageQueueId_t)pool, &block_ptr, 0U, 0U);
    return (status == osOK) ? RTCAN_OS_OK : RTCAN_OS_ERROR;
}

void rtcan_os_yield(void)
{
    (void) osThreadYield();
}
