# RTCAN

## About

RTCAN (Real-Time CAN) is a [ThreadX RTOS](https://learn.microsoft.com/en-us/azure/rtos/threadx/overview-threadx) 
service for managing concurrent access to CAN peripherals on [STM32 microcontrollers](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html).

Features:
- Background thread based ThreadX service.
- FIFO transmit queuing.
- "Subscription" style receiving functionality.

Planned features:
- Automatic CAN filter configuration management.
- Priority queueing for transmissions.

Not currently supported:
- STM32 FDCAN HAL.
- Extended CAN identifiers.

## Dependencies

- 32 bit STM32 microcontroller.
- ThreadX memory pool, thread, semaphore and queue services.
- STM32 Hardware Abstraction Layer (HAL) CAN drivers.

## Adding to a Project

### Submodule

Add this repository as a submodule using:

```sh
git submodule add https://github.com/sufst/rtcan
```

Make sure to change directories to the location you want the submodule to exist
in the project source tree. Note that the use of submodules will require the 
following commands to be run when cloning a project for the first time:

```sh
git submodule init
git submodule update
```

For more information on submodules, see the [Git submodule documentation](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

### Build System

RTCAN consists of one header file (`inc/rtcan.h`) which should be added to the
include path for a project (or just to specific files requiring RTCAN), 
and one source file (`src/rtcan.c`) which should be compiled by the build system
in question. Make sure the [RTCAN dependencies](#dependencies) are satisfied.

## Usage

### Initialisation

RTCAN is provided for a CAN peripheral by an instance of `rtcan_handle_t` which
is initialised with the function `rtcan_init()`. Each RTCAN instance manages
one CAN peripheral and has two background service threads: one for transmitting
and one for receiving.

### Transmitting

The `rtcan_transmit()` function uses a simple FIFO queueing system to transmit
messages with the CAN peripheral. This provides a way of ensuring that there is 
not contention for the CAN peripheral by multiple threads. 

To use the transmit service, CAN Tx interrupts must be enabled and the 
`HAL_CAN_TxMailbox<N>CompleteCallback` must be implemented to call 
`rtcan_handle_tx_mailbox_callback` (for all `N`). For example, for the
HAL callback for Tx mailbox 1:

```c
static rtcan_handle_t rtcan;

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_tx_mailbox_callback(&rtcan, can_h);
}
```

### Receiving and Subscriptions

Receiving functionality in RTCAN is based around a "publisher" / "subscriber"
model in which application threads can register their interest in receiving
CAN messages with a particular ID through the `rtcan_subscribe()` function.
Threads must provide a queue as an endpoint for messages where queue items have
size `TX_1_ULONG`. Incoming CAN messages are published to the queue by the RTCAN
service, where each queue item is a pointer to the received message represented 
as an `rtcan_msg_t` struct. Once a subscriber has finished with a message, it 
**must** call the `rtcan_consumed()` function to indicate this to the service.
Internally `rtcan_msg_t` is a reference counted, dynamically allocated data
structure which is distributed to all the subscribers of a given CAN ID. 
As such, subscribers must treat this message as **read only** and should
not modify the `reference_count` field.

To use the receive service, CAN Rx interrupts must be enabled and the 
`HAL_CAN_RxFifo<N>MsgPendingCallback` must be implemented to call 
`rtcan_handle_rx_it` (for all `N`). For example, for the
HAL callback for Rx FIFO 1:

```c
static rtcan_handle_t rtcan;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_rx_it(&rtcan, can_h, 1);
}
```

### Error Codes

All functions in the RTCAN API return a status code (`rtcan_status_t`) 
indicating the RTCAN error state. `RTCAN_OK` indicates no error and 
`RTCAN_ERROR` indicates an error. The specific error can be checked with the
return value of `rtcan_get_error()`. This design is based around the conventions
of the STM32 HAL.

### Example

The following example uses RTCAN to subscribe to CAN messages with ID `0x100`
and re-transmit the data as a CAN message with ID `0x101`.

```c
#include "tx_api.h"
#include "rtcan.h"
#include "can.h"
#include "string.h" // for memcpy()

#define RTCAN_THREAD_PRIORITY   3
#define MY_THREAD_PRIORITY      4
#define MY_THREAD_STACK_SIZE    1024

static rtcan_handle_t rtcan;
static TX_THREAD my_thread;
static TX_QUEUE rx_queue;
static ULONG rx_queue_mem[10];

static void my_thread_entry(ULONG thread_input);

/**
 * initialise a thread which will use RTCAN services
 */
void init_my_thread(TX_BYTE_POOL* app_mem_pool)
{
    // initialise RTCAN instance
    rtcan_init(&rtcan, 
               &hcan1, 
               RTCAN_THREAD_PRIORITY, 
               app_mem_pool);

    // allocate memory for thread
    void* stack_ptr;
    tx_byte_allocate(app_mem_pool,
                     &stack_ptr,
                     MY_THREAD_STACK_SIZE,
                     TX_NO_WAIT);

    // create thread
    tx_thread_create(&my_thread,
                     my_thread_entry,
                     NULL,
                     stack_ptr,
                     MY_THREAD_STACK_SIZE,
                     MY_THREAD_PRIORITY,
                     MY_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE,
                     TX_AUTO_START);

    // subscribe to a message
    tx_queue_create(&rx_queue,
                    "My Rx Queue",
                    TX_1_ULONG,
                    rx_queue_mem,
                    sizeof(rx_queue));

    rtcan_subscribe(&rtcan, 0x100, &rx_queue);

    // start the RTCAN service
    rtcan_start(&rtcan);
}

/**
 * thread which uses RTCAN services
 */
void my_thread_entry(ULONG thread_input)
{
    (void) thread_input; // unused

    while (1)
    {
        // wait for an item to enter the rx queue
        rtcan_msg_t* msg_ptr;

        tx_queue_receive(&rx_queue, 
                         (void*) &msg_ptr, 
                         TX_WAIT_FOREVER);

        // make a copy of the message but change the ID to 0x101
        rtcan_msg_t new_message;
        new_message.identifier = 0x101;
        new_message.length = message_ptr->length;
        memcpy((void*) new_message.data, (void*) message_ptr->data, msg_ptr->length);

        // transmit the copied message
        rtcan_transmit(&rtcan, &new_message);

        // mark the original received message as consumed
        rtcan_msg_consumed(&rtcan, msg_ptr);
    }
}

/**
 * implement HAL CAN callbacks to call RTCAN handler functions
 * 
 * note: specific callbacks depend on CAN capabilities of target STM32
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_tx_mailbox_callback(&rtcan, can_h);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_tx_mailbox_callback(&rtcan, can_h);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_tx_mailbox_callback(&rtcan, can_h);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_rx_it(&rtcan, can_h, 0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* can_h)
{
    rtcan_handle_rx_it(&rtcan, can_h, 1);
}

```

> Checking of return codes has been omitted here for brevity. In practice,
  you should always check the return codes of both ThreadX and RTCAN 
  functions.

## Other Platforms

This implementation was developed for the STM32 platform, however it should be
relatively simple to port to another platform with a different HAL. A similar
system could also be implemented with another RTOS so long as it provides 
equivalent services to ThreadX.
