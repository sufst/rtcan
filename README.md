# RTCAN

RTCAN (Real-Time CAN) is a portable, memory-safe C11 driver library for managing concurrent access to CAN peripherals on STM32 microcontrollers using a publisher/subscriber model. 

Designed for safety-critical Formula Student systems, the library is **RTOS-agnostic** and adheres to safety guidelines (such as MISRA C:2012) by utilizing **100% static allocation** with zero dynamic memory overhead.

---

## Key Features

- **Operating System Abstraction Layer (OSAL):** Decoupled from any specific RTOS. Native wrappers are provided for:
  - **CMSIS-RTOS v2** (e.g., FreeRTOS, RTX5, Zephyr) in `src/rtcan_osal_cmsis2.c`.
  - **ThreadX** in `src/rtcan_osal_threadx.c` (retaining backward-compatibility with a 100% static control block pool).
- **100% Static Allocation (MISRA C:2012 compliant):** All queues, message pools, and subscriber nodes are allocated statically at compile-time. There is no heap fragmentation or Out-of-Memory risk.
- **Deterministic O(1) Lookup Table (LUT):** Replaced separate-chained collision hashmaps with a direct Lookup Table (2048 entries) for standard 11-bit CAN IDs, ensuring constant-time dispatch on the receive path.
- **Race-Free Concurrent Dispatch:** Multi-threaded publisher/subscriber model using thread-safe C11 atomics (`stdatomic.h`) to handle message reference counting safely across queues.
- **Unsubscribe API:** Allows threads to dynamically unsubscribe from message queues, safely recycling subscriber slots back into the static pool.
- **Decoupled Application Logic:** Filter configurations are passed dynamically to `rtcan_init()` instead of being hardcoded in the driver.

---

## Dependencies

- **C11 compiler** (uses `<stdatomic.h>`).
- **32-bit STM32 Microcontroller** (uses STM32 HAL CAN drivers).
- **An RTOS** supported by the OSAL backends (CMSIS-RTOS v2 or ThreadX).

---

## Adding to a Project

### Option A: Using CMake `FetchContent` (Recommended)

To avoid managing Git submodules, add this to your main project's `CMakeLists.txt`:

```cmake
include(FetchContent)

# Declare RTCAN
FetchContent_Declare(
    rtcan
    GIT_REPOSITORY https://github.com/sufst/rtcan.git
    GIT_TAG        main # Or use a specific tag/commit hash
)
FetchContent_MakeAvailable(rtcan)

# Link it to your executable
target_link_libraries(your_firmware_target PRIVATE rtcan)
```

### Option B: Using Git Submodules
If you prefer submodules:
1. Clone the submodule into your project directory:
   ```sh
   git submodule add https://github.com/sufst/rtcan.git third_party/rtcan
   ```
2. Include the header directory `inc/` in your include paths.
3. Add `src/rtcan.c` to your build sources.
4. Add the appropriate OSAL wrapper to your build sources:
   - For CMSIS-RTOS v2: `src/rtcan_osal_cmsis2.c`
   - For ThreadX: `src/rtcan_osal_threadx.c`

> **ThreadX only:** define `RTCAN_OSAL_THREADX` in your build (e.g. `-DRTCAN_OSAL_THREADX`). This switches `RTCAN_OS_QUEUE_MEM_SIZE` to a ThreadX-specific formula that accounts for the `TX_QUEUE` control block embedded at the front of each statically-allocated queue buffer. Without it, small queues (capacity ≤ 3 for pointer-sized items) will be undersized and queue creation will fail at init.

---

## API Usage Guide

### 1. Initialization
Declare your global RTCAN handle and configure stack allocations, priority, and filters:

```c
#include "rtcan.h"

static rtcan_handle_t rtcan;
static uint64_t rtcan_tx_stack[128]; // 1024 bytes (8-byte aligned)
static uint64_t rtcan_rx_stack[128]; // 1024 bytes (8-byte aligned)

/* Define hardware filters for the STM32 CAN peripheral */
static const CAN_FilterTypeDef my_filters[] = {
    {
        .FilterActivation = ENABLE,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterIdHigh = (0x100 << 5U),      // Filter for VCU simulated command (0x100)
        .FilterIdLow = (0x200 << 5U),       // Filter for Inverter telemetry (0x200)
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterMode = CAN_FILTERMODE_IDLIST,
        .FilterScale = CAN_FILTERSCALE_16BIT,
        .FilterBank = 0
    }
};

void app_can_init(void)
{
    rtcan_config_t config = {
        .thread_priority = 3,
        .tx_thread_stack_size = sizeof(rtcan_tx_stack),
        .tx_thread_stack_mem = rtcan_tx_stack,
        .rx_thread_stack_size = sizeof(rtcan_rx_stack),
        .rx_thread_stack_mem = rtcan_rx_stack,
        .filters = my_filters,
        .filter_count = sizeof(my_filters) / sizeof(my_filters[0])
    };

    /* Initialize the RTCAN driver instance */
    rtcan_init(&rtcan, &hcan1, &config);

    /* Start the background threads and activate CAN interrupts */
    rtcan_start(&rtcan);
}
```

### 2. Subscribing & Unsubscribing
Declare a queue in your application thread, subscribe to standard IDs, and read from the queue. When finished with a message, release it using `rtcan_msg_consumed`.

```c
#include "rtcan.h"

static rtcan_queue_t my_rx_queue;
static uint8_t queue_storage[10U * sizeof(rtcan_msg_t*)];

void app_thread(void* arg)
{
    /* Create an OSAL queue to receive pointers to rtcan_msg_t structs */
    rtcan_os_queue_create(&my_rx_queue, "App Queue", sizeof(rtcan_msg_t*), 10U, queue_storage, sizeof(queue_storage));

    /* Subscribe to message ID 0x100 */
    rtcan_subscribe(&rtcan, 0x100, my_rx_queue);

    while (1)
    {
        rtcan_msg_t* rx_msg = NULL;
        /* Block waiting for an incoming message */
        if (rtcan_os_queue_receive(my_rx_queue, &rx_msg, RTCAN_OS_WAIT_FOREVER) == RTCAN_OS_OK)
        {
            /* Process data ... */
            uint8_t state = rx_msg->data[0];

            /* Free the message reference back to the static pool */
            rtcan_msg_consumed(&rtcan, rx_msg);
        }
    }

    /* Unsubscribe if the thread exits or changes roles */
    rtcan_unsubscribe(&rtcan, 0x100, my_rx_queue);
}
```

### 3. Transmitting
Populate an `rtcan_msg_t` block and pass it to `rtcan_transmit()` to queue it in the background Tx loop:

```c
void send_status(void)
{
    rtcan_msg_t tx_msg = {
        .identifier = 0x201,
        .extended = false,
        .length = 4,
        .data = {0xAA, 0xBB, 0xCC, 0xDD}
    };

    rtcan_transmit(&rtcan, &tx_msg);
}
```

---

## Integrating with `can-defs` (DBC Code Generation)

In SUFST firmware projects, standard practice is to use code-generated C structures and pack/unpack helper functions compiled from the central [can-defs](https://github.com/sufst/can-defs) repository. 

Using code generation alongside RTCAN ensures type safety and eliminates hardcoded CAN IDs and bit-shifting:

### Example: Unpacking a Received Message
```c
#include "rtcan.h"
#include "can_database.h" /* Generated from can-defs DBC */

void app_process_thread(void* arg)
{
    rtcan_msg_t* rx_msg = NULL;
    
    if (rtcan_os_queue_receive(my_rx_queue, &rx_msg, RTCAN_OS_WAIT_FOREVER) == RTCAN_OS_OK)
    {
        /* Structure generated by cantools/can-defs */
        struct can_database_vcu_state_t decoded_vcu;
        
        /* Unpack raw bytes into type-safe fields */
        can_database_vcu_state_unpack(&decoded_vcu, rx_msg->data, rx_msg->length);
        
        /* Use decoded variables */
        uint16_t pedal_position = decoded_vcu.throttle_pedal;
        
        /* Release message block */
        rtcan_msg_consumed(&rtcan, rx_msg);
    }
}
```

### Example: Packing and Transmitting a Message
```c
#include "rtcan.h"
#include "can_database.h"

void send_bms_telemetry(void)
{
    struct can_database_bms_status_t bms_status = {
        .accumulator_voltage = 580U,
        .state_of_charge = 85U,
        .error_flags = 0x00
    };

    rtcan_msg_t tx_msg;
    tx_msg.identifier = CAN_DATABASE_BMS_STATUS_FRAME_ID;
    tx_msg.extended = false;
    tx_msg.length = CAN_DATABASE_BMS_STATUS_LENGTH;

    /* Pack structured data into the raw CAN message buffer */
    can_database_bms_status_pack(tx_msg.data, &bms_status, sizeof(tx_msg.data));

    rtcan_transmit(&rtcan, &tx_msg);
}
```

---

## Mandatory Interrupt Service Routine (ISR) Mappings

To hook the RTCAN engine up to the STM32 HAL callbacks, you must forward the callbacks inside your `stm32xx_it.c` or application callback code.

### 1. Transmit Interrupts
```c
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan) {
    rtcan_handle_tx_mailbox_callback(&rtcan, hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan) {
    rtcan_handle_tx_mailbox_callback(&rtcan, hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan) {
    rtcan_handle_tx_mailbox_callback(&rtcan, hcan);
}
```

### 2. Receive Interrupts
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    rtcan_handle_rx_it(&rtcan, hcan, CAN_RX_FIFO0);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    rtcan_handle_rx_it(&rtcan, hcan, CAN_RX_FIFO1);
}
```

### 3. Error Interrupts
```c
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    rtcan_handle_hal_error(&rtcan, hcan);
}
```
