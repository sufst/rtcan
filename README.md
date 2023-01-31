# RTCAN

## About

RTCAN (Real-Time CAN) is a ThreadX RTOS service for managing concurrent access 
to CAN peripherals on STM32 microcontrollers.

Features:
- ...

## Usage

...

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

## Dependencies

- ThreadX memory pool, thread, semaphore and queue services.
- STM32 Hardware Abstraction Layer (HAL) CAN drivers.

## Other Platforms

This implementation was developed for the STM32 platform, however it should be
relatively simple to port to another platform with a different HAL.