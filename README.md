# Elevator Door FSM

An event-driven Finite State Machine (FSM) implementation for an elevator door controller, built for the ESP32 platform using FreeRTOS. This project focuses on high reliability, safety-critical execution, and robust hardware interaction.

## Core Features

- **Dual-Core Architecture**: Intelligently distributes tasks across the ESP32's dual cores. Safety-critical tasks (FSM Control, Safety Monitor) are pinned to Core 1, while communication and logging tasks are pinned to Core 0.
- **Race Condition Prevention**: Employs a layered defense strategy including a single-writer FSM mutex and C11 atomics (`_Atomic uint32_t` / `_Atomic uint8_t`) for cross-core sensor state communication without priority inversion.
- **NVS Fault Persistence**: Uses Non-Volatile Storage (NVS) to ensure that if a fatal fault triggers a reset or power loss, the system boots immediately into a safe fault state. Faults can only be cleared with an explicit operator acknowledgment (`EVT_CMD_RESET`).
- **Event-Driven Design**: Eliminates polling loops in the application logic. Uses FreeRTOS queues for task communication, timeouts, and hardware events.
- **Hardware Debounce Logic**: Integrates a custom timer-based debounce state machine in the Hardware Abstraction Layer (HAL) to filter mechanical limit switch noise.
- **UART Frame Validation**: All UART messages are validated with a Dallas/Maxim CRC-8 checksum to prevent corrupt commands from triggering unwanted states.
- **Backpressure Handling**: UART queues implement a drop-oldest strategy to prevent memory exhaustion and blocking during periods of high message volume.

## Project Structure

```text
├── src/
│   ├── app_main.c            # Application entry point and task spawning
│   ├── fsm/door_fsm.c        # Main state machine logic
│   ├── dispatcher/           # Event queue routing and dispatching
│   ├── safety/               # Safety monitor and watchdogs
│   ├── nvs/                  # Non-volatile storage management
│   ├── hal/                  # Hardware Abstraction Layer
│   └── logger/               # Async logging task
├── include/                  # Shared header files
├── test/                     # Unit and integration tests
├── platformio.ini            # PlatformIO configuration
├── CMakeLists.txt            # CMake build configuration
└── DESIGN.md                 # Detailed architecture rationale
```

## Setup & Building

This project uses [PlatformIO](https://platformio.org/) for building and dependency management. 

1. Install PlatformIO Core or the VSCode Extension.
2. Clone the repository.
3. Build the project:
   ```bash
   pio run
   ```
4. Flash to your ESP32 device and monitor the serial output:
   ```bash
   pio run --target upload --target monitor
   ```

## Architecture Details

For an in-depth breakdown of the design rationale, race condition mitigations, stack monitoring, and state models, refer to the [DESIGN.md](DESIGN.md) document included in the repository.
