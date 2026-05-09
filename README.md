[![Open in Codespaces](https://classroom.github.com/assets/launch-codespace-2972f46106e565e64193e422d61a12cf1da4916b45550586e14ef0a7c637dd04.svg)](https://classroom.github.com/open-in-codespaces?assignment_repo_id=22323812)

# Elevator Door Safety Controller (CS G523 Embedded Software Project)

An event-driven Finite State Machine (FSM) implementation for an elevator door controller, built for the ESP32 platform using FreeRTOS. This project focuses on high reliability, safety-critical execution, and robust hardware interaction.

## Project Overview

This project implements an **embedded safety controller for an elevator door system**.  
The main goal of the controller is to **open and close elevator doors safely** while handling user requests, sensor inputs, and fault conditions.

The project is developed as part of **CS G523 – Software for Embedded Systems** and focuses on **safety-critical embedded software behavior**, not mechanical or hardware design.

All safety-related decisions are made **locally by the embedded controller** based on inputs received through a simulated UART interface.

---
<img width="1408" height="768" alt="architecture" src="https://github.com/user-attachments/assets/4b38630b-7811-4d97-afaf-0fad79eb4301" />

---

## Problem Statement

Elevator doors are a **safety-critical subsystem**.  
Unsafe door movement can cause passenger injury or damage to the elevator system.

This project ensures that:
- Doors never move in an unsafe manner
- Obstructions are handled correctly
- Missing or invalid sensor data does not lead to dangerous behavior
- Conflicting or repeated commands are handled safely

The controller always behaves in a **predictable and conservative** way, especially during fault conditions.

## System Description

The system acts as a **door safety controller** with the following responsibilities:

- Receive door-related commands and sensor data over **UART**
- Decide when to **OPEN**, **CLOSE**, or **STOP** the elevator door
- Prevent unsafe door motion under all operating conditions
- Detect and handle abnormal situations such as:
  - Door obstructions
  - Missing or invalid sensor data
  - Conflicting user commands
- Move to a **safe stopped state** when behavior is uncertain

Only door behavior is handled by this system.  
Elevator car movement is assumed to be managed by another controller.

## System Boundary

### Inside the System
- Embedded control logic
- Door state management (open, closed, opening, closing, stopped)
- Safety interlocks and checks
- UART message processing
- Fault detection and safe-state handling

### Outside the System
- Door motor and mechanical components
- Elevator car movement system
- Passengers and physical buttons
- Power supply and backup power
- Physical sensors (simulated using UART)

## Inputs and Outputs

### Inputs (UART Simulated)
- Door open request
- Door close request
- Obstruction detection signal
- Door fully open sensor
- Door fully closed sensor
- Emergency or fault signals

UART messages may be **delayed, duplicated, missing, or invalid**, and the controller must still behave safely.

### Outputs
- `OPEN` – command to open the door
- `CLOSE` – command to close the door
- `STOP` – command to immediately stop door movement

## Core Features

- **Dual-Core Architecture**: Intelligently distributes tasks across the ESP32's dual cores. Safety-critical tasks (FSM Control, Safety Monitor) are pinned to Core 1, while communication and logging tasks are pinned to Core 0.
- **Race Condition Prevention**: Employs a layered defense strategy including a single-writer FSM mutex and C11 atomics (`_Atomic uint32_t` / `_Atomic uint8_t`) for cross-core sensor state communication without priority inversion.
- **NVS Fault Persistence**: Uses Non-Volatile Storage (NVS) to ensure that if a fatal fault triggers a reset or power loss, the system boots immediately into a safe fault state. Faults can only be cleared with an explicit operator acknowledgment (`EVT_CMD_RESET`).
- **Event-Driven Design**: Eliminates polling loops in the application logic. Uses FreeRTOS queues for task communication, timeouts, and hardware events.
- **Hardware Debounce Logic**: Integrates a custom timer-based debounce state machine in the Hardware Abstraction Layer (HAL) to filter mechanical limit switch noise.
- **UART Frame Validation**: All UART messages are validated with a Dallas/Maxim CRC-8 checksum to prevent corrupt commands from triggering unwanted states.
- **Backpressure Handling**: UART queues implement a drop-oldest strategy to prevent memory exhaustion and blocking during periods of high message volume.

## Safety Focus

Safety is the **highest priority** of this project.  
At a high level, the controller ensures that:

- The door does not close when an obstruction is present
- Door movement stops immediately when unsafe conditions are detected
- Sensor failures result in a safe stopped state
- Unnecessary or repeated door movements are avoided

Exact timing values and recovery strategies are intentionally not fixed and are justified through design decisions.

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

## Course Context & Project Goals

By completing this project, the goals are to demonstrate:
- Clear system boundary definition
- Well-written, testable functional and safety requirements
- Correct handling of normal and fault scenarios
- State-machine-based reasoning for embedded systems
- Safe and deterministic behavior under abnormal conditions

This project is part of **CS G523 – Software for Embedded Systems**, emphasizing:
- Requirements engineering
- Safety-critical design
- Event-driven control logic
- Fault handling in embedded systems
