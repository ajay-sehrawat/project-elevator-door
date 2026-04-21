# Design Rationale — p7-elevator-door

## 1. Dual-Core Race Condition Strategy

### The Problem

The ESP32 is a true dual-core SMP processor. FreeRTOS on ESP-IDF does not
disable preemption between cores — a task on Core 0 and a task on Core 1
can execute simultaneously. Without explicit synchronisation, any shared
data structure can be corrupted mid-read or mid-write.

### Our Solution: Three Layered Defences

#### Layer 1 — Core Pinning by Domain

| Domain          | Core | Tasks                                         |
|-----------------|------|-----------------------------------------------|
| Safety-Critical | 1    | SafetyTask (P5), DispatcherTask (P4), ControlTask (P4) |
| Comms/Peripherals | 0  | HAL_RX (P3), HAL_TX (P2), LoggerTask (P1)    |

By pinning all safety-critical execution to Core 1, we bound the set of
tasks that can preempt the FSM to a known, small set — all of which share
the same core and are therefore FreeRTOS-scheduled (not simultaneous).
A task on Core 1 cannot be preempted by another Core 1 task while holding
a mutex — only a higher-priority Core 1 task can preempt it.

#### Layer 2 — Single-Writer FSM Mutex

`g_fsm_mutex` protects the entire `fsm_ctx_t` struct. The rule is absolute:

- **One writer**: only `fsm_control_task` writes `s_fsm_ctx`.
- **Readers**: any task calling `fsm_get_state()` acquires the mutex.
- **The Safety Monitor never writes FSM state directly.** It enqueues a
  synthetic `system_event_t` (e.g. `EVT_COMM_TIMEOUT`) to the Dispatcher
  queue. The FSM consumes it in its own context — eliminating concurrent
  writes entirely.

This is the "single-writer, event-driven" pattern. It removes the need for
complex multi-reader/writer locking schemes.

#### Layer 3 — C11 Atomics for Cross-Core Sensor State

The Safety Monitor's communication watchdog timestamp and raw sensor state
are written by HAL_RX (Core 0) and read by SafetyTask (Core 1) — two truly
concurrent cores. For these single 32-bit and 8-bit values, we use
`_Atomic uint32_t` / `_Atomic uint8_t` instead of a mutex to avoid:

- Priority inversion between SafetyTask (P5) and HAL_RX (P3).
- Unbounded blocking of the Safety task's tight 10 ms loop.

A mutex would work, but it introduces latency on the hot path of the
highest-priority task. C11 atomics compile to `LDREX/STREX` instructions on
Xtensa LX6, which are hardware-guaranteed to be indivisible.

#### Motor Commands Outside Mutex Scope

`hal_motor_open()` / `hal_motor_close()` / `hal_motor_stop()` are called
**after** `xSemaphoreGive(g_fsm_mutex)` in every code path. This ensures the
HAL call cannot hold the mutex for an unbounded duration, preventing the
Safety task from starving while waiting to read FSM state.

---

## 2. NVS Fault Persistence (NFR-3, SR-4)

### Why NVS?

Flash-backed NVS survives power loss, watchdog resets, and software panics.
A RAM variable does not. The requirement states: "if a fatal fault is
triggered, the MCU loses power, and upon reboot the FSM must not act as if
no fault ever occurred."

### Write Path

1. `fsm_enter_fault()` is called on any fatal fault condition.
2. It immediately calls `hal_motor_stop()` (safety-first).
3. It then calls `fault_nvs_write(fault_code)`:
   - `nvs_set_u8(NVS_KEY_FAULT_STATE, 1)` — marks fault as active.
   - `nvs_set_u8(NVS_KEY_FAULT_CODE, code)` — stores the specific code.
   - `nvs_commit()` — flushes to flash (blocking ~1–10 ms).
4. Transmits `$STATE,STATE=FAULT` to the Supervisor.

The NVS commit is the only potentially slow call in the fault path. It occurs
**after** motor stop and FSM state update — safety is never delayed by flash.

### Boot Read Path

`app_main()` calls `fault_nvs_read()` **before** spawning any tasks:
- If `NVS_KEY_FAULT_STATE == 0` or key absent → clean boot, normal homing.
- If `NVS_KEY_FAULT_STATE == 1` → `EVT_FAULT_PERSIST` is injected into the
  FSM queue after tasks start. The FSM boots into FAULT immediately.

### Clear Path

`fault_nvs_clear()` is called only after `EVT_CMD_RESET` successfully
transitions the FSM back to homing. This ensures the fault is considered
resolved only after explicit operator acknowledgement AND a physical re-homing
routine confirms the door is in a known safe state.

### NVS Partition Resilience

If the NVS partition is found corrupt on boot (`ESP_ERR_NVS_NO_FREE_PAGES`
or `ESP_ERR_NVS_NEW_VERSION_FOUND`), `app_main` erases and reinitialises it.
The loss of fault state in this scenario is an acceptable trade-off against
a completely unbootable system. The Supervisor's own state log provides the
audit trail in this edge case.

---

## 3. Event-Driven Architecture vs. Polling

The specification forbids polling for application logic. Our implementation
is strictly event-driven:

- The HAL RX task blocks on `uart_read_bytes()` with a `COMM_TIMEOUT_MS` timeout.
  If no bytes arrive, it injects `EVT_COMM_TIMEOUT` — it never busy-waits.
- The FSM task blocks on `xQueueReceive()` with a `STATE_REPORT_INTERVAL_MS`
  timeout. The timeout is only used to trigger periodic `$STATE` transmissions,
  not to check for conditions — conditions arrive as events.
- The Safety Monitor loops every 10 ms with `vTaskDelay()`. This is the one
  deliberate exception: a tight 10 ms periodic safety check is appropriate for
  a watchdog that must react faster than any application-level event can be
  processed. 10 ms << 100 ms COMM_TIMEOUT and << 20 ms obstruction deadline.

---

## 4. Debounce State Machine

Mechanical contact bounce on elevator door limit switches can last 5–20 ms.
The HAL implements a per-sensor debounce state machine:

```
raw_sample ──→ [changed?] ──YES──→ reset timer, mark pending
                   │
                  NO
                   │
              [pending?] ──YES──→ [elapsed ≥ DEBOUNCE_MS?] ──YES──→ DISPATCH
                   │                        │
                  NO                       NO
                   │                        │
               (discard)              (keep waiting)
```

Debounce is performed PER SENSOR, independently. A noisy obstruction sensor
does not affect the fully-open limit switch debounce window.

---

## 5. CRC-8 Frame Validation (NFR-5)

Every UART frame is validated with a Dallas/Maxim CRC-8 (polynomial 0x31,
init 0xFF). The CRC byte is appended as a two-hex-character suffix:
`$CMD,TYPE=OPEN,3A\n`

Frames with CRC errors receive `$NACK`. Valid frames receive `$ACK`.
The simulation accepts frames without a CRC suffix (development mode) with
a `LOGW` warning — this allows raw `$CMD,TYPE=OPEN` commands during bring-up
without modifying application code.

---

## 6. Backpressure Handling (NFR-5)

The UART TX queue (depth `UART_TX_QUEUE_DEPTH = 8`) uses a drop-oldest
strategy: if the queue is full when a new message arrives, the oldest entry
is dequeued and discarded, then the new message is enqueued. This guarantees:
- The system never blocks the FSM waiting for UART TX.
- Memory usage is bounded (no unbounded growth under Supervisor flooding).
- The Supervisor receives the most recent state, not stale queued messages.

---

## 7. Stack Sizing & Monitoring

All task stacks are over-provisioned by ~40% above worst-case measured usage.
`uxTaskGetStackHighWaterMark()` is reported every `STACK_REPORT_INTERVAL_MS`
(5 s) by the Logger task. If any task's high-water-mark approaches zero,
the stack size constant in `config.h` must be increased and the rationale
updated. The `vApplicationStackOverflowHook` provides a hard safety net —
it halts the offending task and lets the TWDT reset the system.