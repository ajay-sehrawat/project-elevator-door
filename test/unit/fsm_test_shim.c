/**
 * @file test/unit/fsm_test_shim.c
 * @brief Test shim that exposes internal FSM functions for unit testing.
 *
 * Compiled ONLY when -DFSM_UNIT_TEST is set in the test build.
 * This avoids any #ifdef contamination in the production FSM code.
 *
 * How it works:
 *   1. The test build compiles door_fsm.c WITH -DFSM_UNIT_TEST.
 *   2. When that flag is set, door_fsm.c exposes two symbols:
 *        void fsm_process_event_test(const system_event_t *evt);
 *        void fsm_reset_for_test(void);
 *   3. This shim file declares them extern and calls them from tests.
 *
 * Adding the test-instrumented entry points to door_fsm.c:
 *   Add the block below at the END of door_fsm.c, guarded by #ifdef FSM_UNIT_TEST:
 *
 * ─── Paste into door_fsm.c (bottom) ────────────────────────────────────────
 * #ifdef FSM_UNIT_TEST
 *
 * void fsm_process_event_test(const system_event_t *evt)
 * {
 *     fsm_process_event(evt);
 * }
 *
 * void fsm_reset_for_test(void)
 * {
 *     // Reset context to clean INIT state.
 *     xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
 *     s_fsm_ctx.state              = FSM_STATE_INIT;
 *     s_fsm_ctx.fault_code         = FAULT_NONE;
 *     s_fsm_ctx.emergency_lock     = EMERGENCY_LOCK_OFF;
 *     s_fsm_ctx.obstruction_active = 0u;
 *     s_fsm_ctx.homing_complete    = 0u;
 *     xSemaphoreGive(g_fsm_mutex);
 *
 *     // Trigger homing as boot would do.
 *     fsm_start_homing();
 * }
 *
 * #endif // FSM_UNIT_TEST
 * ────────────────────────────────────────────────────────────────────────────
 *
 * Test CMakeLists.txt (test/unit/CMakeLists.txt) build flags:
 *   target_compile_definitions(test_door_fsm PRIVATE FSM_UNIT_TEST)
 */

/* This file is intentionally empty — it serves as documentation only.
 * The actual shim code is injected into door_fsm.c via the #ifdef above. */