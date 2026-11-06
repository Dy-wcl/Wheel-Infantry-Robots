# PID device abstraction

## Log Doxygen comment optimization (2026-11-05)
1. Inspect DOXYGEN_REQUEST.md, log implementation and lint.py - complete.
2. Document log APIs, helpers, macros and constants in English - complete.
3. Run scoped lint.py, check source explicitly and verify comment-only changes - complete.
Constraints: preserve executable code, existing unrelated changes and planning history.
Findings: lint.py CLI scans only .h/.hpp; .c requires direct analyze_file invocation.
Baseline: scoped lint reports nine violations.
Discovery errors: searches named absent directories and an invalid Windows wildcard path;
resolved by using existing paths and rg filename globs. An initial multi-file planning patch
used a nonexistent findings heading; corrected with the actual heading before retrying.
Refinement: generation initially detected a parameter line longer than 100 characters; wrapped
comments before writing and then shortened the label description for readability.
Validation: python lint.py bsp/log passed; explicit lint.analyze_file on .h/.c reported zero
violations. Additional checks passed for adjacent function docs, all required function tags,
parameter directions/order, tag order, English text, line width and star alignment. Both files
retain identical non-comment code tokens versus HEAD. git diff --check -- bsp/log passed.

## Every-function log validation (2026-11-04)
1. Inventory all 14 public/private functions and existing coverage gaps - complete.
2. Add reproducible direct-function, integration, fault-injection and coverage tests - complete.
3. Run enabled/disabled suites, address findings and verify ARM Debug/Release - complete.
Constraints: preserve user staging and recent buffer-size/formatting edits; test production source directly, no firmware flashing. Separate host logic evidence from MCU/newlib runtime guarantees.
Finding: host function suites passed 100% enabled/disabled line and branch coverage. ARM ABI probe exposed full-header _reent=320 B vs linked nano _reent=76 B after earlier reentrancy enablement. Move nano.specs to common compile/link language flags; rebuild and reprobe.
Final validation: four host variants pass. Enabled 14/14 functions, 120/120 lines, 92/92 branches; disabled 11/11 functions, 61/61 lines, 38/38 branches. Each variant runs 40000 float roundtrip/truncation checks, two variants run 8000 concurrent records each. ARM Debug/Release full builds and nano ABI checks pass (76 B in both). Test sources, reproducible runner, coverage/ABI checks and Chinese report retained in tests/bsp_log. No flashing or target runtime stack test.

## Log standard-library/helper reuse (2026-11-04)
1. Inspect hand-written formatting and comp_utils helpers - complete.
2. Replace suitable code with standard functions, ARRAY_LEN and RM_UNUSED - complete.
3. Verify float formatting compatibility/boundaries and rebuild Debug/Release - complete.
Constraints: preserve shared err_t, RTT-only synchronization, task errors, Float2StrN length/truncation and half-away-from-zero decimal rounding. Preserve all user staging.
Validation: enabled/disabled host suites passed 40000 Float2StrN baseline comparisons each, explicit rounding/negative-zero/truncation/nonfinite/range/base-1e9 cases, plus existing 8000 concurrent logs and 1100 record boundaries. Debug/Release firmware builds passed; standalone ARM Float2StrN linked against newlib-nano without _printf_float. Whitespace check passed; no hardware flashing.

## Shared err_t log API and Chinese comments (2026-11-04)
1. Locate shared err_t definition and current log return paths - complete.
2. Migrate log header/source/docs to shared err_t; add detailed Chinese comments - complete.
3. Update existing native validation expectations and run Debug/Release builds - complete.
Constraints: reuse component/comp_cmd.h, preserve RTT-only transport locking and task-local errors, retain Float2StrN's length return contract and current user staging.
Validation: Debug/Release firmware builds passed; retained real-RTT host suite passed 8000 concurrent float records and 1100 length boundaries with updated err_t expectations, NULL-pointer and failed/oversized formatting checks. Disabled build and allocator hook suites passed. No hardware flashing.

## RTT-only log synchronization redesign (2026-11-04)
1. Inspect current log, RTT locking, callers and build setup - complete.
2. Remove BSP mutex and shared formatting state; document RTT-only synchronization - complete.
3. Validate concurrent formatting, full-line skip behavior, errors and Debug/Release firmware - complete.
Constraints: preserve current user edits/staging, public logging API, task-local errors and float formatting. No firmware flashing. Keep formatting outside RTT critical sections.
Validation: Debug/Release builds and diff whitespace check passed. Actual RTT host tests passed 8000 concurrent float records and 1100 length boundaries, full-buffer skip, context/error/TLS behavior and disabled API behavior. Production allocator hooks passed startup/recursive/pre-suspended checks. Debug BSPLogVWrite stack frame is 576 B excluding callees; per-task newlib state adds TCB memory. No hardware flashing or runtime stack measurement.

## DR16 shooter positional PID migration (2026-11-01)
1. Inspect current shooter configuration and positional API — complete.
2. Convert four PID instances, initializers, device bindings and documentation — complete.
3. Build Debug/Release and check the scoped diff — complete.

Constraints: preserve existing gains, limits, 0.05 s output/0.1 s derivative filter constants, motor IDs, mode logic and user staging. Do not introduce integral tuning or change rotor angle semantics.

Validation: Debug and Release firmware builds passed; git diff --check passed. Four instances use PIDInstance/PIDInit, and all three device bindings use PID_DEVICE_POSITIONAL. No hardware programming or motion test performed.

## Current hardware/runtime diagnosis (2026-11-01)
1. Trace `verify_failed` definitions and all assertions/verifications — complete.
2. Trace DR16 UART/DMA/idle reception and identify intermittent no-value paths — complete.
3. Trace `dj_motor_feedback_t` production, copy semantics, temperature and angle conversions — complete.
4. Build/static-check and, where possible, add targeted fixes or instrumentation — complete.

Validation: SWD/J-Link probe detected the STM32F4 at 3.28 V. Debug and Release firmware builds pass. `git diff --check` passes. The RX ring handler no longer emits zero-length slices; DR16 accepts arbitrary DMA event fragments and resynchronizes; feedback snapshots are copied only after population; invalid encoder values are rejected; Debug failure file/line symbols are retained. The board was not reflashed automatically.

## Optional shortest-path location control (2026-10-30)
1. Inspect changed API and callers — complete.
2. Fix result scope, migrate callers/docs and cover both modes — complete.
3. Build firmware and run both PID regression suites — complete.

Validation: Debug firmware build passed; positional and incremental host suites passed with both is_optimize modes. All active callers and complete usage examples have four arguments; whitespace check passed.

## Degree feedback (2026-10-30)
1. Inspect feedback conversion and degree consumers — complete.
2. Add normalized logical position_deg and replace redundant conversions — complete.
3. Run firmware build and host regression checks — complete.

Validation: Debug firmware build passed; both PID host suites passed after linking their existing comp_utils.c dependency. A temporary harness compiled the actual motor driver/controller and checked all 8192 encoder values in both directions (16384 snapshots). VT13 shooter passed an ARM compiler syntax check.

## Goal
Centralize reusable motor PID control in app/pid_device and migrate the DR16 chassis, gimbal and shooter without changing control tuning or mode behavior.

## DR16 ASSERT/VERIFY hardening (2026-10-31)
1. Inspect shared assertion semantics and DR16 call sites — complete.
2. Add invariant assertions and return-value verification to the three DR16 modules — complete.
3. Build firmware, run whitespace/static checks and review runtime safety paths — complete.

Validation: Debug and Release firmware builds passed; `git diff --check` passed; all three DR16 sources contain explicit assertion/verification handling and no ignored PID control calls remain.

## Phases
1. Inspect existing motor/PID APIs and build integration — complete.
2. Implement shared APIs, migrate callers and document usage — complete.
3. Build and validate control behavior — complete.
4. Consolidate module implementation into pid_device.c / pid_device.h as requested — complete.
5. Remove include-time alias macros by naming incremental flags PID_INC_*; update callers and verify — complete.

## Constraints
- Preserve pre-existing task/dr16_task/dr16_task.c changes and untracked files.
- Preserve gains, units, feedforward, output limits and transmission behavior.

## Errors
- Degree feedback checks: existing host test linkage fails on undefined clampf; inspect the host dependency configuration. An initial BSP lookup used bsp/can/bsp_can.h instead of locating the header first.
- Initial lookup used a nonexistent component/comp_error/comp_error.h path; locate error declarations with rg instead.
- PowerShell did not expand a wildcard path passed to rg; repeated with a directory argument and -g '*.h'.
- Host test build directory build/Debug/pid_device_tests was absent; configure build/pid_device_tests before building.

## Result
- Shared speed and location APIs, backend adapters, all three subsystem bindings/getters and documentation are complete.
- Final Debug firmware build and both host regression suites passed; PID initialization parameters match HEAD exactly.
- No hardware validation performed. Existing shooter/gimbal registration and mode TODOs remain documented integration prerequisites.

## DR16 shooter ch_res modes (2026-10-30)
1. Inspect `ch_res`, switch semantics, task period and current shooter state — complete.
2. Design and implement single-shot latch / continuous-feed / stop transitions — complete.
3. Build firmware, review diff and document behavior — complete.

Validation: DR16 `ch_res` normalization now matches the other normalized channels; exact switch-middle mode supports debounced single-shot, continuous feed and zero-stop transitions. Debug firmware build and whitespace checks passed. Hardware movement was not tested.

## Circular error helper migration (2026-10-30)
1. Inspect existing angle wrapper and reusable `circle_error` API — complete.
2. Replace location control usage while preserving multi-turn target normalization — complete.
3. Build firmware and check call sites — complete.

Validation: `pid_device_wrap_degrees()` was removed; location control now uses the shared `circle_error()` helper, which first reduces arbitrary angle differences modulo the supplied range. Debug firmware build and whitespace checks passed.

## Optional stall detection and feeder recovery (2026-10-29)
1. Inspect PID timing, device APIs and feeder feedback — complete.
2. Add opt-in, configurable stall detection to both PID backends — complete.
3. Implement configurable angle-based feeder reversal and recovery — complete.
4. Add behavioral regression tests, build firmware and update usage docs — complete.

Constraints: preserve all existing workspace changes; no automatic Ref negation inside PID; use speed-loop detection and measured rotor travel for feeder recovery.

Validation: Debug firmware build passed (RAM 42952 B, flash 93116 B); both host suites passed including actual feeder tuning, multi-turn reversal, feedback/timeout faults and HAL tick rollover. Whitespace check passed. Hardware movement was not tested. Existing shooter registration and Shoot_Mode TODO remain prerequisites for a live firing workflow; recovery is fully connected to public speed/location calls and the existing compatibility entry.

Minor documentation patch failed to match a full paragraph; read the exact paragraph and reapplied the documentation-only update.

## Restore original configuration (user cancellation)
1. Confirm baseline and preserve unrelated changes — complete.
2. Remove recovery integration, test additions and documentation; restore feeder tuning — complete.
3. Build firmware and run original host regression suites — complete.

The PID algorithm files are already back at their pre-redesign contents in the current workspace; do not overwrite them or touch their staged changes.

Restoration verified: Debug firmware builds (RAM 42432 B, flash 89052 B), original host suites pass 2/2, no recovery symbols remain in active code/docs, whitespace check passes, and PID files have no new unstaged changes.

## DAPLink chassis loop diagnosis (2026-11-02)
1. Connect CMSIS-DAP v2 over SWD and verify the running .text against the existing Debug ELF - complete.
2. Read the stopped task, assertion call stack and CAN2 controller registers - complete.
3. Repeat the observation and detach the debugger; record the diagnosis - complete.

Result: b[1] is 4. chassis_task -> Chassis_Mode -> chassis_stop -> verify_failed, failing chassis_control.c:134. CAN2 returns BUSY with all three transmit mailboxes pending; ESR=0x00800033 reports ACK error and TEC=128. DR16 is offline and all four chassis feedback_valid flags are false. A later capture after an observed target reset reproduced the same assertion, counter and CAN status. No firmware programming or production-source edits were performed.

## RTT log stack reduction (2026-11-02)
1. Inspect current formatting, task callers and concurrency requirements - complete.
2. Move the line buffer to static storage with task synchronization; document call context - complete.
3. Compare old/new output, error cleanup and concurrent task behavior; build and measure ARM stack usage - complete.
Constraints: retain float formatting, line length/truncation, colors, virtual terminals and return values. Keep the current DR16 stack allocation until full runtime stack requirements are measured.

Validation: Debug and Release builds passed. ARM Debug .su reports 84 bytes vs original 572 bytes for BSPLogPrintfTerminal (callees excluded); Debug total RAM increased 600 bytes. Native harness compiled the previous staged implementation and updated production source, compared formatted output/return values including 1100 truncation lengths, injected formatting/transport/lock errors, and verified 4000 calls across 4 threads. Whitespace checks passed. Firmware was not flashed in this turn.

## VS Code STM32Cube J-Link RTT setup (2026-11-02)
1. Inspect existing launch type and installed extension schema - complete.
2. Extend the existing jlinkgdbtarget launch configuration and add the pictured RTT terminal task - complete.
3. Validate schema, executable paths and RTT command support; document startup order - complete.
Constraints: modify the existing launch entry; retain STM32Cube build/project selection; use serverRtos for this adapter rather than Cortex-Debug-only rtos.

Validation: existing jlinkgdbtarget entry passed the installed STM32Cube launch Draft7 schema. Both JSON files parse; server/client executables and Debug ELF exist. Confirmed RTT client supports -RTTTelnetPort and defaults to 19021; Debug ELF contains _SEGGER_RTT. Added .vscode/README.md usage instructions. No debug session/reset/programming was initiated. .vscode files remain ignored by existing repository rules.

## Task-local RTT log error state (2026-11-04)
1. Inspect error writers, FreeRTOS TLS support and build setup - complete.
2. Replace shared error state with task-local state and document semantics - complete.
3. Exercise interleaved tasks, startup/ISR boundaries and enabled/disabled builds - complete.
Constraints: preserve current workspace edits, RTT return values and initialization sequence; no firmware flashing. Reserve one FreeRTOS TLS slot, avoid per-task heap allocations, and never read/write task TLS from ISR.
Validation: ARM Debug and Release builds passed. Native harness compiled production bsp_log.c against the actual RTT header and FreeRTOSConfig.h, with task/register/transport shims. Enabled and disabled variants each passed 9000 interleaved iterations across two concurrent workers plus a fresh worker, startup isolation, ISR noninterference, and return preservation. Enabled tests also cover mutex creation/take failures, invalid format/prefix, interrupt masks, scheduler suspension, transport failures and recovery; 9003 mutex takes/gives balanced. No hardware flashing. The disabled test initially failed -Werror on its harness-only unused id; corrected with an explicit unused cast and reran successfully.

## Doxygen DWT page diagnosis (2026-11-05)
1. Compare generated pages, Markdown, source comments and actual generator configuration - complete.
2. Identify scoped documentation/configuration recommendations - complete (diagnosis only; no implementation requested).
3. Verify complete content/rendering in browser - complete (all Markdown content present; 17 graph images absent).
Constraints: preserve user edits and staging; no firmware changes.

## Log header readability cleanup (2026-11-05)
1. Inspect pasted/current header and implementation - complete.
2. Group declarations and consolidate repetitive documentation - complete.
3. Verify unchanged executable tokens and run documentation lint - complete.
Constraints: preserve existing working-tree edits and staging; focus on bsp_log.h readability.
