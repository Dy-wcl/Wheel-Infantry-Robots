# Progress

## Every-function log validation (2026-11-04)
- Started full function inventory, reviewed retained tests and identified missing direct/failure-path cases. New tests will compile the current production source, including static helpers, with real RTT transport and host register/RTOS shims.
- Persistent suite passed every function, enabled/disabled source line and branch coverage at 100%, optimized/default-buffer variants, fault injections, ring wrap/full and concurrent records.
- Supplemental ARM ABI check found full/newlib-nano reent layout mismatch introduced by enabling per-task reentrancy without matching compile specs. Initial probe field lookup was adapted for nano's distinct _mp member. Toolchain now selects nano headers/libraries together; rebuild and ABI regression check are in progress.
- Completed full ARM rebuilds and final run.ps1 -CheckArmAbi: four host variants pass; enabled coverage 14/14 functions, 120/120 lines, 92/92 branches; disabled 11/11 functions, 61/61 lines, 38/38 branches. Both ABI probes now match at 76 B. No firmware code function failure found; the one toolchain ABI problem is fixed.
- Chinese test report and reproducible tests are in tests/bsp_log. Build outputs/logs/coverage remain in ignored build/Debug/bsp_log_tests. User staging and ongoing source formatting/buffer edits preserved. No hardware programming or target runtime validation performed.

## Log standard-library/helper reuse (2026-11-04)
- Inspected full source and comp_utils.h. Selected integer snprintf formatting to remove two private append helpers while preserving Float2StrN rounding, sign and length behavior. Also identified reusable ARRAY_LEN/RM_UNUSED and CRLF memcpy.
- Removed private digit/character append implementations. Float2StrN now uses round/fabs and snprintf with 32-bit decimal groups for newlib-nano compatibility; level table uses ARRAY_LEN and a compile-time completeness check, disabled args use RM_UNUSED, and CRLF/NUL use memcpy.
- Enabled/disabled native suites each passed 40000 baseline float comparisons plus sign/round/truncation/range/group-boundary cases. Existing 8000 concurrent logs, 1100 sizes, TLS/error/disabled API and allocator checks passed. Debug/Release firmware builds passed (RAM 46448/43816 B, flash 97872/62332 B). Standalone ARM Float2StrN links with newlib-nano and no _printf_float; whitespace check passes. User staging preserved, no flashing.

## Shared err_t log API and Chinese comments (2026-11-04)
- Started requested status migration and Chinese source annotation. Located the existing shared enum and reviewed all log return paths and retained validation harness.
- Migrated all logging status types to shared err_t from comp_cmd.h, removed the private enum, differentiated NULL pointers/format failures/oversized records/disabled feature results, and updated module documentation. Added detailed Chinese comments throughout bsp_log.c while retaining Float2StrN's length API.
- Debug and Release builds passed (RAM 46448/43816 B, flash 97856/62332 B). Updated native suite passed 8000 concurrent float records, 1100 size boundaries, actual RTT full-buffer skip, all changed error categories, TLS/context checks, disabled behavior and allocator hooks. No hardware flashing.

- Started DR16 `ASSERT`/`VERIFY` hardening. Reviewed the shared assertion semantics, target modules, motor return codes, task initialization order and existing BSP usage. The next edit will add checks to all three DR16 control files while preserving offline and uninitialized safety behavior.
- Added `ASSERT`/`VERIFY` handling to all three DR16 control modules. Initialization now verifies CAN, bus, motor and PID bindings; chassis validates finite remote/kinematic values and control results; shooter validates feedback/angle state and control results while retaining an uninitialized safety-stop no-op. Debug and Release firmware builds both passed, and whitespace/static searches are clean.

- Started is_optimize review and migration; confirmed branch-scope compiler error and located every three-argument caller/example.
- Moved target_rpm into shared scope, preserved the feeder wrapper with true, and updated API and subsystem docs. Both PID backends now test both location modes against the actual algorithms, including wrap boundaries, signed 180-degree cases, unnormalized targets and invalid input paths.
- Debug firmware built successfully; both host suites passed for optimized/direct modes and failure handling. Verified all call sites and usage examples use the new fourth argument; whitespace checks passed.

- Started degree-feedback update; located driver conversion, feedback snapshot and PID degree consumer. Recovered existing task records.
- Added encoder_to_deg and normalized logical position_deg; migrated common PID and VT13 feeder consumers and adjusted existing host tests/documentation to the degree field.
- Debug firmware build passed. Existing host tests reached linking but lack clampf implementation; investigating their component dependency before rerunning.
- Linked comp_utils.c into the host test targets; both PID suites passed. Temporary tests compiled actual motor sources and verified 16384 feedback snapshots across every encoder value and both directions, including zero/range, unchanged radians/raw feedback and metadata. VT13 shooter passed an ARM syntax check. No hardware movement tested.

- Began requested rollback of automatic reverse recovery. Confirmed both PID backends are already restored; preserving their existing staged changes and all unrelated work.
- Removed the device recovery state machine/APIs, added recovery tests and orphan pid_stall.h; restored feeder flags and original default output-filter RC. Removed reversal documentation and preserved the shared speed/location control APIs.
- Restoration verified: Debug firmware builds, original positional/incremental host regression suites pass 2/2, no recovery symbols remain in active source/docs, and whitespace checks pass. PID staged changes were left untouched.

- Inlined feeder recovery parameter values directly into the recovery configuration initializer as requested; removed the six file-scope static constants and updated documentation.

- Replaced all six feeder tuning macros with typed static const variables private to shoot_control.c, updated initialization references and the feeder parameter documentation. Parameter values and control behavior are preserved.
- Verified the constant conversion with a successful Debug firmware build, whitespace check, and search confirming no feeder parameter macros remain.

- Started optional stall/recovery redesign; inspected both PID implementations and DR16 feeder. Existing prior-task plan is retained and extended.
- Inspection search used a PowerShell wildcard directory that rg rejected; subsequent searches use the concrete modules/motor directory.
- Added optional detection to both PID backends and common state/config types. Added positional runtime reset and removed the incremental header's duplicate conditional DWT_CNT declaration.
- Implemented optional recovery in pid_device and enabled it only for the DR16 feeder; stale feedback, timeout and exhausted attempts latch stopped state. Zero speed or explicit reset cancels recovery.
- Debug firmware compiled and linked (RAM 42952 B, flash 93116 B). Old documented host build path was absent; configuring a fresh build/pid_device_tests directory.
- First host regression run passed 2/2 backends, including opt-in timing, multi-turn reverse travel, recovery, retry limit and stale-feedback stop.
- Review found default incremental 0.05 s output filter attenuates startup increments enough that the feeder cannot reach the detection threshold. Set only feeder speed/location output RC to 0.0002 s and add a regression using its actual gains/flags; generic algorithm and friction-wheel tuning remain intact.
- Final Debug firmware build passed and both expanded host suites passed. Tests cover both directions, >360° signed travel, latest location target restoration, output suppression, limit/timeout/offline locks, explicit cancellation, HAL tick rollover and actual feeder gains/filters. Updated backend, device and feeder usage docs; git diff --check passed.
- Final documentation edit had one paragraph-context mismatch; corrected using its exact content. No hardware validation or automatic firing-mode/startup expansion was performed.

- Inspected workspace status and chassis implementation; initialized persistent plan.
- Completed API/build inspection. Chosen approach: caller-owned PID state with separate positional/incremental adapters and a shared DJI device interface.
- Implemented shared control, algorithm adapters, subsystem getters and shooter compatibility wrapper; added sources/include path to app CMake.
- Debug firmware compiled and linked successfully. Added host regression tests comparing shared control against the original algorithms with deterministic timing and mocked transport.
- Both host algorithm suites passed. Documented common API, subsystem access, exact units and CAN write-all behavior; updated three subsystem documents.
- Final verification passed: Debug firmware links, 2/2 host suites pass, all three modules retain identical PID initialization parameters, and whitespace checks pass. Pre-existing dr16_task changes were preserved.
- User requested only pid_device.c / pid_device.h for module code. Merged adapters as static functions, removed three auxiliary code files and updated both build configurations and documentation.
- Consolidation verified: Debug firmware builds, both host suites pass, no stale auxiliary-file references remain, and git diff --check passes.

- Started DR16 shooter mode redesign requested by user: inspect normalized `ch_res`, switch enum values, 2 ms task period and current latch implementation. Next: move single-shot trigger into `sw_r == CMD_SW_MID` with positive `ch_res`, add negative `ch_res` continuous mode and stop/reset on exit.
- Implemented the requested state behavior: exact left/right MID enables positive single-shot confirmation/latch or negative continuous feed; deadband and all switch exits reset state and send zero. Corrected DR16 `ch_res` normalization so ±0.5 thresholds are reachable. Firmware build and behavioral review remain.
- Completed DR16 shooter redesign. Positive `ch_res` is confirmed for 10 ms, locks one 30-degree target, and stops after reaching it; the same held input cannot retrigger. Negative `ch_res` drives continuous feed; deadband and switch exits reset state and send zero. Updated shooter documentation. Debug firmware build and `git diff --check` passed; no hardware movement test performed.
- Replaced the private degree wrapper in `pid_device.c` with the shared `circle_error()` helper. Added modulo reduction inside `circle_error()` so unnormalized multi-turn targets retain shortest-path behavior. Firmware build and whitespace checks passed.
- Replaced the shooter-local `shoot_angle_error_deg()` loop with the same shared `circle_error()` helper and included `comp_utils.h`. Firmware build and whitespace checks passed.
- Adjusted single-shot behavior so friction wheels remain at 2000 RPM while `ch_res > 0.5`; only the feeder performs one latched 30-degree move, then holds zero speed until the mode exits. Firmware build and whitespace checks passed.
- Fixed the single-shot target initialization: `shoot_single_target_deg` now starts from the fresh encoder feedback before `circle_add()` adds 30°, preventing the first shot from using zero or the previous shot's target. Firmware build and whitespace checks passed.
- Removed pid_device include-time macros. Renamed incremental flags to PID_INC_* and updated its implementation, DR16/VT13 callers, documentation and existing tests. Tests now include both PID headers in both orders.
- Verified Debug firmware build, 2/2 host tests, both header inclusion orders, unchanged flag values, migrated incremental callers and absence of define/undef in pid_device.c. Whitespace check passed.

- Started live hardware diagnosis with the connected J-Link. SWD identified the STM32F4 at VTarget ~3.28 V; no automatic programming was performed.
- Confirmed the intermittent `verify_failed` path in the UART DMA design: after a full circular buffer event, `last_rx_pos_` became `dma_size`; the next wrap called the guarded RX dispatcher with a zero-length first segment. Reworked the handler to use HAL's `size` position, preserve a full-buffer sentinel, and suppress duplicate TC/IDLE reports.
- Confirmed DR16 could drop frames because its callback accepted only exactly 18-byte chunks. Added a persistent accumulator, integer frame validation/resynchronization, and immediate offline/command clearing on parse failure. Added a raw-layout size assertion.
- Fixed `feedback_jie = feedback` ordering: feedback is now copied after `dj_motor_get_feedback()` populates the local snapshot. `temperature` remains `uint8_t` because the CAN protocol field is one unsigned byte; angle fields now reflect the real populated snapshot. Added encoder range rejection and restored modulo reduction in `circle_error()`.
- Added retained `g_verify_failed_file` / `g_verify_failed_line` symbols for J-Link inspection. Fixed Release-only `bsp_log.c` typo (`s_last_error`/`s_initialized`) found during validation. Debug and Release builds plus whitespace checks pass.
- Final rebuild after DR16 parse-failure handling and encoder validation: Debug and Release both pass; `git diff --check` remains clean.
- Applied the user's requested DR16 snapshot critical section: read frame_ready_, copy the 18-byte frame, and clear the flag atomically; parsing stays outside the critical section. Notifications provide the wakeup and frame_ready_ determines availability after waiting. Verified USART2 and DMA1 Stream5 priorities are both 5, covered by the FreeRTOS critical-section threshold of 5. Debug/Release builds and the changed-file whitespace check passed.
- Converted the DR16 shooter's three speed loops and feeder location loop to PIDInstance/PIDInit and PID_DEVICE_POSITIONAL. Preserved gains, limits, 0.05 s output and 0.1 s derivative filter constants, motor IDs and mode logic; updated the module's PID documentation. Debug/Release builds and git diff --check passed. Preserved pre-existing staged edits. No flashing or motion validation performed.
- User requested RC=0.0015 following the derivative-filter discussion. Found output RC already edited to 0.0015 s in all four shooter loops; preserved those edits and changed derivative RC from 0.1 s to 0.0015 s in each loop. Updated module documentation. Debug/Release builds and git diff --check passed; Kd remains zero.

- DAPLink diagnosis (2026-11-02): attached with CMSIS-DAP v2/OpenOCD, compared target .text to the existing Debug ELF, and captured the live assertion chain. Confirmed the chassis loop executes four times, then its offline stop-frame VERIFY fails because CAN2 returns BUSY after three unacknowledged queued frames. Read ACK error/TEC=128 in the controller, offline DR16 and no chassis motor feedback. Repeated the capture after an observed target reset with identical results. Detached GDB and resumed execution; no firmware programming or production-source changes.

- Started RTT log stack reduction. Added a static line buffer and statically allocated mutex; reviewing startup/ISR handling before behavioral validation. Existing DR16 stack remains 2048 bytes.

- Completed RTT log stack reduction: shared static line buffer, static FreeRTOS mutex, boot bypass and invalid-context guards. Preserved format/float support, complete-line RTT output and truncation. Updated header/module docs and the old DR16 stack comment; allocation remains 2048 B.
- Validation passed: Debug/Release builds; ARM stack-frame measurement 572 -> 84 B; previous staged source versus updated source output checks for normal/float/embedded-NUL and 1100 truncation lengths; formatting/transport/mutex error cleanup; 4000 concurrent calls on 4 native threads. Harness files are under ignored build/log_stack_validation. No new firmware programming in this turn.

- Started VS Code RTT setup. Confirmed the existing STM32Cube launch entry and adapter-specific FreeRTOS configuration. Preserving its build and project-selection commands; will add a matching RTT client task.

- Completed VS Code RTT setup on the existing jlinkgdbtarget launch entry; added rtt task and .vscode/README.md. Correct adapter-specific RTOS configuration validated against installed schema. Verified J-Link server/client paths, default RTT Telnet port and _SEGGER_RTT symbol. No hardware session or firmware programming started during this configuration change.

## Task-local RTT log error state (2026-11-04)
- Confirmed the shared-error overwrite and return-value race. Reviewed kernel TLS initialization/access APIs, current configuration, and available ARM/native build tools. Implementation and focused concurrency tests are next.
- Implemented task-local error recording and documented API semantics and the reserved TLS slot. Preserved initialization order, current data/reserved RTT buffers, API return values and user staging.
- Debug and Release firmware builds passed (Debug RAM 45032 B / flash 97444 B; Release RAM 43232 B / flash 61884 B). No compiler warnings/errors in the captured build logs.
- Enabled native suite passed 9000 interleaved iterations with 9003 balanced lock acquisitions/releases. Disabled suite passed 9000 iterations. The first disabled harness compilation exposed an unused test variable under -Werror; fixed the test-only variable and reran successfully. Startup, ISR, task creation, suspended/masked contexts, failure cleanup and successful recovery are covered. Tests use host shims rather than target scheduling; no board programming was performed.
## RTT-only log synchronization redesign (2026-11-04)
- Inspected current unstaged/staged log redesign and existing planning history. Removing the duplicate BSP mutex requires replacing shared formatting storage, not just deleting take/give calls.
- Selected call-local formatting followed by one RTT-owned locked write; implementation and focused concurrency/full-buffer checks are next.
- Removed semaphore include/storage/create/take/give and shared line state. Added explicit channel-0 skip mode, private call buffer, complete-record capacity checks and startup-only/idempotent initialization. Kept public result values and task-local error semantics. Updated module docs and the DR16 stack comment.
- Debug and Release firmware builds passed. Debug RAM 45808 B, flash 97384 B; Release RAM 43240 B, flash 61988 B. Native concurrency and real RTT ring-buffer behavior validation remains.
- Native actual-RTT enabled/disabled tests passed: 8000 interleaved float records with forced formatting yields, 1100 body lengths, tag overflow, embedded NUL, format fault injection, full-buffer all-or-nothing skip, recovery, startup/idempotence, filtering, ISR/masks/suspended scheduler and independent task errors.
- Enabled FreeRTOS per-task newlib reentrancy and added scheduler-based allocator hooks after confirming the ARM image used shared float conversion state and no-op malloc locks. Final Debug/Release firmware builds passed (RAM 46448/43816 B; flash 97816/62332 B). Debug BSPLogVWrite stack frame is 576 B excluding callees.
- First native allocator-hook check failed because generated STM32 _sbrk assumes 32-bit pointers. Host-only compiler flags now suppress both pointer-width cast diagnostics for this unexercised function; target compilation remains warning-free. Validation files moved into ignored build/Debug/log_rtt_only_validation.
- Allocator-hook tests passed for startup, nested acquisitions and pre-suspended scheduler; ARM map/disassembly confirms the production hooks are linked. Final review found no BSP semaphore/mutex or shared line buffer; git diff --check passes. Existing user staging is preserved; no firmware flashed.


## Doxygen DWT page diagnosis (2026-11-05)
- Read existing planning history, located source Markdown and generated pages; inspecting render and generator settings.

- Completed browser reproduction of both linked pages, verified final Markdown example by scrolling, and counted 17 absent graph images. Graphviz binary works but is not on PATH. Actual generation configuration is absent/different from root Doxyfile.
- Initial browser file:// navigation was blocked by Playwright; used temporary loopback HTTP server. Screenshot outside browser allowed roots was denied; saved screenshots to project .playwright-cli.
- Diagnosis complete; no firmware/source/generated HTML edits or regeneration required for this explanatory request. Temporary browser/server will be closed.
## Log Doxygen optimization (2026-11-05)
- Read specification, implementation, public guide and linter. Baseline header scan: nine failures.
- Preserve executable code and unrelated comp_utils.c edits. Add English API and helper docs,
  then check both source files with lint.py and compare code tokens against Git HEAD.
- Completed both source files: documented parameters, ownership, return statuses, context limits,
  TLS status rules, disabled macros and float rounding/truncation. Preserved the Markdown guide.
- python lint.py bsp/log passed. Direct lint.analyze_file checks passed for both .c and .h.
- Supplemental specification checks and comment-free token comparison passed; whitespace check
  passed. No functional tests/build needed because executable tokens are unchanged.


## Log header readability cleanup (2026-11-05)
- Reviewed header, implementation, existing documentation and linter. Consolidating duplicated comments and grouping declarations; executable behavior will be preserved.
- Completed header cleanup: 422 -> 286 lines. Constants use concise trailing docs; functions are separated into sections; shared macro contracts are documented once. Removed misplaced private-function docs and vacuous/repeated tags, and repaired wrapped sentences.
- Initial lint reported seven missing explicit @return tags; added concise return summaries and reran successfully. Final python lint.py bsp/log, git diff --check, executable-token comparison, group-balance and 100-column checks passed. No firmware build was needed for this comment/whitespace-only change.
