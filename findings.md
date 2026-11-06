# Findings

## Log Doxygen optimization (2026-11-05)
- Target sources contain eight public APIs and six private helpers; comments must match enabled
  and disabled builds, per-task error storage and whole-record RTT skip behavior.
- Header lacks a file header and function Doxygen blocks; baseline lint.py reports nine failures.
- lint.py CLI only collects .h/.hpp. Explicit analyze_file calls also check the .c implementation.
- Both log files were last changed by xiaoshuaijie on 2026-11-04, commit f84ee5c.
- Keep the Chinese Markdown guide; optimize source comments in English as required.
- Final source documentation: 42 header blocks cover eight APIs, macros, constants and enum
  values; 24 implementation blocks cover fourteen functions and retained internal storage.
- All scoped lint and supplemental specification checks pass with zero FIXME notes.

## Every-function log validation (2026-11-04)
- Current source has 14 functions: six private helpers and eight public entry points. Existing retained harness lacks direct private-helper checks, initialization fault injection/retry, direct VWrite forwarding and Float2StrN snprintf failure coverage.
- User edited buffer sizes into BSP_LOG_DATA_BUFFER_SIZE/BSP_LOG_RESERVED_BUFFER_SIZE and reformatted source after previous turn; preserve current working copy/staging.
- Added persistent tests/bsp_log suite including production source directly; 14 enabled functions/120 lines/92 branches and 11 disabled functions/61 lines/38 branches all covered. Optimization and default-1024-byte-RTT variants pass; standard integration covers 8000 concurrent records.
- ARM header-layout probe detected ABI mismatch: compiler flags omitted nano.specs but linker selected nano. Full struct _reent is 320 B with _result at offset 56; nano is 76 B with _mp at offset 28. Added compile-time nano selection to toolchain and removed its duplicate link-only flag.
- First ABI probe assumed both reent variants exposed _result/_freelist; nano instead exposes _mp. Corrected the diagnostic fields and reproduced the actual incompatible-layout failure. No production-source function test failed.
- Final Debug/Release rebuilds pass without warnings/errors. Both firmware task-header layouts now match nano at 76 B. Final full test invocation passes all four host variants and enforces 100% function/line/branch coverage. Static ARM frames: BSPLogVWrite 576 B, Float2StrN 88 B, excluding callees.
- Delivered tests/bsp_log/README.md and TEST_REPORT.md plus runner, host shims, direct-function test, coverage checker and ARM ABI regression checker. Production log source/header user edits/staging were preserved; only the needed toolchain ABI fix changes firmware configuration in this turn.

## Log standard-library/helper reuse (2026-11-04)
- Replace manual decimal digit/character append helpers with snprintf, quantizing first with round so %.3f does not change halfway rounding or negative-zero behavior. Toolchain newlib-nano/newlib.h explicitly undefines _WANT_IO_LONG_LONG, so use PRIu32 with base-1e9 integer groups instead of unsupported PRIu64 on target.
- Use ARRAY_LEN for level-name table bounds, RM_UNUSED in disabled output branch, and memcpy for CRLF including NUL.
- Existing Float2StrN's double comparison against INT64_MAX rounds that bound to 2^63 and can admit an invalid signed conversion; explicitly check rounded value against [-2^63, 2^63) and use a uint64_t magnitude.
- Replaced two private append functions with round/fabs plus 32-bit integer snprintf; added ARRAY_LEN/static name-count assertion, RM_UNUSED and memcpy of CRLF/NUL. Detailed Chinese comments and float documentation were updated.
- New native coverage compares output/length/buffer contents with the pre-change implementation for 10000 float bit patterns at four capacities, in enabled and disabled builds. Known exact-half/negative-zero, group boundaries, NaN/Inf/FLT_MAX and integer-range boundaries pass. Existing concurrency and error tests also pass.
- Debug/Release firmware builds passed warning-free. A separate ARM entry at Float2StrN linked successfully with nano.specs/nosys.specs and round/snprintf, without pulling _printf_float. No target runtime test was performed.

## Shared err_t log API and Chinese comments (2026-11-04)
- Shared err_t already lives in component/comp_cmd.h. Reuse it rather than define another enum in bsp_log.h.
- Map successful writes to OK, filtered writes to PENDING (synchronous skip, not a queued record), null pointers to PTR_NULL, invalid levels to ARG_ERR, unavailable init to INIT_ERR, invalid contexts to STATE_ERR, RTT exhaustion to NO_BUFF, libc formatting failures to FAILED, oversized records to SIZE_ERR, and disabled log entry points to NOT_SUPPORT.
- Preserve standalone Float2StrN's int length/error return contract; it is not a logging status API.
- Public header now includes comp_cmd.h and exposes err_t for all log status APIs, without old enum aliases. Source has Chinese module/function/branch comments and separates PTR_NULL, FAILED and SIZE_ERR return paths; docs explain PENDING means filtered, without queuing.
- Updated retained native harness uses the real shared enum, exercises all changed status categories and passes enabled/disabled/concurrency/allocator checks. Debug/Release firmware builds passed without warnings/errors; unrelated settings edits and Git staging remain intact.

## RTT-only log synchronization redesign (2026-11-04)
- User requests removing BSP mutex because RTT already locks writes. Current mutex also protects a shared 512-byte line buffer, so removing it requires private formatting storage.
- Use a call-local line buffer, format before one SEGGER_RTT_Write, and explicitly configure channel 0 as NO_BLOCK_SKIP. This avoids BSP synchronization, heap allocation and long interrupt-masked formatting while preserving complete records.
- Preserve task-local error TLS slot 0, existing caller API and current task/startup context restrictions. Stack cost of the private buffer must be measured and documented; existing DR16 allocation is 2048 bytes.
- ARM ELF confirms _dtoa_r/_Balloc use newlib state and allocator; configUSE_NEWLIB_REENTRANT was 0 and linked retarget allocator locks were no-ops. Removing format serialization therefore also requires per-task newlib state and scheduler-based allocator hooks, implemented in FreeRTOSConfig.h/sysmem.c without a mutex.
- Final enabled/disabled host tests compile production bsp_log.c and actual SEGGER_RTT.c. Host-only RTT lock uses a Win32 critical section, with forced task yields outside the transport lock. All 8000 records, 1100 length boundaries, full-buffer no-partial-write and independent errors passed. This validates logic, not ARM scheduling or newlib peak stack usage.
- Production sysmem.c allocator hooks passed startup/no-lock and nested/pre-suspended scheduler balance tests. ARM map resolves __malloc_lock/unlock to our sysmem.c. Debug/Release builds are warning-free; whitespace checks pass. Debug BSPLogVWrite frame is 576 B; full call-chain runtime stack remains unmeasured.

## DR16 shooter positional PID migration (2026-11-01)
- The user requested conversion to positional PID. All four loops now use the existing positional backend; Ki and Kd remain zero. Original incremental filter defaults are explicitly preserved as 0.05 s output and 0.1 s derivative time constants.
- Correction to the earlier explanation: pid_location.h defines abs as a float-compatible macro, so current abs calls do not truncate float inputs. No shared PID algorithm modification was needed for this migration.
- Both firmware configurations build successfully and whitespace checks pass. Hardware response remains unvalidated.

## Hardware/runtime diagnosis (2026-11-01)
- `verify_failed()` is an intentional infinite loop called by Debug `ASSERT`/`VERIFY`; the caller file/line is discarded, so a J-Link stop only shows the loop unless the call site is recovered from the stack/PC.
- The current UART RX ring handler can call `STM32UART_HandleRxData(..., size=0)`: after a full DMA event it stores `last_rx_pos_ == dma_size`; the next wrapped event computes `first_part_size = dma_size - last_pos == 0`, and Debug `ASSERT(size > 0)` enters `verify_failed`.
- `HAL_UARTEx_RxEventCallback()` receives the authoritative DMA write position in its `size` argument, but the BSP ignores it and rereads the DMA counter. The counter may already have advanced, causing intermittent skipped/duplicated data.
- DR16's callback accepts only exactly 18-byte chunks. Idle/half/full DMA events can split a frame or deliver multiple bytes, so data is discarded and `online_` is cleared intermittently. A persistent accumulator/resynchronization parser is needed.
- `pid_device_control_location()` copies an uninitialized local `dj_motor_feedback_t feedback` into global `feedback_jie` before `dj_motor_get_feedback()` fills it. This directly explains random `temperature`, `position_rad`, `position_deg`, `valid`, and tick values in the debugger.
- `dj_motor_raw_feedback_t.temperature` is correctly `uint8_t`: DJI feedback byte 6 is one unsigned byte in degrees C. A debugger may display it as a character; cast to `unsigned` for numeric display. `position_deg` is normalized to [0,360); reversed `position_rad` is intentionally negative per current API docs, while `position_deg` is the normalized logical angle.

## DR16 ASSERT/VERIFY hardening (2026-10-31)
- `ASSERT`/`VERIFY` are defined in `component/comp_utils.h`; Debug failures enter `verify_failed()`, while Release removes only the check and keeps `VERIFY` expressions evaluated.
- The three DR16 modules currently have no `ASSERT`/`VERIFY` calls. Initialization already returns errors, but CAN/motor/PID binding results and safety flush/control results are not consistently checked.
- `can_device` currently registers only the chassis CAN devices; gimbal/shooter initialization remains an integration prerequisite. Runtime safety paths therefore must remain safe before module initialization, while initialized paths should verify successful motor operations.
- Expected implementation: assert internal pointers and finite control intermediates, verify initialization and successful driver/PID binding, preserve existing return/error guards, and avoid asserting normal receiver-offline or invalid-feedback states.

## Optional shortest-path location control (2026-10-30)
- Both branches declare target_rpm locally, leaving its later uses undeclared. Move its declaration outside the branches.
- Header already includes is_optimize, but production wrapper, tests and documented calls still use three arguments. Existing wrapper should pass true to retain shortest-path behavior.
- false uses the target and measured single-turn position directly; it does not provide accumulated multi-turn feedback, and PID derivative-on-measurement behavior differs from the error-as-target mode by design.
- Verification passed: firmware build and both real PID backends across both modes, wrap/180-degree boundaries, unnormalized targets, independent state and invalid input paths.

## Degree feedback (2026-10-30)
- Existing feedback exposes signed logical position_rad; add position_deg in [0, 360), including reversed motors.
- Preserve existing unrelated workspace changes and prior planning records.
- Common PID and VT13 feeder are the two production radians-to-degrees consumers; both now use position_deg directly. The degree helper uses encoder / resolution * 360 without a PI round-trip; reversed zero remains zero.
- Host tests were missing the real comp_utils.c dependency for clampf; adding that source to their CMake targets fixes linkage. Firmware, both host suites, exhaustive degree snapshot checks and VT13 syntax validation passed.

## Cancellation / restoration
- User cancelled automatic reverse recovery and requested the original configuration.
- Current PID source/header/docs already match the pre-redesign behavior (original positional count detector, no incremental stall additions). Remaining recovery code in pid_device and shooter must be removed without disturbing the earlier shared-device refactor.
- Restoration is complete: no automatic reversal, recovery configuration, new stall helper or feeder RC overrides remain. Earlier stall-recovery notes below describe cancelled work.

## Stall recovery inspection
- Positional detector currently uses previous-cycle data, a 0.1% output gate and a fixed count; it does not reverse Ref. Incremental PID has no detector yet.
- Feeder uses incremental speed/location PID through pid_device; public location API uses single-turn rotor degrees. Shoot_Mode is a TODO and startup does not register shooter motors.
- Existing staged PID changes and unstaged control/device changes must be preserved.
- Implemented shared header-only velocity stall detector with 90% output, 95% lag and 0.5 s defaults; same-cycle input/output, direction-aware lag, automatic reset and saturated diagnostics.
- Recovery lives in the existing pid_device.c/.h so both direct speed/location APIs and the shooter compatibility wrapper share it. Encoder deltas accumulate signed rotor travel across wraps; position loop pauses during reversal, original feedforward is suppressed, PID history resets at transitions.
- DR16 feeder alone enables detection and configures 30 output-shaft degrees at 36:1, 600 rotor RPM, 1 s reversal timeout, 20 ms feedback timeout and 3 attempts between stops/resets. These are configurable initial values, not hardware-validated tuning.
- Kept existing Kp/Ki/Kd and limits; shortened only feeder speed/location output-filter RC to 0.0002 s because the incremental filter otherwise attenuates startup steps below the stall gate. This changes feeder response and is documented for hardware tuning.
- An attempted stub read referenced absent stubs/stm32can.h; actual existing stub headers remain untouched except main.h gains HAL_GetTick declaration for recovery tests.

- app/pid_device contains three empty, untracked files; no implementation exists yet.
- Chassis has a private RPM speed loop with per-wheel PID state, torque feedforward and command clamping.
- No applicable AGENTS.md found in the repository ancestry or target modules.
- Chassis uses positional PID; shooter/gimbal use incremental PID. Incremental flags now use distinct PID_INC_* names, eliminating header collisions and include-time alias macros. All module implementation remains in pid_device.c / pid_device.h; algorithm types, flag values, layout and behavior are unchanged.
- Preserve caller-owned PID state/tuning and bind it to a common device. Optional location state supports speed-only or cascaded single-turn degree control; speed feedback is rotor RPM.
- Shooter and gimbal mode implementations are unfinished; this refactor will expose common device access without adding mode logic.
- ARM GCC, CMake, Ninja and host GCC are available. Existing Debug build directory can validate the firmware.
- Full Debug firmware build passed (RAM 42432 B, flash 89628 B). Existing CAN registration initializes only chassis, not shooter/gimbal; documented rather than altering startup scope.
- Shared speed control preserves the original chassis behavior when feedback.valid is false; location control requires valid feedback as before. New bounds checking prevents shooter array overflow and rejects location control on speed-only friction wheels.

## DR16 shooter ch_res modes (2026-10-30)
- `cmd_rc_t.ch_res` is normalized from the DR16 reserve channel by `(raw - 1024) / 1320`; thresholds `> 0.5` and `< -0.5` are valid with a deadband around zero.
- `cmd_switch_pos_t` values are UP=1, DOWN=2, MID=3. The requested behavior applies inside `sw_l == CMD_SW_MID && sw_r == CMD_SW_MID`.
- Shooter task calls `Shoot_Mode()` every 2 ms. The 0x200 group currently has registered shooter slots 1-3, so every active cycle must write all three commands or explicitly flush/stop.
- Existing single-shot latch is tied to left-switch UP and must be moved/extended to the requested right-switch MID + `ch_res > 0.5` mode. Leaving that mode must clear the latch; `ch_res < -0.5` should command continuous feeder operation until the mode exits.
- The original DR16 parser divided `ch_res` by the full channel range without the factor 2 used by the other channels, making the requested strict `> 0.5` / `< -0.5` tests unreachable for the normal 364..1684 raw range. The parser is corrected to normalize `ch_res` to approximately [-1, 1].

## Circular error helper migration (2026-10-30)
- The repository already provides `circle_error(float sp, float fb, float range)` in `component/comp_utils.c`; there are no current call sites. Its scalar API is established by `comp_utils.h` and documentation, so changing it to pointer parameters would be an unnecessary breaking change.
- `pid_device_wrap_degrees()` normalized the target and then the difference. Calling shared `circle_error()` directly is equivalent for normal targets; adding `fmodf(error, range)` preserves the same shortest-path result for arbitrary multi-turn targets too.

## DAPLink runtime evidence (2026-11-02)
- Connected CMSIS-DAP v2 at 1000 kHz using OpenOCD; the existing Debug ELF .text matched target flash exactly.
- Initial target stopped in verify_failed(file=chassis_control.c, line=134). Callers: chassis_stop -> Chassis_Mode (offline branch) -> chassis_task. b={0,4,8,...,60}; chassis_status=OK. This proves four loop iterations before the fatal VERIFY.
- can2_instance.last_error_=BUSY; hcan2.State=HAL_CAN_STATE_LISTENING. TSR=0x82000008 has no TME bits, and all three mailboxes contain pending standard ID 0x200 zero commands (TIR=0x40000001, DLC=8).
- CAN2 ESR=0x00800033: LEC=3 (ACK error), TEC=128, error warning/passive set, no bus-off. hcan2.ErrorCode=0 does not exclude these hardware errors, because the driver enables RX notifications rather than CAN error notifications.
- DR16 online_=false; all four chassis motors have feedback_valid=false. With automatic retransmission enabled, unacknowledged frames occupy all three mailboxes; the fourth stop-frame enqueue returns BUSY and the VERIFY enters its infinite loop.
- Exact external cause is not determined by SWD: inspect powered CAN nodes, CANH/CANL/common ground, termination and the configured 1 Mbit/s bus rate. Handle BUSY through recorded status and periodic zero-command retry rather than a fatal task loop; retain visibility of persistent communication failure.
- A subsequent capture following an observed target reset reproduced b[1]=4, BUSY, TSR=0x82000008 and ESR=0x00800033. SystemCoreClock then read 168000000. No deliberate reset, flash write, peripheral function call or motor-command injection was issued.

## RTT log stack reduction (2026-11-02)
- Existing BSPLogPrintfTerminal stack frame measured by ARM GCC is 572 bytes, of which 512 bytes are a local line buffer. This excludes libc and RTT callees.
- Current call sites execute in task context; main calls BSPLogInit before scheduler initialization. Retain startup initialization and boot logging; do not disable interrupts around slow floating-point formatting.
- Chosen approach: one static line buffer plus a statically allocated FreeRTOS mutex, with priority inheritance and documented task-context requirements. Retain vsnprintf for format compatibility, including %.2f.
- Existing unrelated edits and previous diagnosis changes are preserved. No applicable AGENTS.md was found by the repository file search.

- Final ARM Debug stack frame: 84 bytes vs original 572 (488 bytes / 85.3% reduction); this excludes libc/RTT callees. Static line=512 B, mutex storage=80 B, handle=4 B; linked RAM increases by 600 B including alignment.
- Startup logging bypasses the mutex before scheduler start. Task logging uses priority-inheriting mutex; ISR, masked interrupts, critical sections and suspended scheduler are rejected rather than blocking in invalid contexts. Existing caller sites all use normal task context.
- Baseline-versus-current host output/return comparisons and 4000 concurrent calls passed; native shims test serialization/cleanup, not Cortex-M timing or newlib peak stack. Full runtime stack high-water measurement remains necessary before reducing DR16 allocation.

## VS Code RTT configuration (2026-11-02)
- User specifically requested editing .vscode/launch.json in place and showed a tasks.json entry launching JLinkRTTClient.exe.
- Existing launch type is STM32Cube jlinkgdbtarget. Installed STM32Cube 1.5.0 schema supports serverRtos (enabled=true, driver=freertos, driverPort=ARM_CM4F), preRunCommands and serverExe; it does not support Cortex-Debug rtos or rttConfig fields.
- Standalone J-Link V9.36 provides both JLinkGDBServerCL.exe and JLinkRTTClient.exe. RTT virtual terminals 0..3 share channel 0 and should use one client for the channel.
- Inspection of a localized package file initially used Windows default GBK and failed; explicit UTF-8 resolves it. JLinkRTTClient does not accept -help; avoid treating it as a successful connection test.

- Extended the original launch entry in place: STM32F405RG/SWD, standalone J-Link V9.36 server, STM32Cube FreeRTOS ARM_CM4F proxy and preRunCommands resolving &_SEGGER_RTT from the loaded ELF. Added tasks.json task rtt using process execution and RTT port 19021; original STM32Cube build/project selection preserved.
- JSON and installed STM32Cube Draft7 schema validation passed, and executable/symbol checks passed. Runtime terminal connection remains to be exercised by starting the VS Code debugger and continuing beyond main. Existing .vscode gitignore rule excludes these local files.

## Task-local RTT log error state (2026-11-04)
- Current last_error is global; init/config/select wrappers also read that shared value for their return. Per-write locking would not preserve a caller's later error query.
- FreeRTOS 10.3.1 supports task-local pointer storage but configNUM_THREAD_LOCAL_STORAGE_POINTERS currently defaults to zero. No application TLS users found.
- Existing docs only promise init/terminal errors; formatted output returns failures without updating last_error. Extend error recording to log/transport entry points while preserving their return values.
- Startup must retain a separate error value; ISR queries return failure and must not overwrite the interrupted task's error. Use one reserved TCB slot for integer status via intptr_t, without allocating objects.
- Initial broad discovery output was too large; subsequent reads/searches are scoped. A search included nonexistent application/ and Release cache paths; corrected to current directories and configure Release when validating.
- Implementation reserves TLS slot 0 in FreeRTOSConfig.h, with a compile-time capacity check. Each API records status in its own task slot and returns the unshared result. Startup uses a separate static error. Getter is non-destructive and rejects ISR; recorder skips ISR before any scheduler/TLS access.
- Error recording now covers initialization/config/select plus formatted output, raw channel write and legacy PrintLog. Negative results are retained; nonnegative results clear the error. Direct RTT calls/LOG_CLEAR and disabled macros do not update it; disabled PrintLog preserves its existing zero return.
- Native tests use actual production source with Win32 worker threads and task-local shim slots. Deliberate yield after TLS writes and forced A/B handoff exercise the overwrite scenario; fresh worker confirms state is not inherited. Register, mutex and transport fault injection cover boundary paths. Harness is retained under ignored build/Debug/log_tls_test.

## Doxygen DWT page diagnosis (2026-11-05)
- User links point to bsp/log/html; root Doxyfile is default configuration and may differ from the actual generator invocation.
- bsp_dwt.md has four fenced code blocks and TIME_ELAPSE macro example; investigate generated output termination.

- Browser verification: Markdown has all four code blocks; #doc-content viewport is 586 px and scrollHeight 907 px at 1280x720. Scrolling its own panel shows final TIME_ELAPSE example completely.
- API page has 17 missing PNG dependencies (include/call/caller graphs); only .md5 artifacts remain for DWT_Delay. Missing generation root cause cannot be proven without actual run log/config.
- Graphviz exists at E:/Graphviz/bin/dot.exe (16.1.0) but dot is not on shell PATH.
- Generated pages use project jie V1.0.0, graphs and source browser; root Doxyfile is default My Project, HAVE_DOT=NO and SOURCE_BROWSER=NO. It is not the configuration that produced the linked output.
- Doxygen merges .c/.h documentation and repeats briefs; internal //! comments also enter public descriptions. Markdown contains stale DWT_DetTimeline_ms and LOGINFO examples.
- No source or generator configuration changed: user asked for diagnosis; report verified causes and recommended settings.

## Log header readability cleanup (2026-11-05)
- The pasted header matches current code and repeats identical macro documentation six times.
- A private BSPLogRecordResult comment is incorrectly attached to the include guard.
- Keep public API contracts, shorten constant docs, and repair comment wrapping.
- Baseline header saved under ignored build/Debug/log_header_cleanup for token comparison.
- Confirmed the command buffer reserves its final byte for NUL; corrected its capacity description accordingly. Doxygen membership uses one enclosing module group, with a nested macro group.
