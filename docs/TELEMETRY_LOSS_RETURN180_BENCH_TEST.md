# YWW V5.3 telemetry-loss RETURN180 read-only bench simulation

Offline implementation and configuration validation: 2026-11-23. Target: bare CUAV V5+, ArduCopter 4.7.3. **No hardware validation was performed during this software task.** The separately supplied [Log 19 summary](YWW_V53_HARDWARE_VALIDATION_LOG19.md) reports one manual bench trial; its raw binary is not included and was not re-decoded here. No commit, push, deployment, controller file copy, or parameter change was performed during this task.

This is a read-only experiment. The operator manually rotates the controller after a simulated-response notification. Turning 180 degrees **does not constitute geographic return-to-home navigation**: the script computes no home bearing, position target, trajectory, or vehicle movement command.

## Architecture and preservation

The single eventual deployment candidate remains `lua/radio_health_monitor.lua`. A marked V5.3 block observes the existing radio state and timestamps; its separately protected callback runs after the existing radio and magnetic callbacks. There is no additional scheduler or script to deploy.

The V5.2 radio core remains byte-for-byte intact underneath the new header, block, and callback invocation. The embedded legacy magnetic monitor differs only in its intentional `MHR_ENABLED = false` production flag. The original startup message still identifies the V5.2 radio core. Preserved behavior includes:

- Heartbeat timeout strictly **>3000 ms**; RADIO_STATUS stale strictly **>3500 ms**.
- RSSI threshold `<90`, three-second low-RSSI persistence, unknown `255` handling, original fault latch and state codes.
- Three recovery observations at least 700 ms apart, with at most one observation per callback; original sysid, GCS type, and channel filtering.
- Original `YWHT` schema, record values, logging schedule, radio notifications, and radio error handling.
- Existing independent 360-degree magnetic algorithm and `YWMR` schema, notifications, timing, and error isolation when enabled by the test loader. It stays disabled in production.
- Initial 100-ms callback and original `ok and 100 or 1000` return behavior. A RETURN180 failure cannot select the radio's 1000-ms error retry.

The existing golden radio replay passes in five modes (production magnetic-disabled; integration normal, arming, magnetic exception, rotation): each preserves all **13 `YWHT` records and 21 radio notifications**, including timestamps and all fields. The trace helper excludes only identified magnetic and new simulation notifications from radio-only comparisons; its original expected fixture is unchanged.

### Production and integration test configurations

| Configuration | Legacy `MHR_ENABLED` | RETURN180 | Expected diagnostic output |
|---|---|---|---|
| A: production (default) | `false`, source loaded unchanged | Enabled | `YWHT`; `YWRT` on completion; no `YWMR` or legacy baseline/timeout notifications |
| B: integration regression | `true`, in memory only | Enabled | Independent `YWHT`, `YWMR`, and `YWRT` with unchanged schemas |

`tests/support/ardupilot_mock.lua` defaults to `configuration='production'`.
Tests that require both diagnostics explicitly request `configuration='integration'`.
The integration transformation requires exactly one literal production
`local MHR_ENABLED = false` declaration, replaces only that declaration, and the loader rejects unknown
configuration names. It never writes source files or introduces a controller setting.
The development fragment intentionally retains `MHR_ENABLED = true`; the sync
assertion compares it against the integration version of the embedded block and
requires every other byte to match.

Investigation reproduced 75 passing RETURN180 scenarios and two failures at the
original lines 344 and 352. Both used the default production fixture but required
a `YWMR` completion, which the disabled legacy callback cannot emit. The older
integration suite also failed its first baseline-readiness assertion and its
strict source/fragment equality would reject the intentional flag difference.
Changing only that flag in a temporary source copy made all original 77 RETURN180
and 57 integration scenarios pass. This isolates the observed failures to the
test configuration; no production logic fix was needed.

The corrected tests retain all existing cases, exercise production through long
idle periods, a half-turn completion, a full turn, and radio recovery, and check
that every production RETURN180 scenario stays silent for legacy magnetic output.
Both logging-error directions are covered with both diagnostics enabled, including
successful original radio recovery afterward.

## Trigger qualification and USB isolation

1. A successful radio update must first establish `LINK_ACTIVE` with both fresh approved GCS heartbeat and fresh RADIO_STATUS on the selected radio channel. No radio at startup, GCS_UNSEEN, or USB-only heartbeat traffic cannot qualify the detector.
2. After qualification, either heartbeat age exceeding the existing three-second timeout or RADIO_STATUS age exceeding 3.5 seconds consumes one loss episode. RSSI does not participate in this trigger and cannot establish interference.
3. Starting the manual experiment requires `arming:is_armed()` to return explicit `false` and `ahrs:get_yaw_rad()` to return usable finite numeric yaw. Armed/unknown state or invalid heading yields a one-time `SIM SKIP` status. Restoring a valid sensor or disarming during the same loss does not start a delayed experiment.
4. Start is the current measured AHRS heading in degrees, normalized to `[0,360)`. Target is `(Start+180)%360`. The loss notification and measured start/target are sent once.
5. While pending, the **first restoration of both fresh selected-channel heartbeat and status cancels** the attempt, including while the radio state is RECOVERING. This check precedes completion, so restoration on the settling-completion poll cannot produce PASS.
6. A future attempt is enabled only after the original radio state machine reaches `LINK_ACTIVE` again. A burst of buffered heartbeats cannot bypass that recovery rule.

Approved heartbeats are read from `gcs_seen[radio_chan]`. USB MAVProxy heartbeats on a different receive channel cannot suppress a ground-radio loss or cancel a pending test. Existing sysid/range and MAV_TYPE_GCS filtering still apply. Notifications can be observed over USB independently of ground-radio reception.

**Physical channel identification is a bench prerequisite.** The existing radio core selects the channel of received RADIO_STATUS; V5.3 preserves that behavior. Verify that the selected `YWHT Ch`/STATUSTEXT `c=` corresponds to the intended telemetry UART, with USB on a different MAVLink receive channel. Do not bridge or inject RADIO_STATUS over USB or allow competing status sources during this experiment. If the selected channel changes during a pending test, the simulation cancels and requires a healthy qualification on the newly selected channel. No packet metadata identifies a physical SiK device beyond this existing selection rule. Loss is a communication observation, not proof of RF interference or jamming.

## State machine and measurement

State is represented by a qualified channel, a loss-consumed latch, and a pending-attempt flag:

| Phase | Behavior | Transition |
|---|---|---|
| Unqualified | No attempt, timer, or simulation timeout notification | Fresh LINK_ACTIVE qualifies a selected radio channel |
| Qualified | Observe only existing heartbeat/status ages | Confirmed timeout consumes one loss episode |
| Skipped | One SIM SKIP for armed/unknown state or invalid heading | Wait for full LINK_ACTIVE recovery |
| Tracking | Capture start/target immediately at trigger; accumulate signed wrapped yaw deltas | Settle, cancel, or timeout |
| Settling | Require net turn magnitude 170–190 degrees and target error <=10 degrees | Complete only after two seconds stable; movement resets settling |
| Completed/cancelled/timed out | Attempt consumed; no retry or idle experiment timer | Full LINK_ACTIVE recovery permits another loss event |

Yaw comes from the **AHRS/EKF estimate**, not a direct raw magnetometer reading. Each delta and target-relative error uses `(a-b+180)%360-180`. Rotation is signed and unwrapped, supporting both positive and negative turns through 0/360. A valid completion requires accumulated net rotation, not a single target-heading coincidence. A 180-degree sensor jump is rejected.

Settling requires individual changes <=0.5 degrees and heading within 1.5 degrees of the settling reference for at least 2000 ms. Reference-window violations restart settling even when every individual step is small. Movement outside the target/turn completion window cancels that settling candidate. Deliberate reversals subtract from net turn.

Cancellation conditions, each terminal for that loss episode:

| Condition | STATUSTEXT suffix |
|---|---|
| Selected-channel heartbeat and status become fresh | `CANCEL recovered` |
| Armed or armed-state indication becomes unknown | `CANCEL armed` |
| Heading missing, nonnumeric, NaN, infinite, or implausibly large (>1e6 radians) | `CANCEL invalid heading` |
| Update interval nonpositive or >1000 ms | `CANCEL sample gap` |
| Single wrapped step >45 degrees | `CANCEL yaw jump` |
| Net rotation magnitude >270 degrees | `CANCEL excessive turn` |
| Selected radio channel changes | `CANCEL channel changed` |
| Original radio update throws | `CANCEL radio error` |
| RETURN180 API/logic exception while pending | `CANCEL error` plus throttled diagnostic |
| Duration from trigger >120000 ms | `TIMEOUT` |

Timing uses existing uint32 subtraction via `age_ms`, including across uptime rollover. Exact 1000-ms gaps and 45-degree steps are permitted. No repeated RETURN180 timeout messages occur before qualification or after a terminal outcome. Legacy magnetic baseline/timeout behavior is exercised only in integration configuration B; it produces no notifications in production configuration A.

## DataFlash `YWRT`

Completion call:

```lua
logger:write('YWRT', 'Start,Target,End,Err,Result,Source,Dir', 'ffffBBB',
    start_heading, target_heading, final_heading, signed_error,
    result_code, trigger_source, direction_code)
```

| Field | Type | Meaning |
|---|---|---|
| Start | `f` | Initial measured AHRS heading, degrees `[0,360)` |
| Target | `f` | `(Start+180)%360`, degrees |
| End | `f` | Final measured AHRS heading, degrees `[0,360)` |
| Err | `f` | Signed wrapped `End-Target`, degrees `[-180,180)` |
| Result | `B` | `2 PASS` for abs(error)<=2; `3 WARN` for 2<abs(error)<=5; `4 FAIL` for 5<abs(error)<=10 |
| Source | `B` | `1` heartbeat timeout, `2` status stale, `3` both present at first trigger observation |
| Dir | `B` | `1` positive net manual rotation; `2` negative net manual rotation |

The call supplies seven fields, seven format characters, and seven numeric arguments. Labels are under ArduPilot's 58-character limit, format under 15, and the message name has four characters. ArduPilot adds `TimeUS`; the payload is four floats and three unsigned bytes. The API shape and limits were checked against the [official bindings](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/generator/description/bindings.desc) and [logger implementation](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/lua_bindings.cpp). This does not validate the exact installed firmware build.

Source is frozen at trigger time. A full ground-link outage normally exceeds the three-second heartbeat timeout before status becomes stale, so it can legitimately record Source=1 even if both streams are absent by completion. Source codes describe observed timeout evidence, not RF cause.

Only completed measurements produce `YWRT`. Skips, cancellations, and timeouts are **separate terminal STATUSTEXT outcomes**, with no completion record or invented PASS/FAIL code. Capture USB console/telemetry output if an audit trail of those outcomes is needed. `YWHT` still records the original radio state transitions. `YWMR` remains dedicated to the independent full-turn experiment and is absent with the production flag disabled.

A completed attempt is marked consumed before either output operation. Logger and result STATUSTEXT are each attempted once under independent protection; failure of either cannot retry the result or stop either existing monitor. A logging exception produces a throttled diagnostic but does not disable future loss episodes. Physical persistence cannot be guaranteed if disarmed logging is disabled, storage is full/unavailable, or a backend fails.

## Notifications

Example normal attempt, excluding original radio and magnetic output:

```text
YWW Telemetry LOST (SIM)
YWW RETURN180 SIM start=35.0 target=215.0
YWW RETURN180 SIM PASS err=+1.20
```

Other possible outcomes:

```text
YWW RETURN180 SIM WARN err=-3.50
YWW RETURN180 SIM FAIL err=+8.00
YWW RETURN180 SIM CANCEL recovered
YWW RETURN180 SIM TIMEOUT
YWW RETURN180 SIM SKIP armed
YWW RETURN180 SIM SKIP invalid heading
YWW RETURN180 SIM error: injected logger exception
```

All new notifications explicitly include `SIM` and fit 50 bytes. Normal attempts send one loss notification, one start/target notification, and one result. Skips/cancellations/timeouts occur once per event. Diagnostics have an independent ten-second throttle and protected telemetry output. Start/result PASS uses INFO (`6`); loss, WARN, FAIL, and terminal warnings/diagnostics use WARNING (`4`). The original radio and magnetic messages remain unchanged.

## Safety and limitations

No vehicle-control API, automatic yaw function, disabled actuator function, arming command, mode change, navigation target, calibration operation, parameter write, or new failsafe is present. A clearly labeled **non-executable comment** marks a possible future integration point and requires a separate safety design and independent review. There is no executable control implementation behind that comment.

The monitor uses a fixed scalar state table with bounded per-poll work and a 120-second active-attempt bound. The original radio receive loop remains capped at 25 messages. Ordinary Lua exceptions are isolated; shared-heap exhaustion, firmware scripting shutdown, unavailable telemetry, or scheduler failure remain system-level limitations.

Historical host Lua 5.3 resource probe from the initial implementation, before
this configuration correction, using identical API doubles and 10,000 stationary polls:

| Measurement | Before V5.3 | V5.3 |
|---|---:|---:|
| Source bytes | 16,330 | 23,243 |
| Stripped host bytecode bytes | 6,743 | 10,604 |
| Warm heap delta including mocks | 26.35 KiB | 41.46 KiB |
| Retained heap drift after 10,000 polls | -0.01 KiB | +0.06 KiB |
| Observed host time per poll | 2.10 microseconds | 2.35 microseconds |

These historical values were not remeasured for the corrected test loader or production flag and do not size embedded heap, peak memory, or CUAV timing. Review existing `SCR_HEAP_SIZE`, other scripts, available memory, and actual script-load behavior during a later authorized bench session. No heap or controller configuration was changed.

Yaw stability is not proof of physical stillness or absolute heading accuracy. EKF resets, finite but stale/incorrect estimates, tilt, magnetic disturbances, and aliasing of rapid physical motion can invalidate a trial. A jump over 45 degrees is rejected only when visible in sampled estimated yaw. Manual rotation should be smooth, level, and physically indexed. An accurate estimated half-turn still says nothing about the direction or distance to home.

## Offline validation

Run from the repository root:

```bash
lua5.3 tests/yww_v5_2_offline_test.lua lua/radio_health_monitor.lua
lua5.3 tests/test_magnetic_return.lua lua/magnetic_return_embed.lua
lua5.3 tests/test_radio_magnetic_integration.lua lua/radio_health_monitor.lua
lua5.3 tests/test_return180.lua lua/radio_health_monitor.lua
luac5.3 -p lua/*.lua tests/*.lua tests/support/*.lua tests/fixtures/*.lua
lua5.3 tests/measure_monitor_resources.lua lua/radio_health_monitor.lua
git diff --check
```

| Suite | Result | Actual count |
|---|---|---|
| Original radio health | PASS | 12 checks; 12 validated simulated YWHT records |
| Magnetic fragment | PASS | 12 scenarios |
| Existing full-script integration | PASS | 58 scenarios, including five unchanged golden radio traces |
| RETURN180 full-script regression | PASS | 79 scenarios, zero failures |
| Lua 5.3 syntax | PASS | 10 Lua files |
| Diff/whitespace | PASS | Tracked diff and release-scoped untracked files checked |
| Physical hardware validation during this task | NOT PERFORMED | Supplied Log 19 findings are reported separately |

**161 checks/scenarios passed; final failures: 0.** The RETURN180 suite covers healthy qualification, missing radio at startup, USB heartbeat isolation, selected-channel identity filtering, heartbeat/status/both trigger sources and exact boundaries, both rotation directions, target wraparound, measured starts, PASS/WARN/FAIL boundaries, incomplete/reversed turns, target coincidences, stationary noise, settling duration/drift, recovery cancellation including the completion poll, recovery bursts and re-enabling, duplicate prevention, idle/active timeout, excessive rotation, nil/NaN/infinite yaw, armed/unknown state, gap/jump boundaries, uint32 wrap, channel changes, radio/AHRS/arming exceptions, logger and STATUSTEXT errors, diagnostic throttling, production magnetic silence, and independent YWHT/YWMR/YWRT logging.

The mock APIs record forbidden method calls before raising errors, so `pcall` cannot hide a detected control invocation. Every RETURN180 runtime scenario checks that record, and an additional production-source method allowlist check passes. These mocked tests do not validate hardware operation or vehicle safety for flight.

## Physical bench procedure (not performed)

1. Review the source diff and this report. Keep the CUAV bare, with no motors, ESCs, frame, or propellers, and keep it disarmed throughout. Use a level, nonmagnetic surface and a physical 0/180-degree indexing mark. Avoid magnetic objects, power cables, or changing current loads near the controller; provide cable slack for manual rotation.
2. Record the exact firmware build and existing scripting, EKF3, GCS identity, logging, and memory configuration. Confirm required bindings, available AHRS heading, successful script loading, adequate heap headroom, and an operational disarmed DataFlash backend. Existing scripting requires `SCR_ENABLE=1`; disarmed recording requires suitable existing logging including `LOG_DISARMED=1`. This task makes no configuration changes.
3. Deployment is **not part of this implementation**. In a separately authorized session, back up the currently deployed script outside the active scripts directory and deploy only the integrated `radio_health_monitor.lua`. Do not deploy `magnetic_return_embed.lua`, tests, or duplicate old `.lua` scripts. Follow the established script-restart procedure.
4. Connect USB MAVProxy for observation and the separate ground telemetry endpoint for radio traffic. Confirm the selected radio `c=`/`Ch` corresponds to the telemetry UART, not USB. Keep the paths separate; do not forward RADIO_STATUS into USB. Wait for the original radio monitor to establish LINK_ACTIVE. With no prior healthy radio link, verify that USB alone never produces a RETURN180 start.
5. Record the physical initial index and estimated heading. Stop the **ground** GCS heartbeat stream or disconnect its radio endpoint while leaving USB MAVProxy connected and active. Do not substitute a USB disconnect for this loss. Observe one `Telemetry LOST (SIM)` and one `RETURN180 SIM start=... target=...` after the applicable timeout. RSSI can be high or unknown and does not change the loss criterion.
6. Only after the simulation start message, manually rotate approximately +180 degrees about the vertical axis over roughly 5–15 seconds, maintaining level orientation and staying below 45 estimated degrees per sample. Stop at the physical opposite index and hold for at least two seconds. Keep the ground connection lost until completion and record the estimated target/error/result.
7. Keep holding; verify no duplicate RETURN180 result or repeated simulation timeout. Restore the ground telemetry path. Wait for all three time-spaced recovery observations and LINK_ACTIVE before deliberately causing another loss. Repeat with -180 degrees, including trials across 0/360. Compare estimated heading error with the independent physical indexing marks.
8. On a separate loss trial, rotate only 90 degrees and hold: no completion should appear. Restore the ground connection before completing the turn: expect one `SIM CANCEL recovered`, including if the radio is still RECOVERING. On an idle loss trial with no rotation, expect one TIMEOUT after 120 seconds and no repeated RETURN180 timeouts for that outage.
9. Verify that USB heartbeat traffic continues throughout the ground loss without preventing the read-only bench simulation. Confirm unchanged `YWHT` and one `YWRT` per completed half-turn. With production `MHR_ENABLED = false`, expect no `YWMR` even after a full turn, and no legacy magnetic baseline or timeout messages. All three schemas are exercised together only by host integration configuration B. Review USB output for cancellations/skips/timeouts, which intentionally have no completion record.
10. Check for scripting errors, memory warnings, EKF resets, unexpected channel selection, and missing/duplicate results. Record actual hardware/configuration/trial evidence before claiming physical validation. Armed-state, injected API errors, logger errors, and discontinuities were tested only in mocks; do not arm the controller to reproduce them as part of this disarmed procedure.

## Rollback

No deployed file or controller parameter was changed, so no controller rollback is currently needed. The pre-V5.3 local version is commit `621decb1777636df0eebfb177fa485a0eb305550`. Extract it for review without overwriting current source:

```bash
git show 621decb1777636df0eebfb177fa485a0eb305550:lua/radio_health_monitor.lua > /tmp/radio_health_monitor_pre_v53.lua
```

Preserve any later local edits before restoring that file or selectively removing the marked V5.3 block, new header, and RETURN180 callback invocation. Re-run the radio and magnetic regression suites. In a separately authorized controller-maintenance session, restore the saved pre-V5.3 integrated script as the only active monitor file and restart it through the normal procedure. Keep backups outside `/APM/scripts/`. No parameter rollback is required.
