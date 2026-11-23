# Magnetic heading-return integration report

Local implementation and offline validation: 2026-11-23. Target: bare CUAV V5+, ArduCopter 4.7.3. **Hardware validation: NOT PERFORMED.** This progress is recorded in a local Git commit at the user's request. No push, deployment, flight-controller file copy, or parameter change was performed.

## Repository inspection and baseline

The actual repository has `tests/`, rather than the task's example `test/`. Inspected the deployed-source candidate `lua/radio_health_monitor.lua`, supplied development fragment `lua/magnetic_return_embed.lua`, both existing test harnesses, and repository notes before editing.

The original radio script was saved locally for comparison. Its repository revision is `fa700b723f5d4ba51d8708891e664612498f687d`; the script SHA-256 is `308d15268051a383eebe0d93149d1736f48168456db8a91748f488960cd1db34`. Its existing ArduRover header was retained; that header does not establish ArduCopter hardware compatibility.

Before editing:

- Radio harness: PASS, 12 state-machine checks and 12 simulated `YWHT` records.
- Supplied magnetic harness: PASS, 12 executed scenarios. Its original summary incorrectly said 11; only that summary was corrected. PASS is covered by the positive/negative full-turn cases.

## Architecture and changes

Only `lua/radio_health_monitor.lua` is an eventual deployment candidate. The magnetic block is embedded directly in that file. The fragment remains a development source with identical content; an integration assertion detects divergence. It has no independent scheduler and must never be copied to `/APM/scripts/`.

Each existing scheduled callback performs:

1. Original radio `step()` inside its original `pcall`, including original error handling.
2. Magnetic update inside a separate `pcall`, regardless of radio state or radio `step()` success.
3. Magnetic reset on an exception, with a separately protected WARNING diagnostic limited to one per 10 seconds.
4. Original `return protected_update, ok and 100 or 1000`.

Initial return remains `protected_update, 100`. Thus normal polling remains 100 ms; a radio exception still selects the original 1000-ms delay. A magnetic exception never selects that delay. Magnetic diagnostics fit 50 characters and cannot terminate the callback if their own `send_text` raises an error.

The magnetic block reads `ahrs:get_yaw_rad()` and converts radians to degrees, reads `arming:is_armed()`, and uses `millis():tofloat()` for measurement timing. It does not depend on SiK status, GCS heartbeat reception, RSSI, radio recovery, or the radio logger-enabled flag. The original script's startup requirements, including valid configured GCS identity and MAVLink bindings, still apply.

Changes beyond embedding the supplied fragment:

- Final settling limits total drift within a 1.5-degree reference window as well as per-sample motion, preventing slow drift from satisfying the two-second settle test.
- A completed attempt is consumed and cooldown established before calling logger or telemetry APIs, preventing retries after an exception.
- Backwards time, including uint32 uptime rollover, discards the baseline and stale cooldown safely.
- Invalid nonnumeric yaw and both infinities are rejected along with missing yaw and NaN.
- Original radio tests gained only the new read-only API doubles and an exact `YWHT` labels assertion. Existing scenarios/assertions remain.
- Added full-script integration tests, a pre-integration radio trace fixture, shared API doubles, and a host resource probe.

## Existing radio behavior preserved

Removing only the inserted magnetic block, diagnostic helper, and callback invocation reproduces the original script byte-for-byte. No original statement, state code, message, schema, threshold, or recovery condition was edited.

| Existing behavior | Verified source behavior retained |
|---|---|
| State codes | `0 WAITING`, `1 GCS_UNSEEN`, `2 LINK_ACTIVE`, `3 GCS_SILENT`, `4 RF_LOSS_SUSPECTED`, `5 STATUS_STALE`, `6 RECOVERING` |
| No RADIO_STATUS received | WAITING; clear low-RSSI/recovery timers |
| Status age | STATUS_STALE only when age **>3500 ms**; stale RF metrics become unknown; fault history remains |
| No approved heartbeat on radio channel | GCS_UNSEEN, or RF_LOSS_SUSPECTED if fault already latched |
| Heartbeat age | GCS_SILENT only when age **>3000 ms**, unless fault already latched |
| RSSI | Low when valid and **<90**; `255` means unknown; not dBm |
| Low-RSSI persistence | During heartbeat silence with fresh radio status, latch after **>=3000 ms** of sustained low RSSI; increment fault count once |
| Recovery | Three accepted heartbeat observations, **>=700 ms** apart; at most one observation per callback, regardless of burst size; latch clears only on qualified recovery |
| Cold startup | Fresh approved heartbeat and radio status can directly establish LINK_ACTIVE |
| GCS filtering | `MAV_GCS_SYSID`, fallback `SYSID_MYGCS`; optional inclusive `MAV_GCS_SYSID_HI` range; MAV_TYPE_GCS (`6`) required |
| Channel filtering | Heartbeats matched to selected RADIO_STATUS channel; changing radio channel resets low-RSSI/recovery timers |
| MAVLink handling | Queue size 25, two subscriptions (`109`, `0`), up to 25 receives per callback; existing raw-string offsets and length checks unchanged |
| Radio STATUSTEXT | Startup and state-transition reports; periodic reports every 5 seconds at INFO (`6`); existing error text/severity unchanged |
| Radio logging | On state transitions and every 30 seconds; logger exception disables only original radio logging, with original ERROR (`3`) notification |
| Radio errors | Original `pcall(step)`, error notification at most once per 10 seconds, original 1000-ms error retry |
| Uptime arithmetic | Original radio uint32 subtraction before `toint()` retained |

Unchanged `YWHT` labels: `S,Ch,R,HB,SA,Rec,Lat,Flt,RR,N,Tx,Err`. Unchanged format: `BBiiiBBIiiii`. The fault latch and event count survive stale status as before.

The saved extended radio replay includes a further 32 seconds of silence to cover periodic logging. In each of four integration modes (normal, arming, magnetic exception, magnetic rotation), all **13 `YWHT` records and 21 radio STATUSTEXT messages**, including timestamps and every field, exactly match the original script's fixture. Rotating mode also produces independent `YWMR` records.

## Magnetic state machine

| Phase | Entry and behavior | Exit |
|---|---|---|
| WAITING (`state=0`) | Disarmed with valid yaw; collect a stationary interval | Capture current heading after >=2000 ms stationary, then send one ready message |
| TRACKING (`state=1`) | Baseline fixed; sum signed shortest differences between samples | Settle candidate, reset, or abort |
| Settling candidate | `abs(turn)>=350` and `abs(wrapped error)<=10`; each step <=0.5 degrees; remain within 1.5 degrees of candidate reference | Complete after >=2000 ms; movement/window violation restarts settling; leaving return window cancels candidate |
| Completion event | Classify actual absolute error; save signed turn/error and direction; consume attempt; request one log write and one result message | Five-second cooldown |
| Cooldown | No baseline capture or result; arming does not bypass cooldown | After >=5000 ms, begin a fresh stationary capture, requiring another two seconds |

Baseline stationarity uses the same <=0.5-degree step and <=1.5-degree reference-window bounds. These are yaw-stability heuristics; yaw alone cannot prove physical stationarity.

Per-sample signed delta and final signed error use `(a-b+180)%360-180`, in `[-180,180)`. Thus `359 -> 0 -> 1` contributes `+1,+1`, and negative turns work symmetrically. Turn is signed net accumulated rotation, rather than the sum of absolute movements.

Reset without a result on arming, unavailable/nonnumeric/NaN/infinite yaw, implausible radians magnitude above `1e6`, nonpositive update interval, update gap **>1000 ms**, or single wrapped step **>45 degrees**. Exceptions also reset the measurement. Timeout **>120000 ms** from baseline or `abs(turn)>540` aborts with one WARNING and a five-second cooldown. A clock rollback discards the measurement and cooldown. Resets require a new stationary baseline before another attempt.

## DataFlash `YWMR`

New independent call: `logger:write('YWMR', 'Start,End,Turn,Err,Result,Dir', 'ffffBB', ...)`. ArduPilot supplies `TimeUS`. Four floats and two unsigned bytes carry:

| Field | Meaning |
|---|---|
| Start | Captured initial AHRS heading in degrees, normalized to `[0,360)` |
| End | Final AHRS heading in degrees, normalized to `[0,360)` |
| Turn | Accumulated signed rotation in degrees |
| Err | Signed wrapped final-minus-initial error in degrees; classification uses its absolute value |
| Result | `2 PASS` if abs(error)<=2; `3 WARN` if 2<abs(error)<=5; `4 FAIL` if 5<abs(error)<=10 |
| Dir | `1` positive net rotation; `2` negative net rotation |

Incomplete, reset, aborted, armed, or stationary-only attempts write no `YWMR`. A normally completed attempt makes exactly one write call. If logging fails, there is no retry, preventing duplicate records even if the logger wrote before throwing. Firmware logging being disabled, backend loss, heap exhaustion, or logger failure can still prevent a durable record; Lua cannot guarantee physical storage. A thrown logger error also skips that attempt's result STATUSTEXT and enters the diagnostic path. Radio monitoring continues.

## Expected MAVProxy output

Normal radio output remains present. Example magnetic payloads, with MAVProxy's own prefixes omitted:

```text
YWW MagReturn ready: start=311.4 deg
YWW MagReturn PASS err=+0.02 deg turn=+360.0 deg
YWW MagReturn WARN err=+3.50 deg turn=-356.5 deg
YWW MagReturn FAIL err=-8.00 deg turn=+352.0 deg
YWW MagReturn aborted: timeout or excessive rotation
YWW MagReturn error: injected AHRS exception
```

Each result line represents a separate possible attempt. The error sign is final minus initial, wrapped; negative rotation of -356.5 degrees therefore has positive +3.5-degree return error. PASS and ready use INFO (`6`); WARN, FAIL, abort, and magnetic error use WARNING (`4`). Normal attempts send one ready and one result notification, never one per poll. An idle tracking baseline eventually times out and may later be recaptured.

## Offline validation

Run from the repository root:

```bash
lua5.3 tests/yww_v5_2_offline_test.lua lua/radio_health_monitor.lua
lua5.3 tests/test_magnetic_return.lua lua/magnetic_return_embed.lua
lua5.3 tests/test_radio_magnetic_integration.lua lua/radio_health_monitor.lua
luac5.3 -p lua/*.lua tests/*.lua tests/support/*.lua tests/fixtures/*.lua
git diff --check
```

| Check | Result | Count/details |
|---|---|---|
| Radio health tests | PASS | 12 original checks; 12 validated simulated `YWHT` records |
| Magnetic return tests | PASS | 12 supplied scenarios; original summary count corrected from 11 |
| Integration tests | PASS | 57 scenarios executing the actual integrated script |
| Lua syntax validation | PASS | 9 Lua files parsed by `luac5.3` |
| Whitespace validation | PASS | `git diff --check`, plus new-file trailing-whitespace inspection |
| Hardware validation | NOT PERFORMED | No CUAV runtime, SITL, RF, physical heading, or actual DataFlash validation |

**Failures: 0.** Integration coverage includes both rotation directions and error signs; exact/adjacent classification boundaries; missing SiK and GCS; measured baselines; incomplete and reversed partial turns; stationary wraparound noise; baseline and settling drift; settle duration and interrupted settling; armed resets; missing/NaN/infinite yaw; gaps and both jump directions; allowed 1-second/45-degree boundaries; timeout/excess rotation; cooldown and repeat attempts; magnetic API/logger/STATUSTEXT exceptions; radio exceptions/logger isolation; uint32 rollover; original full radio traces; identity/channel/type filters; legacy sysid fallback; and exact radio timeout, low-RSSI persistence, and recovery-spacing boundaries.

The supplied magnetic harness validates the fragment; the new integration harness loads the deployable script normally and invokes its returned callback. Its read-only API doubles expose no arming command, motor, navigation, mode, calibration, or parameter-write API. These are mocked host tests, not evidence of hardware validation.

## Resources, API compatibility, and configuration

The magnetic update has no polling loop, yaw-history array, per-sample log write, parameter write, or flight-control output. It uses a fixed set of scalar fields and functions, with bounded computation per callback. The original radio receive loop remains capped at 25 messages. Additional error text is capped at 50 characters.

Host Lua 5.3 resource probe (10,000 stationary polls per script; harness records discarded; garbage collected before retained-heap readings):

| Measurement | Original radio | Integrated script |
|---|---:|---:|
| Source size | 9,584 bytes | 16,330 bytes |
| Stripped host bytecode | 3,778 bytes | 6,743 bytes |
| Warm retained heap increase including mock environment | 14.85 KiB | 24.10 KiB |
| Retained drift after 10,000 polls | -0.01 KiB | -0.01 KiB |
| Observed host CPU time per poll | 1.22 microseconds | 2.08 microseconds |

The measured warm-heap increment is about **9.25 KiB**. Timing is illustrative and environment-dependent. These host measurements include API doubles and do **not** determine CUAV heap requirements, peak load-time memory, garbage-collection latency, or firmware instruction-budget behavior. `pcall` handles ordinary Lua exceptions; it cannot promise isolation from shared-heap exhaustion or a firmware-level scripting shutdown. Run the checked-in probe with:

```bash
lua5.3 tests/measure_monitor_resources.lua lua/radio_health_monitor.lua
# For a comparison, also supply a saved pre-integration script path.
```

The APIs and custom logger call shape were checked against [ArduPilot's published scripting bindings](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/generator/description/bindings.desc) and the locally available ArduPilot binding implementation. This does not certify the exact installed 4.7.3 firmware image. Only firmware-exposed Lua APIs are used, with no module loading or host-only operations in either production Lua file.

Configuration requirements for later operator review, with **no parameter changes made here**:

- Firmware must support Lua scripting and the required AHRS, arming, MAVLink, and logger bindings; scripting must already be enabled (`SCR_ENABLE=1`) for execution.
- Disarmed DataFlash recording requires appropriate existing logging configuration, including `LOG_DISARMED=1` and an operational backend/storage.
- Keep a valid configured GCS identity for the original startup assertion, even with SiK disconnected.
- Confirm the existing AHRS estimate actually uses EKF3 on the bench. The extension neither selects nor modifies estimator configuration or compass calibration.
- Inspect existing `SCR_HEAP_SIZE`, other active scripts, and available memory. Verify load and sustained operation without scripting-memory errors. Do not infer a safe heap setting from host measurements or increase it automatically. ArduPilot documents that heap sizing can affect memory available to other subsystems in its [Lua scripting guide](https://ardupilot.org/copter/docs/common-lua-scripts.html).

## Known limitations

- This measures **AHRS/EKF heading repeatability**, not raw magnetometer output or absolute magnetic accuracy. An independent physical reference is required to assess heading accuracy.
- Yaw stability cannot establish physical stillness, level orientation, compass calibration quality, or estimator health. Finite but incorrect/stale estimates are not independently diagnosed. EKF lane changes or resets can invalidate an experiment, even if individual deltas remain under 45 degrees.
- The monitor does not prove the operator physically returned to the same indexed orientation. Use visible fixture marks. Noise, drift, tilt, environment changes, cables, and EKF behavior can affect classification.
- Sampling is approximate. Firmware float precision and large uptimes may degrade millisecond resolution; nonpositive intervals reset safely. Mock uint32 rollover coverage does not emulate all embedded numerical behavior. Restart the bench session if repeated timing resets prevent capture.
- Turn is net rotation, so deliberate reversals subtract from it. Completion requires a full net turn rather than sufficient absolute travel.
- Firmware logging availability, telemetry delivery, heap headroom, and scheduling need physical validation. Missing GCS heartbeats do not block measurement, but lack of a telemetry path can hide notifications.

## Physical bench-validation procedure (not performed)

1. Review the diff and save both the current controller script and its configuration. Confirm the unit is a bare CUAV V5+, with no motors/ESCs/frame, kept disarmed. Use a nonmagnetic level surface away from steel, magnets, power supplies, and high-current cables; provide cable slack.
2. Review the existing configuration requirements above. Confirm the firmware/version, EKF3 use, estimated-heading availability, scripting health, logging readiness, and heap headroom. Record configuration; this task does not authorize parameter changes or deployment.
3. In a separately authorized deployment session, copy **only the integrated `radio_health_monitor.lua`** into `/APM/scripts/`. Do not copy the development fragment, tests, a duplicate radio monitor, or the old backup with a `.lua` suffix into the active script directory. Follow the operator's normal script-restart procedure.
4. Watch MAVProxy over a separate USB/telemetry path. With SiK absent and without injecting GCS heartbeats on its channel, confirm radio WAITING/status behavior and magnetic baseline readiness coexist. Leave the board stationary for at least two seconds; record the measured ready heading and one physical index mark.
5. Turn the controller smoothly about its vertical axis through one positive 360-degree turn over roughly 10–20 seconds, keeping it level and stopping at the physical mark. Avoid >45 degrees per sample; finish within 120 seconds. Hold still at least two seconds and record the result. Do not adjust final orientation to chase the displayed estimated heading.
6. Confirm only one result appears. Continue holding still; allow five seconds of cooldown and another two seconds for a fresh ready message. Repeat in the negative direction, then collect several trials in each direction. Capture both physical index alignment and estimated-heading error for each trial.
7. Try a partial 90/180-degree turn and hold; confirm no completion. Observe stationary wraparound noise if available. No physical arming is required for this disarmed procedure; armed-state and injected exception cases were covered only in mocks.
8. Reconnect the radio and use the established radio bench procedure to check heartbeat silence, stale status, low-RSSI latching, and three time-spaced recovery observations while the magnetic monitor is present. Do not infer RF behavior from manually rotating the board alone.
9. Download the real DataFlash log and confirm the exact existing `YWHT` schema and behavior, the independent `YWMR` schema/result/direction fields, and one `YWMR` per completed attempt. Check for scripting errors, memory warnings, estimator resets, missed baseline captures, or duplicate results. Record actual firmware build, configuration, and trial outcomes before claiming hardware validation.

## Rollback

No deployed file was changed, so no controller rollback is currently necessary. For a local rollback, preserve any later user edits first. The exact original radio source can be extracted for review without overwriting the working tree:

```bash
git show fa700b723f5d4ba51d8708891e664612498f687d:lua/radio_health_monitor.lua > /tmp/radio_health_monitor_v5_2_original.lua
```

After review, restore the saved pre-integration script using the normal local or controller maintenance process, with separate deployment authorization. An alternative temporary local disable is setting `MHR_ENABLED=false` in both embedded and development blocks; that bypasses measurement while retaining the original radio callback behavior, but retains the added code's load-time heap cost. Re-run radio tests after either approach. Remove any accidental standalone fragment from the controller's active script directory during authorized maintenance; keep backups outside that directory. No parameter rollback is needed for this task.
