# YWW V5.3 — Hardware validation: telemetry-loss RETURN180

- **Date:** 2026-11-23
- **Hardware:** Bare CUAV V5+ flight controller, no frame, ESC, motor, or propeller
- **Firmware:** ArduCopter 4.7.3 (`d9c5097c`)
- **Feature:** RETURN180 read-only bench simulation, GCS telemetry-loss-triggered, **manually executed** 180-degree AHRS-heading reversal
- **Evidence reviewed:** Local DataFlash file `ekf3_compass_log19.BIN` (not included in the public repository)

**Report provenance:** This document retains the supplied Log 19 analysis. The
V5.3 test-configuration correction reviewed its wording and arithmetic; it did
not re-decode the private binary or repeat the physical test.

## Executive result

**Bench validation: successful measurement, WARN classification.** After a previously active GCS radio link exceeded its heartbeat timeout, the Lua script generated a 180-degree opposite-heading target. The operator manually rotated the bare flight controller, held it stable, and the script recorded one `YWRT` result. This verifies event detection, target calculation, manual-rotation measurement, classification, and DataFlash output. **It does not verify autonomous yaw control or geographic return-to-home.**

| Measurement | DataFlash value |
|---|---:|
| Starting AHRS heading (`YWRT.Start`) | 133.087067° |
| Target heading (`YWRT.Target`) | 313.087067° |
| Final AHRS heading (`YWRT.End`) | 310.043762° |
| Wrapped error (`YWRT.Err`) | **−3.043304°** |
| Classification (`YWRT.Result`) | **3 — WARN** |
| Trigger source (`YWRT.Source`) | **1 — GCS heartbeat timeout** |
| Manual rotation direction (`YWRT.Dir`) | **1 — positive net rotation** |
| `YWRT` records in this log | **1** |
| `YWHT` radio-health records | 3 |
| `YWMR` independent 360° test records | 0 |

This **−3.043°** result is from **Log 19** and must not be conflated with a **+2.22°** WARN result reported separately in MAVProxy console output from an earlier trial.

## Event chronology

Times below are seconds since the beginning of this log and are approximate.

| Time | Data source | Observation |
|---:|---|---|
| 4.745 s | `MSG` | Healthy radio state `YWW LINK_ACTIVE`, channel 1 |
| 12.716 s | `YWHT` | `S=3` (`GCS_SILENT`), channel 1, GCS heartbeat age 3020 ms |
| 12.721 s | `MSG` | `YWW Telemetry LOST (SIM)` |
| 12.721 s | `MSG` | `YWW RETURN180 SIM start=133.1 target=313.1` |
| 17.852 s | `YWHT` | `S=4` (`RF_LOSS_SUSPECTED`), RSSI reported 51; `Lat=1`, `Flt=4` |
| 24.112 s | `YWRT` | One WARN result persisted, `Err=−3.043304°` |
| 24.114 s | `MSG` | `YWW RETURN180 SIM WARN err=-3.04` |

The first `YWHT` loss record shows **heartbeat timeout with fresh radio status** (`SA=205 ms`), not proof of deliberate RF interference. The subsequent suspected RF-loss event is an additional radio monitor classification, not proof of jamming.

## Internal consistency and estimator status

The decoded `YWRT` arithmetic is consistent:

- `(133.087067 + 180) mod 360 = 313.087067` degrees.
- Wrapped `(310.043762 − 313.087067) = −3.043304` degrees.
- `|−3.043304|` is above the 2° PASS limit and within the 5° WARN limit.
- `ATT.Yaw` near the event is approximately 310.044°, consistent with the custom log record.

Additional observations in the recording:

| Check | Observation |
|---|---|
| `MAG` health flag | 556 of 556 samples marked healthy |
| Magnetic field magnitude | Mean approximately 524.87 mG; SD approximately 9.45 mG |
| EKF3 cores | Two logged cores |
| `XKF4.FS` | 0 on both cores |
| Maximum `XKF4.SM` | 0.02 on each core |
| Primary EKF core | Core 0 throughout observed data |

These indicators support consistent estimator operation during this bench test, but do not independently validate absolute compass accuracy or true physical angular displacement.

## Constraints and limitations

- **Read-only simulation:** No arming, mode change, servo/motor output, yaw setpoint, trajectory, home bearing, or navigation command was executed.
- **Manual motion:** Completion measures the AHRS/EKF's reported heading after the operator manually rotates the bare controller; it is not an automatic turn.
- **Failure mechanism:** A missing GCS heartbeat can be caused by unplugging ground equipment, host-side shutdown, routing interruption, or radio failure. It does not alone diagnose RF interference or GNSS failure.
- **Physical reference:** These values are heading-estimate errors relative to the calculated opposite heading, not independently measured physical heading errors.
- **Recovery:** This **Log 19 extract does not establish a completed recovery after this particular trigger**. Recovery was separately observed in MAVProxy during earlier tests.
- **Log integrity note:** The binary record parser reported a partially truncated trailing `RATE` record (35/63 bytes), while the earlier `YWRT`, `YWHT`, `ATT`, and estimator records used for this report were complete.
- **Privacy:** Do not publish raw flight logs or unreviewed raw parameter dumps. These can disclose hardware identifiers, locations, or other operational details.

## Test status and publication scope

- **Hardware findings reported for this trial:** Lua-executed telemetry loss event, opposite-heading computation, manual heading-target check, one `YWRT` DataFlash record, WARN classification for this sample, radio-health output, and no reported EKF fault flags in this recording. One WARN sample does not validate every classification boundary.
- **Not hardware-verified by Log 19:** Autonomous yaw response, flight behavior, geographic return-to-home, GNSS interference diagnosis, and a new completed link-recovery event after this specific trigger.
- **Offline suites after configuration correction:** The [V5.3 bench/test report](TELEMETRY_LOSS_RETURN180_BENCH_TEST.md) records **161 checks/scenarios passed, 0 failed** (12 radio, 12 magnetic fragment, 58 integration, 79 RETURN180). These host tests are distinct from the supplied hardware findings in this document.

## Reproducibility

Public repository should contain the deployed-source candidate `lua/radio_health_monitor.lua`, the regression tests, and `docs/TELEMETRY_LOSS_RETURN180_BENCH_TEST.md`, plus this validation summary. The raw `ekf3_compass_log19.BIN` is intentionally retained privately. If a sanitized event-only reproduction fixture is later needed, create it from the `YWRT`/`YWHT` fields with hardware IDs, coordinates, and unrelated telemetry removed, and verify it before committing.

The V5.3 source candidate keeps `MHR_ENABLED = false`; the separate legacy
360-degree heading-return monitor generates no `YWMR`, baseline, or timeout
messages. Both diagnostics are enabled together only by the host integration
test loader. RETURN180 remains a read-only bench simulation with manual motion.
Any future autonomous heading-control implementation requires separate design
and validation; this release provides no such control or flight failsafe.
