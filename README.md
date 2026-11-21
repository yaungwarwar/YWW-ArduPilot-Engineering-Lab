# YWW ArduPilot Engineering Lab

Experimental ArduPilot engineering projects, Lua scripts, MAVLink telemetry tools, and flight-controller validation.

## Hardware

- Flight Controller: CUAV V5+
- Firmware: ArduCopter 4.7.3
- Companion development environment: Ubuntu 24.04
- Ground tools: MAVProxy and Mission Planner
- Telemetry: SiK radio modules

## Projects

### 1. YWW RF Health Monitor V5.2

A read-only ArduPilot Lua telemetry-monitoring system.

Features:
- Ground-control heartbeat monitoring
- RADIO_STATUS monitoring
- Radio-link state detection
- RF loss detection and fault latching
- Recovery confirmation
- DataFlash logging
- Offline state-machine tests

The RF monitor was physically validated on ArduRover 4.7.1.
Revalidation on ArduCopter 4.7.3 is pending.

### 2. Compass and EKF3 Experiments

Bench investigations include:
- IST8310 magnetometer calibration
- Hard-iron offset estimation
- Four-position heading measurements
- EKF3 yaw estimation
- Heading repeatability
- Yaw stability
- EKF innovation and status monitoring

These are engineering experiments, not flight-readiness certification.

## Repository Structure

- `lua/` - ArduPilot Lua scripts
- `tests/` - Offline validation tests
- `docs/` - Engineering documentation

## Offline Validation

Run:

```bash
lua tests/yww_v5_2_offline_test.lua lua/radio_health_monitor.lua
