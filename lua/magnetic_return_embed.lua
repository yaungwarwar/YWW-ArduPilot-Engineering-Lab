-- YWW Magnetic Heading Return Monitor (MHRM) - EMBED in radio_health_monitor.lua
-- Offline-only disarmed bench test. Read-only: no arming, actuator, or parameter writes.
-- Uses AHRS/EKF yaw, not an independent raw compass heading measurement.
-- Development source only: never deploy this fragment to /APM/scripts/.
-- The identical block is embedded in radio_health_monitor.lua and invoked with
-- pcall once per existing update cycle, without altering the radio return.
-- Requires ArduPilot scripting bindings: millis, ahrs, arming, gcs, logger.

local MHR_ENABLED = true
local MHR_START_STILL_MS = 2000
local MHR_END_STILL_MS = 2000
local MHR_RESTART_PAUSE_MS = 5000
local MHR_MAX_DURATION_MS = 120000
local MHR_MAX_GAP_MS = 1000
local MHR_MAX_STEP_DEG = 45.0
local MHR_STILL_STEP_DEG = 0.5
local MHR_STILL_WINDOW_DEG = 1.5
local MHR_COMPLETE_DEG = 350.0
local MHR_MAX_TURN_DEG = 540.0
local MHR_PASS_DEG = 2.0
local MHR_WARN_DEG = 5.0
local MHR_SETTLE_ERR_MAX_DEG = 10.0

-- Status codes: 0 WAITING, 1 TRACKING, 2 PASS, 3 WARN, 4 FAIL.
-- Direction codes: 1 positive yaw, 2 negative yaw.
local mhr = {
    state = 0, baseline = nil, prev = nil, prev_ms = nil,
    still_since = nil, still_reference = nil,
    begin_ms = nil, turn = 0, target_still_ms = nil, target_reference = nil,
    pause_until_ms = nil
}
local mhr_last_clock_ms = nil

local function mhr_wrap_diff(a, b)
    -- Signed shortest difference from b to a, in [-180,180).
    return (a - b + 180.0) % 360.0 - 180.0
end

local function mhr_reset()
    mhr.state = 0
    mhr.baseline = nil
    mhr.prev = nil
    mhr.prev_ms = nil
    mhr.still_since = nil
    mhr.still_reference = nil
    mhr.begin_ms = nil
    mhr.turn = 0
    mhr.target_still_ms = nil
    mhr.target_reference = nil
end

local function mhr_finish(now_ms, yaw_deg)
    local err = mhr_wrap_diff(yaw_deg, mhr.baseline)
    local abs_err = math.abs(err)
    local status = 4
    local word = 'FAIL'
    local sev = 4
    if abs_err <= MHR_PASS_DEG then
        status, word, sev = 2, 'PASS', 6
    elseif abs_err <= MHR_WARN_DEG then
        status, word, sev = 3, 'WARN', 4
    end
    local direction = mhr.turn >= 0 and 1 or 2
    local baseline, turn = mhr.baseline, mhr.turn
    -- Consume the attempt BEFORE external calls: even a logger/send_text
    -- exception must never retry a completed result. Preserve this cooldown
    -- across resets (including arming and the scheduler's error handler).
    mhr_reset()
    mhr.pause_until_ms = now_ms + MHR_RESTART_PAUSE_MS
    -- Independent new DataFlash type. Do not edit existing YWHT labels or format.
    logger:write('YWMR', 'Start,End,Turn,Err,Result,Dir', 'ffffBB',
                 baseline, yaw_deg, turn, err, status, direction)
    gcs:send_text(sev, string.format('YWW MagReturn %s err=%+.2f deg turn=%+.1f deg',
                                     word, err, turn))
end

local function magnetic_return_update()
    if not MHR_ENABLED then return end

    local now_ms = millis():tofloat()
    -- A backwards clock (including uint32 uptime rollover) invalidates all
    -- intervals. Start fresh rather than retaining a stale cooldown/baseline.
    if mhr_last_clock_ms and now_ms < mhr_last_clock_ms then
        mhr_reset()
        mhr.pause_until_ms = nil
    end
    mhr_last_clock_ms = now_ms
    if arming:is_armed() then
        mhr_reset()
        return
    end
    if mhr.pause_until_ms and now_ms < mhr.pause_until_ms then return end
    if mhr.pause_until_ms then
        mhr.pause_until_ms = nil
        mhr_reset()
    end

    local yaw_rad = ahrs:get_yaw_rad()
    if type(yaw_rad) ~= 'number' or yaw_rad ~= yaw_rad or
       math.abs(yaw_rad) > 1e6 then -- also rejects both infinities
        mhr_reset()
        return
    end
    local yaw = math.deg(yaw_rad) % 360.0
    if not mhr.prev then
        mhr.prev, mhr.prev_ms = yaw, now_ms
        mhr.still_since, mhr.still_reference = now_ms, yaw
        return
    end

    local dt = now_ms - mhr.prev_ms
    local change = mhr_wrap_diff(yaw, mhr.prev)
    mhr.prev, mhr.prev_ms = yaw, now_ms
    if dt <= 0 or dt > MHR_MAX_GAP_MS or math.abs(change) > MHR_MAX_STEP_DEG then
        mhr_reset() -- discontinuity / stale update: never claim successful loop
        return
    end

    local still = math.abs(change) <= MHR_STILL_STEP_DEG
    if not still then
        mhr.still_since, mhr.still_reference = nil, nil
    elseif not mhr.still_reference or
           math.abs(mhr_wrap_diff(yaw, mhr.still_reference)) > MHR_STILL_WINDOW_DEG then
        mhr.still_since, mhr.still_reference = now_ms, yaw
    end

    if mhr.state == 0 then
        if mhr.still_since and now_ms - mhr.still_since >= MHR_START_STILL_MS then
            mhr.baseline, mhr.begin_ms, mhr.turn = yaw, now_ms, 0
            mhr.state, mhr.target_still_ms = 1, nil
            gcs:send_text(6, string.format('YWW MagReturn ready: start=%.1f deg', yaw))
        end
        return
    end

    mhr.turn = mhr.turn + change
    local abs_turn = math.abs(mhr.turn)
    if now_ms - mhr.begin_ms > MHR_MAX_DURATION_MS or abs_turn > MHR_MAX_TURN_DEG then
        mhr_reset()
        mhr.pause_until_ms = now_ms + MHR_RESTART_PAUSE_MS
        gcs:send_text(4, 'YWW MagReturn aborted: timeout or excessive rotation')
        return
    end

    local abs_err = math.abs(mhr_wrap_diff(yaw, mhr.baseline))
    local near_full_turn = abs_turn >= MHR_COMPLETE_DEG and
                           abs_err <= MHR_SETTLE_ERR_MAX_DEG
    if near_full_turn and still then
        if not mhr.target_still_ms or
           math.abs(mhr_wrap_diff(yaw, mhr.target_reference)) > MHR_STILL_WINDOW_DEG then
            -- Bound total drift during settling, not just each 100-ms step.
            mhr.target_still_ms, mhr.target_reference = now_ms, yaw
        elseif now_ms - mhr.target_still_ms >= MHR_END_STILL_MS then
            mhr_finish(now_ms, yaw)
        end
    else
        mhr.target_still_ms, mhr.target_reference = nil, nil
    end
end
