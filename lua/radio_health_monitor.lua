-- YWW RF Health Monitor V5.2 (CUAV V5+ / ArduRover 4.7)
-- READ ONLY. Bench diagnostic, NOT a flight failsafe.
-- Subscribe to SiK RADIO_STATUS (109) and approved GCS HEARTBEAT (0).
-- V5.2 requires time-spaced GCS heartbeats for recovery (not just a burst).
-- Preserves V5.1 DataFlash (YWHT) format for compatibility.
-- LOG_DISARMED=1 is required to record while bench-testing disarmed.

local RADIO_ID, HEARTBEAT_ID = 109, 0
local HB_TIMEOUT_MS, STATUS_TIMEOUT_MS = 3000, 3500
local LOW_RSSI, LOW_HOLD_MS = 90, 3000  -- experimentally chosen, not dBm
local RECOVERY_REQUIRED, REPORT_MS = 3, 5000
local RECOVERY_MIN_GAP_MS = 700 -- time between accepted recovery heartbeats
local LOG_INTERVAL_MS = 30000  -- plus immediate record on state transitions

-- YWHT State codebook (logged as an unsigned byte):
-- 0 WAITING, 1 GCS_UNSEEN, 2 LINK_ACTIVE, 3 GCS_SILENT,
-- 4 RF_LOSS_SUSPECTED, 5 STATUS_STALE, 6 RECOVERING.
local STATE_CODE = {
    WAITING = 0, GCS_UNSEEN = 1, LINK_ACTIVE = 2,
    GCS_SILENT = 3, RF_LOSS_SUSPECTED = 4,
    STATUS_STALE = 5, RECOVERING = 6,
}

-- Custom DataFlash log name is <= 4 chars. The logger adds TimeUS.
-- S=state, Ch=MAVLink channel, R=local RSSI, HB=heartbeat age (ms),
-- SA=status age (ms), Rec=recovery count, Lat=latched fault flag,
-- Flt=number of suspected-loss events since start,
-- RR=remote RSSI, N=local noise, Tx=free TX buffer %, Err=RX error count.
local LOG_NAME = 'YWHT'
local LOG_LABELS = 'S,Ch,R,HB,SA,Rec,Lat,Flt,RR,N,Tx,Err'
local LOG_FORMAT = 'BBiiiBBIiiii'  -- 12 fields, well within ArduPilot limits

-- ArduPilot 4.7 renamed SYSID_MYGCS to MAV_GCS_SYSID.
-- Match the autopilot's configured GCS identity; fail closed if unknown.
local gcs_id = param:get('MAV_GCS_SYSID') or param:get('SYSID_MYGCS')
assert(gcs_id and gcs_id >= 1 and gcs_id <= 255, 'YWW: GCS SYSID unavailable')
local gcs_hi = param:get('MAV_GCS_SYSID_HI')
local function allowed_gcs(sysid)
    if gcs_hi and gcs_hi >= gcs_id then
        return sysid >= gcs_id and sysid <= gcs_hi
    end
    return sysid == gcs_id
end

mavlink:init(25, 2)
mavlink:register_rx_msgid(RADIO_ID)
mavlink:register_rx_msgid(HEARTBEAT_ID)

local radio_chan = nil
local last_radio = nil          -- uint32 millis object, not signed integer
local last_rssi = -1            -- -1 = unknown/invalid
local last_noise, last_remrssi = -1, -1
local last_txbuf, last_errors = -1, -1
local gcs_seen = {}             -- last approved GCS heartbeat per channel
local low_since = nil
local rf_fault_latched = false  -- persists across STATUS_STALE
local rf_fault_count = 0       -- history since this script started
local recovery_count = 0
local recovery_last_accepted = nil -- millis() of last counted recovery heartbeat
local state = 'WAITING'
local last_report = millis()
local last_error_report = nil
local last_log = nil
local log_enabled = true -- disabled if logger:write raises a Lua error

gcs:send_text(6, string.format('YWW V5.2 started GCS=%d', gcs_id))

local function age_ms(now, past)
    if not past then return -1 end
    -- Subtract uint32 timestamps BEFORE toint(), to tolerate uptime wrap.
    return (now - past):toint()
end

local function step()
    local now = millis()
    local hb_batch = {}         -- count incoming approved heartbeat messages by channel

    for _ = 1, 25 do
        local msg, chan = mavlink:receive_chan()
        if not msg then break end
        if type(msg) == 'string' and chan ~= nil and #msg >= 13 then
            local msgid = string.unpack('<I3', msg, 10)

            if msgid == RADIO_ID and #msg >= 21 then
                -- RADIO_STATUS: rxerrors, fixed, rssi, remrssi, txbuf, noise, remnoise
                local rxerrors, _, rssi, remrssi, txbuf, noise =
                    string.unpack('<I2I2BBBBB', msg, 13)
                if radio_chan ~= nil and radio_chan ~= chan then
                    -- Avoid counting unrelated heartbeats from previous channel.
                    low_since = nil
                    recovery_count = 0
                    recovery_last_accepted = nil
                end
                radio_chan = chan
                last_radio = now
                last_rssi = rssi == 255 and -1 or rssi
                last_noise = noise == 255 and -1 or noise
                last_remrssi = remrssi == 255 and -1 or remrssi
                last_txbuf = txbuf
                last_errors = rxerrors

            elseif msgid == HEARTBEAT_ID and #msg >= 21 then
                local sysid = msg:byte(8)
                local mav_type = msg:byte(17)
                if mav_type == 6 and allowed_gcs(sysid) then
                    gcs_seen[chan] = now
                    hb_batch[chan] = (hb_batch[chan] or 0) + 1
                end
            end
        end
    end

    local hb_age = (radio_chan ~= nil) and age_ms(now, gcs_seen[radio_chan]) or -1
    local status_age = age_ms(now, last_radio)
    local new_hb_count = (radio_chan ~= nil and hb_batch[radio_chan]) or 0
    local previous = state

    if status_age < 0 then
        state = 'WAITING'
        low_since = nil
        recovery_count = 0
        recovery_last_accepted = nil

    elseif status_age > STATUS_TIMEOUT_MS then
        state = 'STATUS_STALE'
        -- Last RSSI is stale. Reset the low-RSSI timer, NOT fault history.
        low_since = nil
        recovery_count = 0
        recovery_last_accepted = nil

    elseif hb_age < 0 then
        state = rf_fault_latched and 'RF_LOSS_SUSPECTED' or 'GCS_UNSEEN'
        low_since = nil
        recovery_count = 0
        recovery_last_accepted = nil

    elseif hb_age > HB_TIMEOUT_MS then
        recovery_count = 0
        recovery_last_accepted = nil
        if last_rssi >= 0 and last_rssi < LOW_RSSI then
            if not low_since then low_since = now end
            if age_ms(now, low_since) >= LOW_HOLD_MS and not rf_fault_latched then
                rf_fault_latched = true
                rf_fault_count = rf_fault_count + 1
            end
        else
            -- High/unknown RSSI cannot establish low-signal duration.
            low_since = nil
        end
        state = rf_fault_latched and 'RF_LOSS_SUSPECTED' or 'GCS_SILENT'

    else
        low_since = nil
        if previous == 'RECOVERING' or previous == 'GCS_SILENT' or
           previous == 'RF_LOSS_SUSPECTED' or previous == 'STATUS_STALE' then
            if previous ~= 'RECOVERING' then
                recovery_count = 0
                recovery_last_accepted = nil
            end
            -- Count at most one heartbeat per update. All messages read in
            -- this poll share the same receive timestamp, even if buffered.
            -- Requiring an arrival-time gap prevents one burst of queued
            -- heartbeats from prematurely clearing a suspected RF fault.
            if new_hb_count > 0 and
               (not recovery_last_accepted or
                age_ms(now, recovery_last_accepted) >= RECOVERY_MIN_GAP_MS) then
                recovery_count = recovery_count + 1
                recovery_last_accepted = now
            end
            if recovery_count >= RECOVERY_REQUIRED then
                state = 'LINK_ACTIVE'
                rf_fault_latched = false  -- only clear after time-qualified recovery
            else
                state = 'RECOVERING'
            end
        else
            state = 'LINK_ACTIVE' -- cold startup, no prior failure observed
        end
    end

    local rssi_report = status_age >= 0 and status_age <= STATUS_TIMEOUT_MS
                        and last_rssi or -1
    if previous ~= state or age_ms(now, last_report) >= REPORT_MS then
        gcs:send_text(6, string.format(
            'YWW %s c=%d r=%d hb=%d st=%d rec=%d lat=%d faults=%d',
            state, radio_chan or -1, rssi_report, hb_age, status_age,
            recovery_count, rf_fault_latched and 1 or 0, rf_fault_count))
        last_report = now
    end

    -- One DataFlash record on every state transition plus one every 30 s.
    -- Log unknown fields as -1; never reuse old radio metrics after STATUS_STALE.
    if log_enabled and (previous ~= state or not last_log or
                        age_ms(now, last_log) >= LOG_INTERVAL_MS) then
        local status_fresh = status_age >= 0 and status_age <= STATUS_TIMEOUT_MS
        local rem_report = status_fresh and last_remrssi or -1
        local noise_report = status_fresh and last_noise or -1
        local tx_report = status_fresh and last_txbuf or -1
        local errors_report = status_fresh and last_errors or -1

        -- Isolate logging errors from the monitor's state machine.
        local ok, err = pcall(function()
            logger:write(LOG_NAME, LOG_LABELS, LOG_FORMAT,
                STATE_CODE[state] or 0, radio_chan or 255,
                rssi_report, hb_age, status_age,
                recovery_count, rf_fault_latched and 1 or 0,
                rf_fault_count, rem_report, noise_report,
                tx_report, errors_report)
        end)
        if ok then
            last_log = now
        else
            log_enabled = false
            gcs:send_text(3, 'YWW logging disabled: ' .. tostring(err):sub(1, 60))
        end
    end
end

-- BEGIN embedded magnetic_return_embed.lua
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
-- END embedded magnetic_return_embed.lua

-- Magnetic errors have their own throttle and never affect the radio delay.
local last_mhr_error_report = nil
local function report_magnetic_error(err)
    local now = millis()
    if not last_mhr_error_report or age_ms(now, last_mhr_error_report) >= 10000 then
        last_mhr_error_report = now
        local message = tostring(err):gsub('^.-:%d+: ', '')
        gcs:send_text(4, 'YWW MagReturn error: ' .. message:sub(1, 29))
    end
end

local function protected_update()
    local ok, err = pcall(step)
    if not ok then
        local now = millis()
        if not last_error_report or age_ms(now, last_error_report) >= 10000 then
            gcs:send_text(3, 'YWW V5.2 error: ' .. tostring(err):sub(1, 80))
            last_error_report = now
        end
    end
    local mhr_ok, mhr_err = pcall(magnetic_return_update)
    if not mhr_ok then
        mhr_reset()
        -- Protect diagnostics too: a telemetry failure must not stop the radio.
        pcall(report_magnetic_error, mhr_err)
    end
    return protected_update, ok and 100 or 1000
end

return protected_update, 100
