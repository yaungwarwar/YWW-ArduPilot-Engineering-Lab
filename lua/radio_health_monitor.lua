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

local function protected_update()
    local ok, err = pcall(step)
    if not ok then
        local now = millis()
        if not last_error_report or age_ms(now, last_error_report) >= 10000 then
            gcs:send_text(3, 'YWW V5.2 error: ' .. tostring(err):sub(1, 80))
            last_error_report = now
        end
    end
    return protected_update, ok and 100 or 1000
end

return protected_update, 100
