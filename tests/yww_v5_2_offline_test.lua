-- YWW V5.2 offline logic harness (no flight controller or RF hardware required)
-- Usage: lua ~/yww_v5_2_offline_test.lua ~/radio_health_v5_2.lua
-- Lua 5.3+ required for string.pack/string.unpack.

local script_path = assert(arg[1], 'Supply the YWW V5 Lua script path')
local clock = 0
local queue = {}
local logs = {}
local latest = ''
local ids = {}

-- Approximate the uint32 millis userdata used by ArduPilot.
local U32 = 4294967296
local Time = {}
Time.__index = Time
function Time:toint()
    return self.value < 2147483648 and self.value or self.value - U32
end
function Time.__sub(a, b)
    return setmetatable({value = (a.value - b.value) % U32}, Time)
end
function millis()
    return setmetatable({value = clock % U32}, Time)
end

param = {}
function param:get(key)
    if key == 'MAV_GCS_SYSID' then return 255 end
    if key == 'MAV_GCS_SYSID_HI' then return 0 end
    return nil
end

gcs = {}
function gcs:send_text(_severity, message)
    logs[#logs + 1] = message
    if message:match('^YWW [A-Z_]+ c=') then latest = message end
    print(string.format('%7.1fs  %s', clock / 1000, message))
end

-- Collect simulated DataFlash records; validate format and capture state changes.
local records = {}
logger = {}
function logger:write(name, labels, fmt, ...)
    assert(name == 'YWHT', 'Unexpected DataFlash record name')
    assert(#name <= 4 and #labels < 58 and #fmt < 15)
    local vals = {...}
    local cols = 1
    for _ in labels:gmatch(',') do cols = cols + 1 end
    assert(cols == #fmt and #vals == #fmt, 'DataFlash format/argument mismatch')
    assert(fmt == 'BBiiiBBIiiii', 'Unexpected YWHT field types')
    records[#records + 1] = {clock=clock, values=vals}
end

mavlink = {}
function mavlink:init(_queue_size, _registration_count) end
function mavlink:register_rx_msgid(msgid) ids[msgid] = true end
function mavlink:receive_chan()
    if #queue == 0 then return nil end
    local item = table.remove(queue, 1)
    return item.msg, item.chan
end

local function make_msg(msgid, payload, sysid)
    -- ArduPilot Lua receive_chan(): 2-byte checksum, 10-byte MAVLink v2
    -- header, then payload starting at Lua byte 13.
    return '\0\0' .. string.pack('<BBBBBBBI3',
        253, #payload, 0, 0, 0, sysid or 1, 1, msgid) .. payload
end

local function send_radio(rssi)
    assert(ids[109])
    local payload = string.pack('<I2I2BBBBB', 0, 0, rssi, rssi, 100, 30, 30)
    queue[#queue + 1] = {msg=make_msg(109, payload, 1), chan=1}
end
local function send_hb(sysid)
    assert(ids[0])
    local payload = string.pack('<I4BBBBB', 0, 6, 8, 0, 4, 3)
    queue[#queue + 1] = {msg=make_msg(0, payload, sysid), chan=1}
end

local fn, period = assert(loadfile(script_path))()
assert(type(fn) == 'function' and period == 100,
    'Monitor must return an update callback scheduled at 100 ms')

local function run_for(duration, radio_rssi, heartbeat_sysid)
    assert(duration % 100 == 0)
    for _ = 1, duration / 100 do
        clock = clock + 100
        if clock % 1000 == 0 then
            if radio_rssi ~= nil then send_radio(radio_rssi) end
            if heartbeat_sysid ~= nil then send_hb(heartbeat_sysid) end
        end
        local next_fn, next_ms = fn()
        assert(type(next_fn) == 'function' and next_ms >= 100)
        fn = next_fn
    end
end

local checks = 0
local function verify(label, state, fragments)
    assert(latest:match('^YWW ' .. state .. ' c='),
        label .. ': expected ' .. state .. ', got: ' .. latest)
    for _, text in ipairs(fragments or {}) do
        assert(latest:find(text, 1, true), label .. ': missing ' .. text .. ': ' .. latest)
    end
    checks = checks + 1
    print('       PASS: ' .. label)
end

run_for(4000, 200, 255)
verify('Normal ground link', 'LINK_ACTIVE', {'lat=0', 'faults=0'})

run_for(5000, 200, nil)
verify('GCS silent, signal still present', 'GCS_SILENT', {'lat=0', 'faults=0'})

run_for(5000, 35, nil)
verify('Sustained low RSSI latches fault', 'RF_LOSS_SUSPECTED', {'lat=1', 'faults=1'})

run_for(4500, nil, nil)
verify('Status lost, fault history retained', 'STATUS_STALE', {'r=-1', 'lat=1', 'faults=1'})

run_for(5000, 190, nil)
verify('Status returns, stale low reading not reused', 'RF_LOSS_SUSPECTED',
       {'lat=1', 'faults=1'})

run_for(4000, 190, 42)
verify('Unauthorized GCS ignored', 'RF_LOSS_SUSPECTED', {'lat=1', 'faults=1'})

-- Inject multiple valid heartbeats into one 100ms poll. V5.1 would
-- incorrectly accept all three immediately and clear the fault latch.
local function send_burst(count)
    for _ = 1, count do send_hb(255) end
    clock = clock + 100
    local next_fn, next_ms = fn()
    assert(type(next_fn) == 'function' and next_ms >= 100)
    fn = next_fn
end

send_burst(3)
verify('Three buffered heartbeats count as one', 'RECOVERING',
       {'rec=1', 'lat=1', 'faults=1'})

-- A fourth heartbeat arrives only 400ms after the first accepted one.
run_for(300, 190, nil)
send_burst(1)
verify('Closely spaced heartbeat cannot finish recovery', 'RECOVERING',
       {'rec=1', 'lat=1'})

-- Another heartbeat >=700ms after the last accepted heartbeat counts.
run_for(400, 190, nil)
send_burst(1)
verify('Second spaced heartbeat remains in RECOVERING', 'RECOVERING',
       {'lat=1', 'faults=1'})

run_for(800, 190, nil)
send_burst(1)
verify('Three time-spaced heartbeats clear active latch', 'LINK_ACTIVE',
       {'rec=3', 'lat=0', 'faults=1'})

run_for(6000, 255, nil)
verify('RSSI 255 is unknown, not new RF loss', 'GCS_SILENT', {'r=-1', 'faults=1'})

run_for(5000, nil, 255)
verify('Status stale despite fresh approved GCS heartbeats', 'STATUS_STALE',
       {'r=-1', 'faults=1'})

-- State codes: 2 active, 3 silent, 4 suspected RF loss, 5 status stale, 6 recovering.
assert(#records > 10 and #records < 30, 'Unexpected logger record count ' .. #records)
local seen = {}
for _, rec in ipairs(records) do
    local v = rec.values
    local code, ch, rssi, hbage, st_age, recovery, latch, faults =
          v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
    assert(type(code)=='number' and (ch==1 or ch==255) and type(hbage)=='number')
    seen[code] = true
    if code == 5 then
        assert(rssi == -1 and v[9] == -1 and v[10] == -1,
            'Stale status must log unknown RF fields')
    end
    if code == 4 then assert(latch == 1 and faults >= 1) end
    if code == 6 then assert(recovery >= 1) end
end
for _, code in ipairs({2,3,4,5,6}) do assert(seen[code], 'No logged state code: ' .. code) end
print(string.format('PASS: %d simulated DataFlash records; log format and state transitions.', #records))
print(string.format('\nPASS: %d offline state-machine checks.', checks))
