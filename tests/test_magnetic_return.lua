-- Offline test harness for the embeddable MHRM fragment.
local path = arg[1] or 'magnetic_return_embed.lua'
local f = assert(io.open(path, 'r'))
local source = f:read('*a'); f:close()
local current_ms, yaw_deg, armed = 0, 311.0, false
local notices, records = {}, {}
_G.millis = function() return {tofloat = function() return current_ms end} end
_G.arming = {is_armed = function() return armed end}
_G.ahrs = {get_yaw_rad = function() if yaw_deg == nil then return nil end; return math.rad(yaw_deg) end}
_G.gcs = {send_text = function(self, severity, msg)
    notices[#notices + 1] = {sev=severity, msg=msg}
end}
_G.logger = {write = function(self, name, labels, fmt, ...)
    assert(name == 'YWMR' and fmt == 'ffffBB', 'bad logger format')
    assert(labels == 'Start,End,Turn,Err,Result,Dir', 'bad logger labels')
    records[#records + 1] = {...}
end}
local extension = '\nreturn magnetic_return_update, mhr_reset\n'
local fn = assert(load(source .. extension, '@magnetic_return_embed.lua'))
local tick, reset = fn()
local function step(deg, count, armed_value)
    yaw_deg = deg
    if armed_value ~= nil then armed = armed_value end
    for i=1,(count or 1) do
        current_ms = current_ms + 100
        tick()
    end
end
local function wait_heading(deg, n) step(deg, n or 25) end
local function fresh()
    current_ms = current_ms + 6000
    armed=false; reset(); records={}; notices={}
end
local function rotate(start_deg, amount)
    local sign = amount < 0 and -1 or 1
    local steps = math.floor(math.abs(amount) / 5)
    for i=1,steps do step(start_deg + sign*5*i, 1) end
    local remainder = math.abs(amount) - 5*steps
    if remainder > 0 then step(start_deg+amount, 1) end
    return start_deg+amount
end
local function run_turn(amount, expected_status, expected_dir)
    fresh(); wait_heading(311, 30)
    local ending = rotate(311, amount)
    wait_heading(ending, 25)
    assert(#records == 1, 'expected one recorded test, got '..#records)
    assert(records[1][5] == expected_status, 'wrong result '..tostring(records[1][5]))
    assert(records[1][6] == expected_dir, 'wrong direction '..tostring(records[1][6]))
    assert(math.abs(records[1][3] - amount) < 0.1, 'incorrect accumulated turn')
end
run_turn(360, 2, 1)
run_turn(-360, 2, 2)
run_turn(356, 3, 1) -- 4 degree return error -> WARN
run_turn(351, 4, 1) -- 9 degree return error -> FAIL
fresh(); wait_heading(311, 30); rotate(311, 90); wait_heading(401, 30)
assert(#records == 0, 'should not detect an incomplete rotation')
fresh(); wait_heading(359.7, 30)
for i=1,100 do step(i%2==0 and 359.9 or 0.1,1) end
assert(#records == 0, 'wrap jitter should not trigger')
fresh(); wait_heading(311, 30)
rotate(311, 180); step(491, 1, true)
rotate(491, 180); wait_heading(671, 25)
assert(#records == 0, 'arming must reset test')
fresh(); wait_heading(311, 30); step(nil, 1); rotate(311, 360); wait_heading(671, 25)
assert(#records == 0, 'nil yaw must reset and prevent stale return')
fresh(); wait_heading(311, 30); step(0/0, 1); rotate(311, 360); wait_heading(671, 25)
assert(#records == 0, 'NaN yaw must reset and prevent stale return')
fresh(); wait_heading(311, 30); current_ms = current_ms + 1500
rotate(311, 360); wait_heading(671, 25)
assert(#records == 0, 'sample gap must reset and prevent stale return')
fresh(); wait_heading(311, 30); step(411, 1); rotate(411, 360); wait_heading(771, 25)
assert(#records == 0, 'sample jump must reset and prevent stale return')
fresh(); wait_heading(311, 30); rotate(311, 360); wait_heading(671, 80)
assert(#records == 1, 'must not create duplicate results after completion')
print('PASS: 12 scenarios: positive/negative, warn/fail, incomplete, jitter, armed, nil, NaN, gap, jump, duplicate')
