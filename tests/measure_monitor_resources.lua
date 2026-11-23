-- Host Lua 5.3 resource probe. These numbers do not size an ArduPilot heap.
-- Pass one or more scripts; the same API double is used for each.
local Mock = dofile('tests/support/ardupilot_mock.lua')
assert(#arg>0, 'Supply script path(s)')
for _, path in ipairs(arg) do
    collectgarbage('collect')
    local before = collectgarbage('count')
    local h = Mock.new(path)
    h:hold(311,30)
    -- Do not measure accumulation of records in the host-side test harness.
    h.notices, h.records = {}, {}
    collectgarbage('collect')
    local warm = collectgarbage('count')
    local started = os.clock()
    for _=1,10000 do
        h:tick(311)
        h.notices, h.records = {}, {}
    end
    local elapsed = os.clock()-started
    collectgarbage('collect')
    local retained = collectgarbage('count')
    local fn = assert(loadfile(path))
    local bytecode = #string.dump(fn,true)
    print(string.format('%s: warm heap delta=%.2f KiB; retained drift after 10000 polls=%.2f KiB; stripped bytecode=%d bytes; host time=%.2f us/poll',
        path,warm-before,retained-warm,bytecode,elapsed*1000000/10000))
end
