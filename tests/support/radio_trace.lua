-- Deterministic radio-only replay; fixture captured from the pre-integration
-- V5.2 source. Compare every YWHT field and radio STATUSTEXT with that fixture.
local Trace = {}
function Trace.run(h)
    local function run(ms, rssi, sysid)
        for _=1,ms/100 do
            if (h.clock+100)%1000 == 0 then
                if rssi then h:radio(rssi) end
                if sysid then h:heartbeat(sysid) end
            end
            h:tick(h.yaw)
        end
    end
    local function burst(count)
        for _=1,count do h:heartbeat() end
        h:tick(h.yaw)
    end
    run(4000, 200, 255)
    run(5000, 200)
    run(5000, 35)
    run(4500)
    run(5000, 190)
    run(4000, 190, 42)
    burst(3)
    run(300, 190); burst(1)
    run(400, 190); burst(1)
    run(800, 190); burst(1)
    run(6000, 255)
    run(5000, nil, 255)
    run(32000) -- verify periodic 30-s log and repeated status reports
end
function Trace.capture(h)
    local out = {}
    for _, n in ipairs(h.notices) do
        if not n.message:match('^YWW MagReturn') then
            out[#out+1] = string.format('T|%d|%d|%s', n.clock, n.severity, n.message)
        end
    end
    for _, r in ipairs(h:logs('YWHT')) do
        local values = {}
        for i,v in ipairs(r.values) do values[i] = tostring(v) end
        out[#out+1] = string.format('L|%d|%s|%s|%s|%s',
            r.clock, r.name, r.labels, r.fmt, table.concat(values, ','))
    end
    return out
end
return Trace
