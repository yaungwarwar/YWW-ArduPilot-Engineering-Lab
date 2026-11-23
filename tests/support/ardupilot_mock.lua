-- Host-side, read-only API double. No hardware or firmware execution.
local Mock = {}
local U32 = 4294967296
local Time = {}
Time.__index = Time
function Time:toint()
    return self.value < U32 / 2 and self.value or self.value - U32
end
function Time:tofloat() return self.value end
function Time.__sub(a, b)
    return setmetatable({value = (a.value - b.value) % U32}, Time)
end

function Mock.new(path, options)
    local opts = options or {}
    local h = {clock = opts.clock or 0, yaw = 311, armed = false,
               notices = {}, records = {}, queue = {}, writes = {}, ids = {}}
    local env = {assert=assert, error=error, ipairs=ipairs, pairs=pairs,
                 math=math, pcall=pcall, string=string, tostring=tostring,
                 tonumber=tonumber, type=type}
    env.millis = function()
        return setmetatable({value = h.clock % U32}, Time)
    end
    env.param = {get = function(_, key)
        if opts.params then return opts.params[key] end
        if key == 'MAV_GCS_SYSID' then return 255 end
        return nil
    end}
    env.arming = {is_armed = function()
        if h.arm_error then error('arming unavailable') end
        return h.armed
    end}
    env.ahrs = {get_yaw_rad = function()
        if h.yaw_error then error('injected AHRS exception') end
        if h.yaw == nil then return nil end
        return math.rad(h.yaw)
    end}
    env.gcs = {send_text = function(_, severity, message)
        if h.text_error and message:match('^YWW MagReturn') then
            error('injected STATUSTEXT exception')
        end
        h.notices[#h.notices+1] = {clock=h.clock, severity=severity, message=message}
    end}
    env.logger = {write = function(_, name, labels, fmt, ...)
        assert((name == 'YWHT' and labels == 'S,Ch,R,HB,SA,Rec,Lat,Flt,RR,N,Tx,Err'
                and fmt == 'BBiiiBBIiiii') or
               (name == 'YWMR' and labels == 'Start,End,Turn,Err,Result,Dir'
                and fmt == 'ffffBB'), 'unexpected DataFlash schema')
        local values = {...}
        assert(#values == #fmt)
        h.writes[name] = (h.writes[name] or 0) + 1
        if h.log_error == name then error('injected logger exception') end
        h.records[#h.records+1] = {clock=h.clock, name=name, labels=labels,
                                  fmt=fmt, values=values}
        if h.log_after_error == name then error('exception after logging') end
    end}
    env.mavlink = {
        init = function(_, size, count) assert(size == 25 and count == 2) end,
        register_rx_msgid = function(_, id) h.ids[id] = true end,
        receive_chan = function()
            if h.radio_error then error('injected radio exception') end
            if #h.queue == 0 then return nil end
            local item = table.remove(h.queue, 1)
            return item.msg, item.chan
        end,
    }
    h.callback, h.delay = assert(loadfile(path, 't', env))()
    assert(type(h.callback) == 'function' and h.delay == 100)
    setmetatable(h, {__index=Mock})
    return h
end

function Mock:tick(deg, dt)
    self.yaw = deg
    self.clock = self.clock + (dt or 100)
    if self.before_tick then self:before_tick() end
    local callback, delay = self.callback()
    assert(type(callback) == 'function')
    assert(delay == (self.radio_error and 1000 or 100), 'scheduler behavior changed')
    self.callback, self.delay = callback, delay
end
function Mock:hold(deg, count)
    for _=1,(count or 25) do self:tick(deg) end
end
function Mock:rotate(start, amount, size)
    size = size or 5
    local sign = amount < 0 and -1 or 1
    local count = math.floor(math.abs(amount)/size)
    for i=1,count do self:tick(start + sign*size*i) end
    if math.abs(amount) > count*size then self:tick(start+amount) end
    return start+amount
end
function Mock:logs(name)
    local out = {}
    for _, r in ipairs(self.records) do
        if r.name == name then out[#out+1] = r end
    end
    return out
end
function Mock:texts(pattern)
    local out = {}
    for _, n in ipairs(self.notices) do
        if n.message:match(pattern) then out[#out+1] = n end
    end
    return out
end
function Mock:send(id, payload, sysid, chan)
    assert(self.ids[id])
    local msg = '\0\0' .. string.pack('<BBBBBBBI3',
        253, #payload, 0, 0, 0, sysid or 1, 1, id) .. payload
    self.queue[#self.queue+1] = {msg=msg, chan=chan or 1}
end
function Mock:radio(rssi, chan)
    self:send(109, string.pack('<I2I2BBBBB', 17, 0, rssi, 180, 97, 30, 31), 1, chan)
end
function Mock:heartbeat(sysid, chan, mavtype)
    self:send(0, string.pack('<I4BBBBB', 0, mavtype or 6, 8, 0, 4, 3), sysid or 255, chan)
end
return Mock
