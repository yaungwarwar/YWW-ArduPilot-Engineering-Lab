-- Full-script V5.3 bench simulation regression tests; no hardware execution.
-- Run from repository root: lua5.3 tests/test_return180.lua lua/radio_health_monitor.lua
local path = arg[1] or 'lua/radio_health_monitor.lua'
local Mock = dofile('tests/support/ardupilot_mock.lua')
local checks, failed = 0, 0
local harnesses = {}
local function test(name, fn)
    harnesses = {}
    local ok, err = pcall(fn)
    for _, h in ipairs(harnesses) do
        if #h.forbidden_calls>0 then
            ok,err=false,'forbidden control/API call: '..table.concat(h.forbidden_calls,',')
        end
        if h.configuration=='production' and
           (h.writes.YWMR or #h:texts('^YWW MagReturn')>0) then
            ok,err=false,'legacy magnetic monitor ran in production configuration'
        end
    end
    checks=checks+1
    if ok then print('PASS: '..name)
    else failed=failed+1; print('FAIL: '..name..': '..tostring(err)) end
end
local function fresh(opts)
    local h=Mock.new(path,opts); harnesses[#harnesses+1]=h; return h
end
local function near(a,b) assert(math.abs(a-b)<0.00001,a..' differs from '..b) end
local function count(h,pattern) return #h:texts(pattern) end
local function starts(h) return count(h,'^YWW RETURN180 SIM start=') end
local function empty(h) assert(#h:logs('YWRT')==0,'unexpected completion') end
local function healthy(h,start)
    start=start or 35
    h:radio(200,1); h:heartbeat(255,1); h:tick(start)
    local radio=h:logs('YWHT')
    assert(radio[#radio].values[1]==2,'healthy link not established')
    assert(starts(h)==0)
end
local function loss(h,mode,yaw)
    local n=mode=='status' and 36 or 31
    for _=1,n do
        if mode=='heartbeat' then h:radio(200,1) end
        if mode=='status' then h:heartbeat(255,1) end
        h:tick(yaw)
    end
end
local function pending(mode,start,opts)
    local h=fresh(opts); start=start or 35
    healthy(h,start); loss(h,mode or 'heartbeat',start)
    assert(starts(h)==1 and count(h,'^YWW Telemetry LOST %(SIM%)')==1)
    empty(h); return h
end
local function completion(h,start,amount,status,source)
    local ending=h:rotate(start,amount)
    h:hold(ending,21)
    local records=h:logs('YWRT')
    assert(#records==1,'expected exactly one completion, got '..#records)
    local v=records[1].values
    near(v[1],start%360); near(v[2],(start+180)%360)
    near(v[3],ending%360); near(v[4],(amount+180+180)%360-180)
    assert(v[5]==status and v[6]==source and v[7]==(amount>=0 and 1 or 2),
        'incorrect classification, trigger source, or direction')
    local notices=h:texts('^YWW RETURN180 SIM [PWF]')
    assert(#notices==1 and notices[1].severity==(status==2 and 6 or 4))
    return ending
end
local function recover(h,yaw)
    h:radio(200,1); h:heartbeat(255,1); h:tick(yaw)
    for _=1,2 do
        for i=1,7 do
            h:radio(200,1)
            if i==7 then h:heartbeat(255,1) end
            h:tick(yaw)
        end
    end
    local texts=h:texts('^YWW LINK_ACTIVE c=')
    assert(texts[#texts].message:find('rec=3',1,true))
end

test('healthy ground link alone never starts a simulation',function()
    local h=fresh(); healthy(h)
    for _=1,200 do h:radio(200,1); h:heartbeat(255,1); h:tick(35) end
    assert(starts(h)==0); empty(h)
end)
test('production remains magnetic-silent through idle, RETURN180 completion, and recovery',function()
    local h=fresh()
    h:hold(35,4000) -- spans multiple legacy baseline/timeout intervals
    healthy(h,35); loss(h,'heartbeat',35)
    assert(starts(h)==1)
    completion(h,35,180,2,1)
    h:rotate(215,180); h:hold(395,1300) -- even a full turn cannot log YWMR
    assert(#h:logs('YWMR')==0 and #h:texts('^YWW MagReturn')==0)
    recover(h,395)
    local radio=h:logs('YWHT')
    assert(radio[#radio].values[1]==2)
end)
test('no radio at startup, including USB GCS heartbeats, never triggers',function()
    local h=fresh()
    for _=1,1500 do h:heartbeat(255,0); h:tick(35) end
    assert(starts(h)==0 and count(h,'^YWW RETURN180 SIM')==0); empty(h)
end)
test('radio present without an approved radio-channel GCS never qualifies',function()
    local h=fresh()
    for _=1,100 do h:radio(20,1); h:heartbeat(255,0); h:heartbeat(42,1); h:tick(35) end
    h:hold(35,50); assert(starts(h)==0); empty(h)
end)
test('heartbeat timeout is strictly greater than 3000 ms',function()
    local h=fresh(); healthy(h)
    for _=1,30 do h:radio(200,1); h:tick(35) end
    assert(starts(h)==0)
    h:radio(200,1); h:tick(35); assert(starts(h)==1)
end)
test('status timeout is strictly greater than 3500 ms',function()
    local h=fresh(); healthy(h)
    for _=1,35 do h:heartbeat(255,1); h:tick(35) end
    assert(starts(h)==0)
    h:heartbeat(255,1); h:tick(35); assert(starts(h)==1)
end)
test('USB heartbeats do not mask missing radio-channel heartbeats',function()
    local h=fresh(); healthy(h)
    for _=1,31 do h:radio(200,1); h:heartbeat(255,0); h:tick(35) end
    assert(starts(h)==1)
    completion(h,35,180,2,1)
end)
test('stale RADIO_STATUS triggers despite fresh selected-channel GCS',function()
    local h=pending('status'); completion(h,35,180,2,2)
end)
test('simultaneous heartbeat/status expiry records source 3',function()
    local h=fresh(); healthy(h)
    for _=1,5 do h:heartbeat(255,1); h:tick(35) end
    loss(h,'both',35); assert(starts(h)==1)
    completion(h,35,180,2,3)
end)
test('source remains fixed when a second timeout follows the first',function()
    local h=pending('heartbeat'); h:hold(35,50); completion(h,35,180,2,1)
end)
for _,rssi in ipairs({20,255}) do
    test('RSSI '..rssi..' alone is not a loss/interference trigger',function()
        local h=fresh(); healthy(h)
        for _=1,100 do h:radio(rssi,1); h:heartbeat(255,1); h:tick(35) end
        assert(starts(h)==0); empty(h)
    end)
end
for _,c in ipairs({
    {'positive 180',180,2}, {'negative 180',-180,2},
    {'positive WARN',176,3}, {'negative WARN',-176,3},
    {'positive FAIL',171,4}, {'negative FAIL',-171,4},
    {'2-degree boundary',178,2}, {'5-degree boundary',175,3},
    {'10-degree boundary',170,4}, {'positive overshoot',184,3},
    {'negative overshoot',-189,4}, {'just beyond PASS',177.999,3},
    {'just beyond WARN',174.999,4},
}) do
    test(c[1]..' classified from measurements',function()
        completion(pending('heartbeat'),35,c[2],c[3],1)
    end)
end
test('positive heading wraparound and measured start/target',function()
    completion(pending('heartbeat',311.4),311.4,180,2,1)
end)
test('negative heading wraparound',function()
    completion(pending('heartbeat',35),35,-180,2,1)
end)
test('start=180 calculates target=0',function()
    completion(pending('heartbeat',180),180,180,2,1)
end)
test('start=0 is valid',function()
    completion(pending('heartbeat',0),0,-180,2,1)
end)
test('incomplete 90-degree rotation produces no result',function()
    local h=pending(); h:rotate(35,90); h:hold(125,40); empty(h)
end)
test('partial turn followed by reversal cannot fake 180 degrees',function()
    local h=pending(); h:rotate(35,90); h:rotate(125,-90); h:hold(35,40); empty(h)
end)
test('target coincidence from a 180-degree jump is rejected',function()
    local h=pending(); h:tick(215); h:hold(215,50); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL yaw jump')==1)
end)
test('stationary wrap noise cannot complete',function()
    local h=pending('heartbeat',359.8)
    for i=1,1000 do h:tick(i%2==0 and 359.9 or 0.1) end
    empty(h)
end)
test('settling requires a full two seconds',function()
    local h=pending(); h:rotate(35,180); h:hold(215,20); empty(h)
    h:tick(215); assert(#h:logs('YWRT')==1)
end)
test('settling drift remains bounded across small steps',function()
    local h=pending(); h:rotate(35,170)
    for i=1,45 do h:tick(205+i*0.2) end
    empty(h); h:hold(214,21); assert(#h:logs('YWRT')==1)
end)
test('settling restarts after movement',function()
    local h=pending(); h:rotate(35,180); h:hold(215,15)
    h:tick(216); h:hold(216,20); empty(h)
    h:tick(216); assert(#h:logs('YWRT')==1)
end)
test('191-degree final heading remains outside completion window',function()
    local h=pending(); h:rotate(35,191); h:hold(226,40); empty(h)
end)
test('first fresh restoration cancels before qualified LINK_ACTIVE',function()
    local h=pending(); h:rotate(35,90)
    h:radio(200,1); h:heartbeat(255,1); h:tick(125)
    assert(count(h,'^YWW RETURN180 SIM CANCEL recovered')==1)
    assert(count(h,'^YWW LINK_ACTIVE c=')==1) -- only the original healthy event
    h:rotate(125,90); h:hold(215,30); empty(h)
end)
test('recovery on the completion poll cancels instead of PASS',function()
    local h=pending(); h:rotate(35,180); h:hold(215,20)
    h:radio(200,1); h:heartbeat(255,1); h:tick(215)
    empty(h); assert(count(h,'^YWW RETURN180 SIM CANCEL recovered')==1)
end)
test('USB heartbeat alone cannot cancel a pending experiment',function()
    local h=pending()
    h.before_tick=function(self) self:heartbeat(255,0) end
    completion(h,35,180,2,1)
    assert(count(h,'^YWW RETURN180 SIM CANCEL recovered')==0)
end)
test('a heartbeat burst cannot re-enable another attempt',function()
    local h=pending(); h:radio(200,1)
    for _=1,3 do h:heartbeat(255,1) end
    h:tick(35); loss(h,'heartbeat',35)
    assert(starts(h)==1); empty(h)
end)
test('three spaced recovery observations re-enable a future loss event',function()
    local h=pending(); recover(h,35); loss(h,'heartbeat',35)
    assert(starts(h)==2 and count(h,'^YWW Telemetry LOST %(SIM%)')==2)
    completion(h,35,-180,2,1)
end)
test('completion is never duplicated during a continuing loss',function()
    local h=pending(); local ending=completion(h,35,180,2,1)
    h:hold(ending,2000)
    assert(starts(h)==1 and #h:logs('YWRT')==1)
    assert(count(h,'^YWW RETURN180 SIM TIMEOUT')==0)
end)
test('idle never triggers repeated simulation timeouts',function()
    local h=pending(); h:hold(35,4000); empty(h)
    assert(starts(h)==1 and count(h,'^YWW RETURN180 SIM TIMEOUT')==1)
end)
test('timeout is strictly beyond 120 seconds',function()
    local h=pending(); h:hold(35,1200)
    assert(count(h,'^YWW RETURN180 SIM TIMEOUT')==0)
    h:tick(35); assert(count(h,'^YWW RETURN180 SIM TIMEOUT')==1); empty(h)
end)
test('more than 270 degrees aborts and 540 cannot fake a half turn',function()
    local h=pending(); h:rotate(35,540); h:hold(575,30); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL excessive turn')==1)
end)
for _,c in ipairs({{'missing',nil},{'NaN',0/0},{'positive infinity',math.huge},
                   {'negative infinity',-math.huge}}) do
    test(c[1]..' AHRS at trigger consumes loss without starting',function()
        local h=fresh(); healthy(h); loss(h,'heartbeat',c[2])
        assert(starts(h)==0 and count(h,'^YWW RETURN180 SIM SKIP invalid heading')==1)
        h:hold(35,50); h:rotate(35,180); h:hold(215,30); empty(h)
    end)
    test(c[1]..' AHRS during rotation cancels',function()
        local h=pending(); h:rotate(35,90); h:tick(c[2]); h:rotate(125,90)
        h:hold(215,30); empty(h)
        assert(count(h,'^YWW RETURN180 SIM CANCEL invalid heading')==1)
    end)
end
test('armed at loss skips with no retry after disarming',function()
    local h=fresh(); healthy(h); h.armed=true; loss(h,'heartbeat',35)
    h.armed=false; h:rotate(35,180); h:hold(215,30); empty(h)
    assert(starts(h)==0 and count(h,'^YWW RETURN180 SIM SKIP armed')==1)
end)
test('arming during manual rotation explicitly cancels',function()
    local h=pending(); h:rotate(35,90); h.armed=true; h:tick(125)
    h.armed=false; h:rotate(125,90); h:hold(215,30); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL armed')==1)
end)
test('unknown armed state at loss fails closed',function()
    local h=fresh(); healthy(h); h.arm_missing=true; loss(h,'heartbeat',35)
    assert(starts(h)==0 and count(h,'^YWW RETURN180 SIM SKIP armed')==1); empty(h)
end)
test('unknown armed state during rotation cancels',function()
    local h=pending(); h.arm_missing=true; h:tick(35); h.arm_missing=false
    h:rotate(35,180); h:hold(215,30); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL armed')==1)
end)
for _,c in ipairs({{'long sample gap',35,1001,'sample gap'},
                   {'duplicate timestamp',35,0,'sample gap'},
                   {'positive discontinuity',81,100,'yaw jump'},
                   {'negative discontinuity',-11,100,'yaw jump'}}) do
    test(c[1]..' cancels once',function()
        local h=pending(); h:tick(c[2],c[3]); h:rotate(c[2],180); h:hold(c[2]+180,40)
        empty(h); assert(count(h,'^YWW RETURN180 SIM CANCEL '..c[4])==1)
    end)
end
test('exact 1000-ms gap and 45-degree samples are permitted',function()
    local h=pending(); h:tick(35,1000); h:rotate(35,180,45); h:hold(215,21)
    assert(#h:logs('YWRT')==1)
end)
test('uint32 uptime wrap preserves elapsed sample and settle timing',function()
    local h=pending('heartbeat',35,{clock=4294963000})
    completion(h,35,180,2,1)
end)
test('selected radio channel change cancels instead of mixing histories',function()
    local h=pending(); h:radio(200,2); h:tick(35)
    h:rotate(35,180); h:hold(215,30); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL channel changed')==1)
end)
test('radio update exception cancels and preserves original scheduler delay',function()
    local h=pending(); h.radio_error=true; h:tick(35,1000)
    assert(h.delay==1000 and count(h,'^YWW RETURN180 SIM CANCEL radio error')==1)
    h.radio_error=false; h:rotate(35,180); h:hold(215,30); empty(h)
end)
test('AHRS exception cancels pending simulation and isolates both monitors',function()
    local h=pending(); h.yaw_error=true; h:tick(35)
    h.yaw_error=false; h:rotate(35,180); h:hold(215,30); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL error')==1)
    assert(count(h,'^YWW RETURN180 SIM error:')==1 and h.delay==100)
end)
test('unavailable arming API fails safely',function()
    local h=pending(); h.arm_error=true; h:tick(35); h.arm_error=false
    h:rotate(35,180); h:hold(215,30); empty(h)
    assert(count(h,'^YWW RETURN180 SIM CANCEL error')==1)
end)
for _,after in ipairs({false,true}) do
    test('logger exception '..(after and 'after' or 'before')..' storage is never retried',function()
        local h=pending()
        if after then h.log_after_error='YWRT' else h.log_error='YWRT' end
        h:rotate(35,180); h:hold(215,200)
        assert(h.writes.YWRT==1 and #h:logs('YWRT')==(after and 1 or 0))
        assert(count(h,'^YWW RETURN180 SIM PASS')==1)
        assert(count(h,'^YWW RETURN180 SIM error:')==1)
        recover(h,215); loss(h,'heartbeat',215)
        assert(starts(h)==2) -- the logger error did not disable future episodes
    end)
end
test('result STATUSTEXT failure cannot duplicate the logged completion',function()
    local h=pending(); h:rotate(35,180); h.return180_text_error=true
    h:hold(215,200); assert(h.writes.YWRT==1 and #h:logs('YWRT')==1)
    h.return180_text_error=false; h:hold(215,20); assert(h.writes.YWRT==1)
end)
test('trigger STATUSTEXT failure and its diagnostic cannot terminate scheduler',function()
    local h=fresh(); healthy(h); h.return180_text_error=true
    loss(h,'heartbeat',35); h.return180_text_error=false
    h:rotate(35,180); h:hold(215,30); empty(h); assert(h.delay==100)
    recover(h,215); loss(h,'heartbeat',215); assert(starts(h)==1)
end)
test('new errors have their own 10-second diagnostic throttle',function()
    local h=fresh(); healthy(h); h.yaw_error=true
    loss(h,'heartbeat',35)
    for _=1,3 do recover(h,35); loss(h,'heartbeat',35) end
    local errors=h:texts('^YWW RETURN180 SIM error:')
    assert(#errors>=2)
    for i=2,#errors do assert(errors[i].clock-errors[i-1].clock>=10000) end
    empty(h)
end)
test('YWHT logger failure does not disable YWRT',function()
    local h=fresh(); h.log_error='YWHT'
    h:radio(200,1); h:heartbeat(255,1); h:tick(35)
    assert(count(h,'^YWW LINK_ACTIVE c=')==1 and h.writes.YWHT==1)
    loss(h,'heartbeat',35); assert(starts(h)==1)
    completion(h,35,180,2,1)
    assert(h.writes.YWHT==1)
end)
test('YWMR and YWRT complete independently without modifying schemas',function()
    local h=pending('heartbeat',35,{configuration='integration'})
    completion(h,35,180,2,1)
    h:rotate(215,180); h:hold(395,21)
    local magnetic=h:logs('YWMR')
    assert(#magnetic==1 and #h:logs('YWRT')==1 and #h:logs('YWHT')>=2)
    local v=magnetic[1].values
    near(v[1],35); near(v[2],35); near(v[3],360); near(v[4],0)
    assert(v[5]==2 and v[6]==1)
    recover(h,395)
    local radio=h:logs('YWHT'); assert(radio[#radio].values[1]==2)
end)
test('YWRT logger failure leaves the existing YWMR logger operational',function()
    local h=pending('heartbeat',35,{configuration='integration'})
    h.log_error='YWRT'; h:rotate(35,180); h:hold(215,21)
    h:rotate(215,180); h:hold(395,21)
    assert(h.writes.YWRT==1 and #h:logs('YWRT')==0 and #h:logs('YWMR')==1)
    assert(count(h,'^YWW RETURN180 SIM error:')==1)
    recover(h,395)
    local radio=h:logs('YWHT'); assert(radio[#radio].values[1]==2)
end)
test('YWMR logger failure leaves YWRT and original radio recovery operational',function()
    local h=fresh({configuration='integration'})
    h.before_tick=function(self) self:radio(200,1); self:heartbeat(255,1) end
    healthy(h,35); h:hold(35,21)
    h.log_error='YWMR'; h:rotate(35,360); h:hold(395,21)
    assert(h.writes.YWMR==1 and #h:logs('YWMR')==0)
    assert(count(h,'^YWW MagReturn error:')==1)
    h.before_tick=nil; loss(h,'heartbeat',395)
    assert(starts(h)==1)
    completion(h,395,180,2,1)
    recover(h,575)
    local radio=h:logs('YWHT'); assert(radio[#radio].values[1]==2)
end)
test('new STATUSTEXT is simulation-labelled and <=50 bytes',function()
    local h=pending(); completion(h,35,180,2,1)
    for _,n in ipairs(h.notices) do
        if n.message:find('RETURN180',1,true) or n.message:find('Telemetry LOST',1,true) then
            assert(n.message:find('SIM',1,true) and #n.message<=50)
        end
    end
end)
test('all production API method calls are on an explicit read-only allowlist',function()
    local f=assert(io.open(path)); local source=f:read('*a'); f:close()
    source=source:gsub('%-%-[^\n]*','')
    local allowed={['param:get']=true,['mavlink:init']=true,
        ['mavlink:register_rx_msgid']=true,['mavlink:receive_chan']=true,
        ['gcs:send_text']=true,['logger:write']=true,['arming:is_armed']=true,
        ['ahrs:get_yaw_rad']=true,['msg:byte']=true,['s:sub']=true,['s:gsub']=true}
    for object,method in source:gmatch('([%w_]+):([%w_]+)%s*%(') do
        local key=object..':'..method
        -- String methods on local diagnostics are also read-only.
        assert(allowed[key] or method=='sub' or method=='gsub' or method=='toint',key)
    end
end)
print(string.format('%s: %d RETURN180 scenarios, %d failures; forbidden API calls checked in every scenario.',
    failed==0 and 'PASS' or 'FAIL',checks,failed))
assert(failed==0,'RETURN180 regression failures: '..failed)
