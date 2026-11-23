-- Execute the actual deployable script with bounded, read-only API doubles.
-- Usage: lua5.3 tests/test_radio_magnetic_integration.lua lua/radio_health_monitor.lua
local path = arg[1] or 'lua/radio_health_monitor.lua'
local Mock = dofile('tests/support/ardupilot_mock.lua')
local Trace = dofile('tests/support/radio_trace.lua')
local golden = dofile('tests/fixtures/yww_v5_2_radio_trace.lua')
local checks = 0
local function test(name, fn)
    fn()
    checks = checks + 1
    print('PASS: '..name)
end
local function near(a,b) assert(math.abs(a-b)<0.00001, a..' differs from '..b) end
local function empty(h) assert(#h:logs('YWMR')==0, 'unexpected heading-return result') end
local function fresh(options) return Mock.new(path, options) end
local function ready(h, start)
    start = start or 311
    h:hold(start, 21)
    assert(#h:texts('^YWW MagReturn ready:')==1, 'baseline not ready after 2 s')
end
local function result(h, turn, status, dir, start)
    start = start or 311
    ready(h,start)
    local ending = h:rotate(start,turn)
    h:hold(ending,21)
    local records = h:logs('YWMR')
    assert(#records==1, 'expected one YWMR, got '..#records)
    local v = records[1].values
    near(v[1],start%360); near(v[2],ending%360); near(v[3],turn)
    near(v[4],(turn+180)%360-180)
    assert(v[5]==status and v[6]==dir, 'incorrect classification/direction')
    local texts = h:texts('^YWW MagReturn [PWF]')
    assert(#texts==1 and #texts[1].message<=50)
    assert(texts[1].severity==(status==2 and 6 or 4))
    return ending
end

for _, case in ipairs({
    {'positive full turn',360,2,1}, {'negative full turn',-360,2,2},
    {'WARN negative error',356,3,1}, {'FAIL negative error',351,4,1},
    {'WARN positive error',-356,3,2}, {'FAIL positive error',-351,4,2},
    {'PASS boundary 2 degrees',358,2,1}, {'WARN boundary 5 degrees',355,3,1},
    {'FAIL boundary 10 degrees',350,4,1}, {'PASS just inside',358.001,2,1},
    {'WARN just outside PASS',357.999,3,1}, {'FAIL just outside WARN',354.999,4,1},
}) do
    test(case[1]..' without radio/GCS',function()
        local h=fresh(); result(h,case[2],case[3],case[4])
        for _, r in ipairs(h:logs('YWHT')) do assert(r.values[1]==0) end
    end)
end
test('baseline comes from current AHRS heading',function()
    result(fresh(),360,2,1,47.25)
end)
test('baseline requires 2 seconds and sends one notice',function()
    local h=fresh(); h:hold(73,20)
    assert(#h:texts('^YWW MagReturn ready:')==0)
    h:hold(73,1); assert(#h:texts('^YWW MagReturn ready:')==1)
    h:hold(73,100); assert(#h:texts('^YWW MagReturn ready:')==1); empty(h)
end)
test('incomplete rotation cannot complete',function()
    local h=fresh(); ready(h); h:rotate(311,90); h:hold(401,40); empty(h)
end)
test('returning after a partial turn cannot complete',function()
    local h=fresh(); ready(h); h:rotate(311,180); h:rotate(491,-180)
    h:hold(311,40); empty(h)
end)
test('stationary wraparound noise cannot complete',function()
    local h=fresh(); ready(h,359.7)
    for i=1,1200 do h:tick(i%2==0 and 359.9 or 0.1) end
    empty(h)
end)
test('slow drifting baseline is not stationary',function()
    local h=fresh()
    for i=1,100 do h:tick(i*0.2) end
    assert(#h:texts('^YWW MagReturn ready:')==0); empty(h)
end)
test('settling needs 2 seconds at return',function()
    local h=fresh(); ready(h); h:rotate(311,360); h:hold(671,20); empty(h)
    h:hold(671,1); assert(#h:logs('YWMR')==1)
end)
test('small steps with unbounded settling drift cannot complete',function()
    local h=fresh(); ready(h); h:rotate(311,352)
    for i=1,45 do h:tick(663+i*0.2) end
    empty(h)
    h:hold(672,21); assert(#h:logs('YWMR')==1)
end)
test('settling motion restarts the 2-second interval',function()
    local h=fresh(); ready(h); h:rotate(311,360); h:hold(671,15)
    h:tick(672); h:hold(672,20); empty(h)
    h:tick(672); assert(#h:logs('YWMR')==1)
end)
test('final error outside 10 degrees cannot complete',function()
    local h=fresh(); ready(h); h:rotate(311,371); h:hold(682,30); empty(h)
end)
test('arming aborts and disarming needs a fresh baseline',function()
    local h=fresh(); ready(h); h:rotate(311,180)
    h.armed=true; h:tick(491); h.armed=false
    h:rotate(491,180); h:hold(671,30); empty(h)
    h:rotate(671,360); h:hold(1031,21); assert(#h:logs('YWMR')==1)
end)
for _, case in ipairs({{'nil',nil}, {'NaN',0/0}, {'positive infinity',math.huge},
                        {'negative infinity',-math.huge}}) do
    test(case[1]..' heading aborts attempt',function()
        local h=fresh(); ready(h); h:rotate(311,180); h:tick(case[2])
        h:rotate(491,180); h:hold(671,30); empty(h)
    end)
end
for _, case in ipairs({{'gap over 1 second',311,1001},
                        {'positive discontinuity',357,100},
                        {'negative discontinuity',265,100}}) do
    test(case[1]..' resets baseline',function()
        local h=fresh(); ready(h); h:tick(case[2],case[3])
        h:rotate(case[2],360); h:hold(case[2]+360,30); empty(h)
    end)
end
test('exact 1-second gap and 45-degree steps are allowed',function()
    local h=fresh(); ready(h); h:tick(311,1000)
    h:rotate(311,360,45); h:hold(671,21); assert(#h:logs('YWMR')==1)
end)
test('duration beyond 120 seconds aborts',function()
    local h=fresh(); ready(h); h:hold(311,1201)
    assert(#h:texts('^YWW MagReturn aborted:')==1)
    h:rotate(311,360); h:hold(671,30); empty(h)
end)
test('more than 540 degrees aborts',function()
    local h=fresh(); ready(h); h:rotate(311,545)
    assert(#h:texts('^YWW MagReturn aborted:')==1); empty(h)
    h:rotate(856,175); h:hold(1031,30); empty(h)
end)
test('cooldown prevents duplicates and permits a fresh second attempt',function()
    local h=fresh(); local ending=result(h,360,2,1)
    h:hold(ending,49); assert(#h:texts('^YWW MagReturn ready:')==1)
    h:hold(ending,21); assert(#h:texts('^YWW MagReturn ready:')==2)
    h:rotate(ending,-360); h:hold(ending-360,21)
    assert(#h:logs('YWMR')==2)
end)
test('arming during cooldown cannot bypass it',function()
    local h=fresh(); local ending=result(h,360,2,1)
    h.armed=true; h:tick(ending); h.armed=false; h:hold(ending,30)
    assert(#h:texts('^YWW MagReturn ready:')==1)
    assert(#h:logs('YWMR')==1)
end)
test('AHRS exceptions are throttled and radio continues',function()
    local h=fresh(); h.yaw_error=true
    for i=1,251 do
        if i%10==0 then h:radio(200); h:heartbeat() end
        h:tick(311)
    end
    local errors=h:texts('^YWW MagReturn error:')
    assert(#errors==3)
    for i=2,#errors do assert(errors[i].clock-errors[i-1].clock==10000) end
    assert(h:logs('YWHT')[2].values[1]==2); empty(h)
    h.yaw_error=false; result(h,360,2,1)
end)
test('magnetic error diagnostics can throw without losing scheduler',function()
    local h=fresh(); h.yaw_error=true; h.text_error=true
    h:radio(200); h:heartbeat(); h:hold(311,120)
    assert(h:logs('YWHT')[1].values[1]==2)
end)
test('AHRS exception during rotation discards the partial turn',function()
    local h=fresh(); ready(h); h:rotate(311,180)
    h.yaw_error=true; h:tick(491); h.yaw_error=false
    h:rotate(491,180); h:hold(671,30); empty(h)
end)
for _, after in ipairs({false,true}) do
    test('magnetic logger exception '..(after and 'after' or 'before')..' write is never retried',function()
        local h=fresh(); ready(h)
        if after then h.log_after_error='YWMR' else h.log_error='YWMR' end
        h:rotate(311,360); h:hold(671,120)
        assert(h.writes.YWMR==1)
        assert(#h:logs('YWMR')==(after and 1 or 0))
        assert(#h:texts('^YWW MagReturn error:')==1)
        assert(#h:logs('YWHT')>=1)
    end)
end
test('magnetic STATUSTEXT exception consumes attempt before retry',function()
    local h=fresh(); ready(h); h.text_error=true
    h:rotate(311,360); h:hold(671,120)
    assert(h.writes.YWMR==1 and #h:logs('YWMR')==1)
end)
test('radio logger failure does not disable magnetic logging',function()
    local h=fresh(); h.log_error='YWHT'; result(h,360,2,1)
    assert(h.writes.YWHT==1)
end)
test('radio exception retains original 1000-ms return and magnetic independence',function()
    local h=fresh(); h.radio_error=true; result(h,360,2,1)
    assert(#h:texts('^YWW V5.2 error:')>=1)
    h.radio_error=false; h:radio(200); h:heartbeat(); h:tick(671)
    assert(h.delay==100 and #h:logs('YWHT')==1)
end)
test('uint32 rollover resets magnetic baseline safely',function()
    local h=fresh({clock=4294964000}); ready(h)
    h:rotate(311,180); h:hold(491,30); empty(h)
    h:rotate(491,360); h:hold(851,21); assert(#h:logs('YWMR')==1)
end)
test('cooldown recovers after uint32 rollover',function()
    local h=fresh({clock=4294954000}); result(h,360,2,1)
    h:hold(671,100)
    assert(#h:texts('^YWW MagReturn ready:')==2)
end)
for _, mode in ipairs({'normal','armed','exception','rotating'}) do
    test('original full radio trace preserved with magnetic '..mode,function()
        local h=fresh()
        if mode=='armed' then
            h.before_tick=function(self) self.armed=self.clock%7000<3000 end
        elseif mode=='exception' then h.yaw_error=true
        elseif mode=='rotating' then
            h.before_tick=function(self)
                local phase=self.clock%15000
                self.yaw=311+math.min(360,math.max(0,(phase-3000)/20))
            end
        end
        Trace.run(h)
        local actual=Trace.capture(h)
        assert(#actual==#golden)
        for i,line in ipairs(golden) do assert(actual[i]==line,'radio trace changed at '..i) end
        if mode=='rotating' then assert(#h:logs('YWMR')>0) end
    end)
end
test('sysid range, MAV_TYPE_GCS, and channel filters remain enforced',function()
    local h=fresh({params={MAV_GCS_SYSID=250,MAV_GCS_SYSID_HI=252}})
    h:radio(200,1); h:heartbeat(249,1); h:heartbeat(251,2); h:heartbeat(251,1,2)
    h:tick(311); assert(h:logs('YWHT')[1].values[1]==1)
    h:heartbeat(251,1); h:tick(311); assert(h:logs('YWHT')[2].values[1]==2)
    h:radio(200,3); h:tick(311); assert(h:logs('YWHT')[3].values[1]==1)
end)
test('legacy SYSID_MYGCS fallback is retained',function()
    local h=fresh({params={SYSID_MYGCS=42}})
    h:radio(200); h:heartbeat(42); h:tick(311)
    assert(h:logs('YWHT')[1].values[1]==2)
end)
test('radio heartbeat timeout boundary is unchanged',function()
    local h=fresh(); h:radio(200); h:heartbeat(); h:tick(311)
    for _=1,30 do h:radio(200); h:tick(311) end
    assert(#h:texts('^YWW GCS_SILENT c=')==0)
    h:radio(200); h:tick(311); assert(#h:texts('^YWW GCS_SILENT c=')==1)
end)
test('radio status timeout boundary is unchanged',function()
    local h=fresh(); h:radio(200); h:heartbeat(); h:tick(311)
    h:hold(311,35); assert(#h:texts('^YWW STATUS_STALE c=')==0)
    h:tick(311); assert(#h:texts('^YWW STATUS_STALE c=')==1)
end)
test('RSSI 90 cannot establish a low-signal fault',function()
    local h=fresh(); h:radio(90); h:heartbeat(); h:tick(311)
    for _=1,80 do h:radio(90); h:tick(311) end
    assert(#h:texts('^YWW RF_LOSS_SUSPECTED c=')==0)
end)
test('low RSSI must persist for the full 3 seconds',function()
    local h=fresh(); h:radio(89); h:heartbeat(); h:tick(311)
    for _=1,60 do h:radio(89); h:tick(311) end
    assert(#h:texts('^YWW RF_LOSS_SUSPECTED c=')==0)
    h:radio(89); h:tick(311)
    assert(#h:texts('^YWW RF_LOSS_SUSPECTED c=')==1)
end)
test('recovery counts observations at exactly 700 ms apart',function()
    local h=fresh(); h:radio(89); h:heartbeat(); h:tick(311)
    for _=1,61 do h:radio(89); h:tick(311) end
    h:radio(200); h:heartbeat(); h:tick(311)
    h:hold(311,5); h:heartbeat(); h:tick(311) -- only 600 ms: ignored
    assert(#h:texts('^YWW LINK_ACTIVE c=')==1)
    h:heartbeat(); h:tick(311) -- 700 ms: second observation
    assert(#h:texts('^YWW LINK_ACTIVE c=')==1)
    h:hold(311,6); h:heartbeat(); h:tick(311) -- 700 ms: third
    local active=h:texts('^YWW LINK_ACTIVE c=')
    assert(#active==2 and active[2].message:find('rec=3 lat=0 faults=1',1,true))
end)
test('embedded development fragment stays in sync',function()
    local f=assert(io.open(path)); local source=f:read('*a'); f:close()
    f=assert(io.open('lua/magnetic_return_embed.lua')); local fragment=f:read('*a'); f:close()
    local embedded=assert(source:match('%-%- BEGIN embedded magnetic_return_embed.lua\n(.-)%-%- END embedded magnetic_return_embed.lua'))
    assert(embedded==fragment,'development fragment differs from integrated block')
end)
print(string.format('PASS: %d integration scenarios; golden replay checks 13 YWHT records and 21 radio notices per run.',checks))
