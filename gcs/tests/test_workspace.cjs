const assert = require('node:assert/strict');
const test = require('node:test');
const {harness} = require('./test_gui_wiring.cjs');
const tick = () => new Promise(resolve=>setImmediate(resolve));
function telemetry(h, extra={}) {
  const t={connected:true,armed:false,position_hold_available:true,lat:22.31,lon:91.81,
    home_lat:22.31,home_lon:91.81,custom_mode:3<<16,sats:12,fix:'GPS_FIX_TYPE_3D_FIX',rc_available:true,...extra};
  h.run(`window.workspaceTelemetry(${JSON.stringify(t)})`);
}
function prepare(h) { h.run('addWaypoint({lat:22.32,lng:91.82}); routeLL=[[22.31,91.81,45],[22.32,91.82,45]]'); telemetry(h); }
const accepted = async()=>new Response(JSON.stringify({ok:true,accepted:true}));

test('workspace starts with disabled flight controls and all new handlers attached',async()=>{
  const h=await harness({},true);
  for(const id of ['flight-go','flight-hold','flight-home','emergency-open']) assert.equal(h.elements.get(id).disabled,true,id);
  for(const id of ['view-map','view-aircraft','view-gps','open-mission','close-mission','open-status','reset-layout','maximize-map','maximize-gazebo','flight-go','flight-hold','flight-home','control-auto','control-manual','emergency-send']) assert.equal(typeof h.elements.get(id).onclick,'function',id);
});
test('shared divider keyboard adjusts layout within limits and reset restores it',async()=>{
  const h=await harness({},true); const divider=h.elements.get('split-main');
  for(let i=0;i<20;i++) divider.emit('keydown',{key:'ArrowRight',preventDefault(){}});
  assert.equal(h.elements.get('col-center').style['--split-x'],'80%');
  h.elements.get('reset-layout').click();assert.equal(h.elements.get('col-center').style['--split-x'],'68%');
});
test('maximize and collapse keep controls outside the panel layout',async()=>{
  const h=await harness({},true);
  h.elements.get('maximize-gazebo').click();assert.equal(h.elements.get('col-center').classList.contains('max-gazebo'),true);
  h.elements.get('maximize-map').click();assert.equal(h.elements.get('col-center').classList.contains('max-gazebo'),false);
  h.elements.get('collapse-fpv').click();h.elements.get('collapse-gazebo').click();
  assert.equal(h.elements.get('col-center').classList.contains('collapsed-fpv'),false);
  assert.ok(h.elements.get('flight-overlay'));
});
test('ARM press cannot start mission, and waits for telemetry after ACK',async()=>{
  const h=await harness({'/api/flight':accepted},true);prepare(h);
  await h.elements.get('flight-go').click();
  const actions=h.requests.filter(r=>r.url==='/api/flight').map(r=>JSON.parse(r.options.body).action);
  assert.deepEqual(actions,['arm']);assert.equal(h.elements.get('flight-go').disabled,true);
  assert.equal(h.elements.get('flight-go').textContent,'PENDING');
  telemetry(h,{armed:true});assert.equal(h.elements.get('flight-go').textContent,'GO');
});
test('GO uploads first and sends start with the verified mission token',async()=>{
  const h=await harness({'/api/flight':accepted,'/api/mission/upload':async()=>new Response(JSON.stringify({ok:true,count:4,verified:true,mission_token:'test-token'}))},true);
  prepare(h);telemetry(h,{armed:true});
  await h.elements.get('flight-go').click();
  const req=h.requests.find(r=>r.url==='/api/flight');
  assert.deepEqual(JSON.parse(req.options.body),{action:'start',mission_token:'test-token'});
  assert.ok(h.requests.findIndex(r=>r.url==='/api/mission/upload')<h.requests.indexOf(req));
  telemetry(h,{armed:true,custom_mode:(4<<16)|(4<<24),mission_token:'test-token'});
  assert.equal(h.elements.get('flight-go').textContent,'FLYING');
});
test('failed readback cannot trigger GO',async()=>{
  const h=await harness({'/api/mission/upload':async()=>new Response(JSON.stringify({ok:false,error:'Mission differs'}))},true);
  prepare(h);telemetry(h,{armed:true});await h.elements.get('flight-go').click();
  assert.equal(h.requests.filter(r=>r.url==='/api/flight').length,0);
});
test('RC takeover cancels GO while upload is pending',async()=>{
  let finish;
  const h=await harness({'/api/flight':accepted,'/api/mission/upload':()=>new Promise(resolve=>{finish=resolve;})},true);
  prepare(h);telemetry(h,{armed:true});const go=h.elements.get('flight-go').click();await tick();
  // Priority operation may also be invoked through an accessible control handler.
  await h.elements.get('flight-hold').click();
  finish(new Response(JSON.stringify({ok:true,count:4,verified:true,mission_token:'test-token'})));await go;
  assert.deepEqual(h.requests.filter(r=>r.url==='/api/flight').map(r=>JSON.parse(r.options.body).action),['hold']);
});
test('emergency dialog requires deliberate acknowledgement and does not send on open',async()=>{
  const h=await harness({'/api/flight':accepted},true);telemetry(h,{armed:true});
  h.elements.get('emergency-open').click();
  assert.equal(h.elements.get('emergency-dialog').open,true);
  await h.elements.get('emergency-send').click();
  assert.equal(h.requests.filter(r=>r.url==='/api/flight').length,0);
  h.elements.get('emergency-confirm').checked=true;h.elements.get('emergency-confirm').onchange();
  await h.elements.get('emergency-send').click();
  const payload=JSON.parse(h.requests.find(r=>r.url==='/api/flight').options.body);
  assert.equal(payload.action,'emergency_stop');assert.equal(payload.confirmation,'STOP MOTORS');
});
test('changing return settings invalidates the verified mission and external changes do too',async()=>{
  const h=await harness({},true);prepare(h);
  h.run("window.workspaceMissionUploaded({verified:true,mission_token:'one'},routeRevision)");
  assert.match(h.elements.get('mission-sync').textContent,/verified/);
  h.elements.get('rtl-alt').emit('change');assert.match(h.elements.get('mission-sync').textContent,/upload required/);
  h.run("window.workspaceMissionUploaded({verified:true,mission_token:'two'},routeRevision)");
  telemetry(h,{armed:true,mission_token:null,mission_changed:true});assert.match(h.elements.get('mission-sync').textContent,/upload required/);
});
test('link loss disables control and clears mission verification',async()=>{
  const h=await harness({},true);prepare(h);
  h.run("window.workspaceMissionUploaded({verified:true,mission_token:'one'},routeRevision); window.workspaceTelemetry({connected:false})");
  assert.equal(h.elements.get('flight-go').disabled,true);assert.equal(h.elements.get('emergency-open').disabled,true);
  assert.match(h.elements.get('mission-sync').textContent,/upload required/);
});

test('return settings changed during upload cannot start the obsolete mission',async()=>{
  let finish;
  const h=await harness({'/api/flight':accepted,'/api/mission/upload':()=>new Promise(resolve=>{finish=resolve;})},true);
  prepare(h);telemetry(h,{armed:true});const go=h.elements.get('flight-go').click();await tick();
  h.elements.get('rtl-alt').emit('change');
  finish(new Response(JSON.stringify({ok:true,count:4,verified:true,mission_token:'old-return-settings'})));
  await go;
  assert.equal(h.requests.filter(r=>r.url==='/api/flight').length,0);
  assert.match(h.elements.get('mission-sync').textContent,/upload required/);
});
test('aircraft mission read does not replace the local draft',async()=>{
  const h=await harness({'/api/mission/read':async()=>new Response(JSON.stringify({ok:true,items:[{command:{type:'MAV_CMD_NAV_WAYPOINT'},x:223000000,y:918000000,z:45,param4:null}]}))},true);
  prepare(h); const before=h.run('JSON.stringify(routeLL)');
  await h.elements.get('read-aircraft-mission').click();
  assert.equal(h.run('JSON.stringify(routeLL)'),before);
  assert.match(h.elements.get('aircraft-mission-info').textContent,/1 aircraft items read/);
});

test('large display preference toggles without changing flight state',async()=>{
  const h=await harness({},true);
  h.elements.get('display-size').click();
  assert.equal(h.elements.get('display-size')['aria-pressed'],'true');
  assert.equal(h.elements.get('display-size').textContent,'Normal text');
  assert.equal(h.elements.get('flight-go').disabled,true);
  h.elements.get('display-size').click();
  assert.equal(h.elements.get('display-size')['aria-pressed'],'false');
});


test('camera placeholders handle stopped, streaming, failure, and restart states', async () => {
  const h = await harness({}, true);
  for (const [button, image] of [['btn-feed','feed-img'], ['btn-feed2','feed2-img']]) {
    const btn=h.elements.get(button), img=h.elements.get(image);
    const empty=h.elements.get(image+'-empty'), state=h.elements.get(image+'-state');
    assert.equal(state.textContent,'Stream stopped');
    btn.click();
    assert.equal(state.textContent,'Connecting to camera…');
    assert.equal(img.style.visibility,'visible');
    img.emit('load'); assert.equal(empty.hidden,true);
    img.emit('error'); assert.equal(empty.hidden,false);
    assert.equal(state.textContent,'Stream unavailable');
    btn.click(); assert.equal(state.textContent,'Stream stopped');
    assert.equal(img.src,undefined);
    img.emit('load'); assert.equal(empty.hidden,false);
    btn.click(); img.emit('load'); assert.equal(empty.hidden,true);
  }
});
test('GPS instrument updates from telemetry and clears when disconnected', async () => {
  const h=await harness({},true);
  telemetry(h,{sats:10}); assert.equal(h.elements.get('header-gps').textContent,'10 · 3D fix');
  telemetry(h,{sats:14}); assert.equal(h.elements.get('header-gps').textContent,'14 · 3D fix');
  telemetry(h,{connected:false}); assert.equal(h.elements.get('header-gps').textContent,'GPS unavailable');
});

test('readiness does not equate connection or arming with flight readiness', async () => {
  const h=await harness({},true), label=h.elements.get('header-readiness');
  assert.equal(label.textContent,'Not ready to fly');
  assert.equal(h.elements.get('header-readiness-link').textContent,'Disconnected');
  telemetry(h); assert.equal(label.textContent,'Not ready to fly');
  prepare(h); assert.equal(label.textContent,'Ready to arm');
  telemetry(h,{armed:true}); assert.equal(label.textContent,'Verify mission');
  telemetry(h,{armed:true,position_hold_available:false}); assert.equal(label.textContent,'Not ready to fly');
  telemetry(h,{connected:false}); assert.equal(label.textContent,'Not ready to fly');
  assert.equal(h.elements.get('header-readiness-card').dataset.state,'offline');
});

test('floating map panels minimize independently and reset restores both', async () => {
  const h=await harness({},true);
  const map=h.elements.get('map-toolbar'), flight=h.elements.get('flight-overlay');
  const toggle=h.elements.get('minimize-map-tools');
  toggle.click(); assert.equal(map.classList.contains('overlay-minimized'),true);
  assert.equal(toggle['aria-expanded'],'false');
  assert.equal(flight.classList.contains('overlay-minimized'),false);
  toggle.click(); assert.equal(map.classList.contains('overlay-minimized'),false);
  assert.equal(toggle['aria-expanded'],'true');
  toggle.click(); h.elements.get('minimize-flight-controls').click();
  h.elements.get('reset-layout').click();
  for (const panel of [map,flight]) assert.equal(panel.classList.contains('overlay-minimized'),false);
  assert.equal(h.elements.get('minimize-flight-controls')['aria-expanded'],'true');
});
