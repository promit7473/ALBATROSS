// Run the complete app against a minimal DOM/map and mocked API. No aircraft I/O.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');

function element() {
  const classes = new Set(), children = new Map(), listeners = new Map();
  return { style: {setProperty(k,v){this[k]=v;}}, dataset: {}, value: '45', checked: true, disabled: false,
    textContent: '', innerHTML: '',
    classList: { add: v => classes.add(v), remove: (...values) => values.forEach(v=>classes.delete(v)), contains: v => classes.has(v),
      toggle(v, on = !classes.has(v)) { on ? classes.add(v) : classes.delete(v); return on; } },
    addEventListener(name,fn) {if(!listeners.has(name)) listeners.set(name,[]);listeners.get(name).push(fn);},
    emit(name,event={}) {for(const fn of listeners.get(name)||[]) fn(event);},
    setAttribute(k,v) {this[k]=v;}, removeAttribute(k) {delete this[k];}, appendChild() {}, focus(){},
    showModal(){this.open=true;}, close(){this.open=false;},
    setPointerCapture(){}, getBoundingClientRect(){return {left:0,top:0,width:1000,height:800};},
    click() { return this.onclick?.(); },
    querySelector(s) { if (!children.has(s)) children.set(s, element()); return children.get(s); },
  };
}
function layer(points = []) {
  const events = new Map();
  return { on(name,fn) {if(!events.has(name)) events.set(name,[]);events.get(name).push(fn);return this;}, async fire(name,e){for(const fn of events.get(name)||[]) await fn(e);}, addTo() { return this; }, bindTooltip() { return this; },
    setView() { return this; }, setIcon() {}, remove() {}, invalidateSize() {},
    getCenter: () => ({lat: 22.31, lng: 91.81}),
    getLatLngs: () => [points.map(p => ({lat: p[0], lng: p[1]}))],
    getBounds() { return {pad() {return this;}, contains() {return true;}}; },
    fitBounds() {}, pm: {setGlobalOptions() {}, setPathOptions() {}},
  };
}
async function harness(overrides = {}, workspace = false) {
  const elements = new Map([...html.matchAll(/id="([^"]+)"/g)].map(m => [m[1], element()]));
  const rows = [], requests = [];
  const fixtures = {'/api/mode': {mode:'sitl'}, '/api/kml': {features:[]},
    '/api/operating_area': {polygon:null}, '/api/caab': {max_alt_ft:200,max_alt_m:60.96},
    '/api/telemetry': {connected:false}, '/api/plan_multi': {ok:true,route:[[22.31,91.81,45],[22.32,91.82,45]]}};
  const context = vm.createContext({console, URLSearchParams, AbortController, Response,
    location: {search:''}, navigator:{}, localStorage:{getItem(){return null;},setItem(){}},
    setTimeout() {}, clearTimeout() {}, requestAnimationFrame() {}, confirm:()=>true,
    window:{addEventListener() {}},
    document:{getElementById: id => elements.get(id) || null, body:element(), addEventListener(){},
      createElement(){const row=element(); rows.push(row); return row;}},
    L:{map:layer,tileLayer:layer,polygon:layer,marker:layer,circleMarker:layer,geoJSON:layer,
      polyline:layer,divIcon:()=>({}),DomEvent:{stopPropagation(){}}},
    fetch: async (url, options={}) => {
      requests.push({url,options});
      if (overrides[url]) return overrides[url](options);
      if (url === '/api/operating_area' && options.method === 'POST') return new Response(JSON.stringify({ok:true}));
      return new Response(JSON.stringify(fixtures[url] || {ok:true}));
    },
  });
  vm.runInContext(source, context);
  if (workspace) vm.runInContext(fs.readFileSync(path.join(__dirname, "../web/workspace.js"), "utf8"), context);
  await new Promise(resolve => setImmediate(resolve));
  return {elements,rows,requests,run: s=>vm.runInContext(s, context)};
}

test('full UI initializes with matching DOM IDs and offline telemetry', async () => {
  const h=await harness();
  assert.equal(h.elements.get('conn').textContent,'NO LINK');
  for (const [id,el] of h.elements) if(id.startsWith('btn-')) assert.equal(typeof el.onclick,'function',id);
});
test('waypoint altitude editor clamps both stored and displayed height',async()=>{
  const h=await harness(); h.run('addWaypoint({lat:22.32,lng:91.82})');
  const input=h.rows.at(-1).querySelector('.wp-alt'); input.value=200; input.onchange({target:input});
  assert.equal(h.run('waypoints[0].alt'),60); assert.equal(input.value,60);
});
test('boundary edits invalidate route and block planning until saved',async()=>{
  const h=await harness(); h.run('addWaypoint({lat:22.32,lng:91.82})');
  await h.elements.get('btn-plan').click(); assert.ok(h.run('routeLL'));
  h.run('boundaryChanged()'); assert.equal(h.run('routeLL'),null);
  const count=h.requests.length; await h.elements.get('btn-plan').click();
  assert.equal(h.requests.length,count); assert.match(h.elements.get('mission-info').textContent,/Save/);
  await h.elements.get('btn-save').click(); assert.equal(h.run('boundaryDirty'),false);
});
test('late planner response cannot restore route after waypoint edits',async()=>{
  let finish;
  const h=await harness({'/api/plan_multi':()=>new Promise(resolve=>{finish=resolve;})});
  h.run('addWaypoint({lat:22.32,lng:91.82})'); const pending=h.elements.get('btn-plan').click();
  await new Promise(resolve=>setImmediate(resolve));
  h.run('addWaypoint({lat:22.33,lng:91.83})');
  finish(new Response(JSON.stringify({ok:true,route:[[22.31,91.81,45],[22.32,91.82,45]]})));
  await pending; assert.equal(h.run('routeLL'),null); assert.equal(h.elements.get('btn-plan').disabled,false);
});
test('failed replan clears the previously uploadable route',async()=>{
  const h=await harness({'/api/plan_multi':async()=>new Response(JSON.stringify({ok:false,error:'No path'}))});
  h.run('addWaypoint({lat:22.32,lng:91.82}); routeLL=[[22.31,91.81],[22.32,91.82]]');
  await h.elements.get('btn-plan').click(); assert.equal(h.run('routeLL'),null);
});
test('stop simulator reports rejected HTTP request without claiming success',async()=>{
  const h=await harness({'/api/sitl/stop':async()=>new Response(JSON.stringify({ok:false,error:'Wrong mode'}),{status:409})});
  await h.elements.get('btn-sitl-stop').click(); assert.match(h.elements.get('mission-info').textContent,/Stop error: Wrong mode/);
});
test('3D dependency failure restores the usable 2D view',async()=>{
  const h=await harness(); h.elements.get('btn-3d').click();
  assert.equal(h.elements.get('cesiumContainer').classList.contains('on'),false);
  assert.match(h.elements.get('mission-info').textContent,/3D library unavailable/);
});
test('clearing a route removes its 3D entity too',async()=>{
  const h=await harness();
  h.run('viewer={entities:{remove(entity){ this.removed=entity; }}}; cesRoute={id:123}; clearRoute()');
  assert.equal(h.run('viewer.entities.removed.id'),123); assert.equal(h.run('cesRoute'),null);
});

test('plan and upload send the selected waypoint heights and return settings',async()=>{
  const h=await harness({'/api/mission/upload':async()=>new Response(JSON.stringify({ok:true,count:7}))});
  h.run('addWaypoint({lat:22.32,lng:91.82})');
  await h.elements.get('btn-plan').click();
  await h.elements.get('btn-upload').click();
  const payload=JSON.parse(h.requests.find(r=>r.url==='/api/mission/upload').options.body);
  assert.deepEqual(payload.route,[[22.31,91.81,45],[22.32,91.82,45]]);
  assert.equal(payload.rtl,true); assert.equal(payload.rtl_alt,45);
  assert.equal(h.elements.get('btn-upload').disabled,false);
  assert.match(h.elements.get('mission-info').innerHTML,/Mission uploaded/);
});
test('flight buttons send their intended API command',async()=>{
  const h=await harness();
  const expected={'btn-arm':'arm','btn-fly':'start_mission','btn-rtl':'rtl',
    'btn-toggle-mode':'toggle_mode','btn-stop-mission':'stop_mission','btn-drop':'drop',
    'btn-proceed':'continue_mission','btn-goto':'reposition','btn-alt-hold':'reposition'};
  h.run('addWaypoint({lat:22.32,lng:91.82}); currentDroneLL=[22.31,91.81]');
  for (const [button,command] of Object.entries(expected)) {
    await h.elements.get(button).click();
    await new Promise(resolve=>setImmediate(resolve));
    const request=h.requests.filter(r=>r.url==='/api/command').at(-1);
    assert.equal(JSON.parse(request.options.body).cmd,command,button);
  }
});

module.exports = {harness};
