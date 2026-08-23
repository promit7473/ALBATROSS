/* Maritime GCS — live map, draw/edit boundary, plan+upload mission, 2D+3D */
const $ = id => document.getElementById(id);
let BASE = { lat: 22.318725, lon: 91.813156 };

// ================= LIVE DRONE STATE (updated every tick) =================
let currentDroneLL = null;   // [lat, lon] from telemetry
let droneIsArmed = false;
let droneRelAlt = 0;

// ================= 2D MAP =================
const map = L.map('map', { zoomControl: true, attributionControl: false }).setView([22.31, 91.81], 12);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19 }).addTo(map);

function setActive(id, on) { const b = $(id); if (b) b.classList.toggle('active', on); }
function haversineKm(a, b) {
  const R = 6371, d2r = Math.PI / 180, dLat = (b[0]-a[0])*d2r, dLon = (b[1]-a[1])*d2r;
  const s = Math.sin(dLat/2)**2 + Math.cos(a[0]*d2r)*Math.cos(b[0]*d2r)*Math.sin(dLon/2)**2;
  return 2*R*Math.asin(Math.sqrt(s));
}
function ringLatLngs(layer) { return (layer.getLatLngs()[0] || []).map(p => [p.lat, p.lng]); }

// -------- operating boundary (editable) --------
let opLayer = null, kmlOperating = null;
const OP_STYLE = { color:'#34d399', weight:2.5, fill:true, fillColor:'#34d399', fillOpacity:0.07, dashArray:'6 4' };
function updateAreaInfo() {
  if (!opLayer) { $('area-info').textContent = 'no boundary set'; return; }
  let maxKm = 0; ringLatLngs(opLayer).forEach(p => maxKm = Math.max(maxKm, haversineKm([BASE.lat,BASE.lon], p)));
  $('area-info').innerHTML = `verts <b>${ringLatLngs(opLayer).length}</b> · max reach <b>${maxKm.toFixed(1)} km</b>`;
}
function makeOpLayer(latlngs) {
  if (opLayer) opLayer.remove();
  opLayer = L.polygon(latlngs, OP_STYLE).addTo(map);
  opLayer.on('pm:edit pm:dragend pm:markerdragend', updateAreaInfo);
  opLayer.on('click', e => { if (addMode) { addWaypoint(e.latlng); L.DomEvent.stopPropagation(e); } });
  updateAreaInfo(); return opLayer;
}

fetch('/api/kml').then(r => r.json()).then(g => {
  if (g.base) {
    BASE = g.base;
    L.circleMarker([g.base.lat, g.base.lon], { radius:6, color:'#0ea5e9', fillColor:'#38bdf8', fillOpacity:1, weight:2 })
      .addTo(map).bindTooltip(g.base.name);
  }
  g.features.forEach(f => {
    if (f.properties.kind === 'exclusion') {
      const ex = L.geoJSON(f, { style:{ color:'#ef4444', weight:1, fillColor:'#ef4444', fillOpacity:.28 } })
        .addTo(map).bindTooltip(f.properties.name || 'exclusion', { sticky:true });
      ex.on('click', e => { if (addMode) { addWaypoint(e.latlng); L.DomEvent.stopPropagation(e); } });
    } else if (f.properties.kind === 'operating') {
      kmlOperating = f.geometry.coordinates[0].map(c => [c[1], c[0]]);
    }
  });
  fetch('/api/operating_area').then(r => r.json()).then(o => {
    const ring = (o && o.polygon && o.polygon.length >= 3) ? o.polygon : kmlOperating;
    if (ring) { makeOpLayer(ring); try { map.fitBounds(opLayer.getBounds().pad(0.2)); } catch(e){} }
  });
});

map.pm.setGlobalOptions({ snappable: true, allowSelfIntersection: false });
map.pm.setPathOptions(OP_STYLE);
$('btn-draw').onclick = () => { if (opLayer && opLayer.pm) opLayer.pm.disable(); setActive('btn-edit', false);
  map.pm.enableDraw('Polygon', { finishOn:'dblclick' }); setActive('btn-draw', true); };
map.on('pm:create', e => { map.pm.disableDraw(); setActive('btn-draw', false);
  const ll = ringLatLngs(e.layer); e.layer.remove(); makeOpLayer(ll); });
$('btn-edit').onclick = () => { if (!opLayer) return;
  const on = !(opLayer.pm && opLayer.pm.enabled());
  if (on) opLayer.pm.enable({ draggable:true, snappable:true }); else opLayer.pm.disable();
  setActive('btn-edit', on); };
$('btn-clear').onclick = () => { if (opLayer) { opLayer.remove(); opLayer = null; } setActive('btn-edit', false); updateAreaInfo(); };
$('btn-save').onclick = () => {
  const poly = opLayer ? ringLatLngs(opLayer) : null;
  fetch('/api/operating_area', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({polygon: poly}) })
    .then(r => r.json()).then(res => { const b = $('btn-save'), old = b.textContent;
      b.textContent = res.ok ? `✓ saved (${res.points})` : '✗ error'; setTimeout(() => b.textContent = old, 1600); });
};

// -------- multi-waypoint mission (each waypoint has its own height) --------
let waypoints = [];        // [{ ll:[lat,lon], alt, marker }]
let routeLL = null, routeLine = null, addMode = false;

function wpAlt() { return Math.max(5, Math.min(200, Math.round(+$('alt').value || 45))); }
function numberIcon(n) {
  return L.divIcon({ className:'', iconSize:[26,26], iconAnchor:[13,13],
    html:`<div style="width:24px;height:24px;border-radius:50%;background:#38bdf8;color:#04263b;
      font:700 12px sans-serif;display:flex;align-items:center;justify-content:center;
      border:2px solid #e0f2ff;box-shadow:0 1px 4px rgba(0,0,0,.6)">${n}</div>` });
}
function renumber() { waypoints.forEach((w,i) => w.marker.setIcon(numberIcon(i+1))); }
function clearRoute() { if (routeLine) { routeLine.remove(); routeLine = null; } routeLL = null; }
function refreshWpList() {
  const box = $('wp-list');
  if (!waypoints.length) { box.innerHTML = '<div class="wp-empty">no waypoints — click ➕ then click the map</div>'; return; }
  box.innerHTML = '';
  waypoints.forEach((w, i) => {
    const row = document.createElement('div'); row.className = 'wp-row';
    row.innerHTML = `<span class="wp-n">${i+1}</span>
      <input class="wp-alt" type="number" min="5" max="200" value="${w.alt}" /> m
      <span class="wp-del" title="remove">✕</span>`;
    row.querySelector('.wp-alt').onchange = e => { w.alt = Math.max(5, Math.min(200, +e.target.value || w.alt)); clearRoute(); };
    row.querySelector('.wp-del').onclick = () => { w.marker.remove(); waypoints.splice(i,1); renumber(); refreshWpList(); clearRoute(); };
    box.appendChild(row);
  });
}
function addWaypoint(latlng) {
  const ll = [latlng.lat, latlng.lng];
  const m = L.marker(ll, { icon: numberIcon(waypoints.length+1), zIndexOffset: 800 }).addTo(map);
  m.on('click', ev => { if (addMode) { addWaypoint(ev.latlng); L.DomEvent.stopPropagation(ev); } });
  waypoints.push({ ll, alt: wpAlt(), marker: m });
  refreshWpList(); clearRoute();
  $('mission-info').innerHTML = `➕ WP <b>${waypoints.length}</b> @ ${wpAlt()} m — add more or <b>Plan route</b>`;
}
function setAddMode(on) { addMode = (on !== undefined) ? on : !addMode;
  setActive('btn-add-wp', addMode); $('map').style.cursor = addMode ? 'crosshair' : ''; }
$('btn-add-wp').onclick = () => setAddMode();
$('btn-clear-wp').onclick = () => { waypoints.forEach(w => w.marker.remove()); waypoints = [];
  clearRoute(); refreshWpList(); setAddMode(false); $('mission-info').textContent = 'waypoints cleared'; };
// clicks on the map (and overlay polygons) add a waypoint while in Add mode
map.on('click', e => { if (addMode) addWaypoint(e.latlng); });
refreshWpList();

$('btn-sitl-start').onclick = async () => {
  $('mission-info').innerHTML = '⏳ <b>Starting PX4 SITL + Gazebo...</b> (takes ~15s)';
  try {
    const res = await (await fetch('/api/sitl/start', { method: 'POST' })).json();
    if (res.ok) {
      $('mission-info').innerHTML = '🚀 <b>SITL Booting...</b> Telemetry link will turn green shortly!';
    } else {
      $('mission-info').textContent = 'Launch failed: ' + res.error;
    }
  } catch(e) {
    $('mission-info').textContent = 'Error: ' + e.message;
  }
};
$('btn-sitl-stop').onclick = async () => {
  $('mission-info').textContent = 'Stopping SITL simulator...';
  try {
    await fetch('/api/sitl/stop', { method: 'POST' });
    $('mission-info').textContent = '⏹ SITL simulator stopped';
  } catch(e) {
    $('mission-info').textContent = 'Stop error: ' + e.message;
  }
};

// -------- PLAN: exclusion-aware route through all waypoints --------
$('btn-plan').onclick = async () => {
  if (!waypoints.length) { $('mission-info').innerHTML = '⚠️ Add at least one waypoint (➕ then click the map)'; return; }
  const isAirborne = droneIsArmed && droneRelAlt > 2.0;
  const start = (isAirborne && currentDroneLL) ? currentDroneLL : [BASE.lat, BASE.lon];
  const wps = waypoints.map(w => ({ lat: w.ll[0], lon: w.ll[1], alt: w.alt }));
  $('mission-info').textContent = 'Planning exclusion-aware route…';
  try {
    const r = await (await fetch('/api/plan_multi', { method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ start, waypoints: wps, alt: wpAlt() }) })).json();
    if (!r.ok) { $('mission-info').textContent = 'Plan failed: ' + r.error; return; }
    routeLL = r.route;                                   // [[lat,lon,alt],...]
    if (routeLine) routeLine.remove();
    routeLine = L.polyline(routeLL.map(p => [p[0], p[1]]), { color:'#38bdf8', weight:3, dashArray:'4 6' }).addTo(map);
    let msg = `✓ Route planned (<b>${routeLL.length}</b> legs)`;
    if (r.clamped && r.clamped.length) {
      msg += ` · <span style="color:#fbbf24;">⚠️ WP ${r.clamped.map(i=>i+1).join(', ')} in a no-fly zone → routed to the safe edge (won't enter)</span>`;
    }
    $('mission-info').innerHTML = msg + ' · Click <b>Upload mission</b>';
  } catch(e) {
    $('mission-info').textContent = 'Plan error: ' + e.message;
  }
};

// -------- UPLOAD: per-waypoint heights + adjustable auto-RTL altitude --------
$('btn-upload').onclick = async () => {
  if (!routeLL) { $('mission-info').innerHTML = '⚠️ Add waypoints → <b>Plan route</b> first'; return; }
  $('mission-info').textContent = 'Uploading mission to flight controller…';
  const rtl = $('chk-rtl').checked;
  const rtlAlt = Math.max(10, Math.min(200, +$('rtl-alt').value || 60));
  const isAirborne = droneIsArmed && droneRelAlt > 2.0;
  try {
    const r = await (await fetch('/api/mission/upload', { method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ route: routeLL, alt: wpAlt(), rtl, rtl_alt: rtlAlt, is_airborne: isAirborne, hold_s: 15.0 }) })).json();
    if (r.ok) {
      $('mission-info').innerHTML = `✓ Mission uploaded (<b>${r.count}</b> items · RTL ${rtl ? ('@ '+rtlAlt+' m') : 'off'}) · Click <b>Arm</b> then <b>Start</b>`;
    } else {
      $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ Upload failed (${r.error}). Is SITL running?</span>`;
    }
  } catch(e) {
    $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ Connection error: ${e.message}</span>`;
  }
};

// -------- command helper --------
function cmdPost(payload) {
  return fetch('/api/command', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload) });
}
const cmd = c => cmdPost({cmd: c});

$('btn-arm').onclick = () => {
  const isArmed = $('armed').textContent.includes('ARMED');
  cmd(isArmed ? 'disarm' : 'arm');
  $('mission-info').innerHTML = isArmed ? 'Command: Disarming' : 'Command: 🔓 <b>Arming motors</b>';
};
$('btn-fly').onclick = () => { cmd('start_mission'); $('mission-info').innerHTML = '🚀 <b>Mission started ▶ Auto-Takeoff</b>'; };
$('btn-rtl').onclick = () => { cmd('rtl'); $('mission-info').innerHTML = '⟲ <b>RTL Commanded (Returning to Port Base)</b>'; };
$('btn-stop-mission').onclick = () => {
  cmd('stop_mission');
  if (routeLine) { routeLine.remove(); routeLine = null; }
  routeLL = null;
  $('mission-info').innerHTML = '⏹ <b>Mission stopped</b> — holding in place & cleared. Re-Plan then Upload for a new run.';
};
$('btn-drop').onclick = () => {
  if (confirm('Deploy SAR Cargo / Buoy Release Mechanism?')) {
    cmd('drop');
    $('mission-info').innerHTML = '<span style="color:#f97316;font-weight:bold;">🛟 Cargo release command triggered!</span>';
  }
};

// -------- FLY DIRECT TO LAST WAYPOINT (reposition, no-fly-aware) --------
$('btn-goto').onclick = async () => {
  if (!waypoints.length) {
    $('mission-info').innerHTML = '⚠️ <b>Add a waypoint first</b> (➕ then click the map).';
    return;
  }
  const w = waypoints[waypoints.length - 1];
  const alt = w.alt || wpAlt();
  $('mission-info').innerHTML = `📍 Flying direct to WP ${waypoints.length} at ${alt} m…`;
  try {
    const r = await (await cmdPost({ cmd: 'reposition', lat: w.ll[0], lon: w.ll[1], alt })).json();
    if (r.ok) {
      $('mission-info').innerHTML = `📍 <b>Reposition sent</b> → WP ${waypoints.length} @ ${alt} m`;
    } else {
      $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ ${r.error}</span>`;
    }
  } catch(e) {
    $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ Reposition error: ${e.message}</span>`;
  }
};

// -------- DYNAMIC ALTITUDE CHANGE --------
const altNum = $('alt'), altSlider = $('alt-slider');
let altChangeTimeout = null;
function syncAlt(v) {
  v = Math.max(5, Math.min(200, Math.round(+v || 45)));
  altNum.value = v;
  altSlider.value = v;
  // If drone is flying, push altitude change after a short debounce
  if (droneIsArmed && droneRelAlt > 2.0) {
    clearTimeout(altChangeTimeout);
    altChangeTimeout = setTimeout(() => {
      // Use reposition to current lat/lon but new altitude
      if (currentDroneLL) {
        cmdPost({ cmd: 'reposition', lat: currentDroneLL[0], lon: currentDroneLL[1], alt: v });
        $('mission-info').innerHTML = `📐 <b>Altitude change → ${v}m</b> commanded`;
      }
    }, 600); // 600ms debounce so slider dragging doesn't spam commands
  }
}
altNum.oninput = () => syncAlt(altNum.value);
altSlider.oninput = () => syncAlt(altSlider.value);

// ================= 3D VIEW (Cesium) =================
let viewer = null, cesReady = false, cesDrone = null, cesRoute = null, cesTrail = [], cesTrailEnt = null;
function init3D() {
  if (viewer) return;
  Cesium.Ion.defaultAccessToken = undefined;
  const esri = new Cesium.UrlTemplateImageryProvider({
    url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', maximumLevel: 19 });
  viewer = new Cesium.Viewer('cesiumContainer', {
    baseLayer: new Cesium.ImageryLayer(esri), baseLayerPicker:false, geocoder:false, timeline:false,
    animation:false, sceneModePicker:false, homeButton:false, navigationHelpButton:false,
    infoBox:false, selectionIndicator:false, fullscreenButton:false });
  viewer.scene.skyAtmosphere.show = true;
  if (opLayer) {
    const ring = ringLatLngs(opLayer).flatMap(p => [p[1], p[0]]);
    viewer.entities.add({ polygon: { hierarchy: Cesium.Cartesian3.fromDegreesArray(ring),
      material: Cesium.Color.fromCssColorString('#34d399').withAlpha(0.12),
      outline: true, outlineColor: Cesium.Color.fromCssColorString('#34d399') } });
  }
  fetch('/api/kml').then(r=>r.json()).then(g => g.features.forEach(f => {
    if (f.properties.kind === 'exclusion') {
      const ring = f.geometry.coordinates[0].flatMap(c => [c[0], c[1]]);
      viewer.entities.add({ polygon: { hierarchy: Cesium.Cartesian3.fromDegreesArray(ring), material: Cesium.Color.RED.withAlpha(0.3) } });
    }
  }));
  cesDrone = viewer.entities.add({
    position: new Cesium.CallbackProperty(() => cesDronePos, false),
    orientation: new Cesium.CallbackProperty(() =>
      Cesium.Transforms.headingPitchRollQuaternion(cesDronePos,
        new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(cesDroneHdg), 0, 0)), false),
    model: { uri: '/drone.glb', scale: 40, minimumPixelSize: 110, maximumScale: 20000 },
    label: { text: 'UAV', font: 'bold 12px sans-serif', pixelOffset: new Cesium.Cartesian2(0, -34),
      fillColor: Cesium.Color.YELLOW, showBackground: true, backgroundColor: Cesium.Color.BLACK.withAlpha(0.5),
      disableDepthTestDistance: Number.POSITIVE_INFINITY } });
  cesReady = true;
  viewer.camera.flyTo({ destination: Cesium.Cartesian3.fromDegrees(BASE.lon, BASE.lat - 0.025, 3000),
    orientation: { heading: 0, pitch: Cesium.Math.toRadians(-32), roll: 0 } });
}
let cesDronePos = Cesium.Cartesian3.fromDegrees(BASE.lon, BASE.lat, 0), cesDroneHdg = 0;
let cesFollowing = false;

function toggle3DFollow(force) {
  if (!viewer || !cesDrone) return;
  cesFollowing = (force !== undefined) ? force : !cesFollowing;
  viewer.trackedEntity = cesFollowing ? cesDrone : undefined;
  const b = $('btn-3d-follow');
  if (b) {
    b.textContent = cesFollowing ? '🎥 Following Drone (Click for Free Cam)' : '🎥 Follow Drone';
    b.style.background = cesFollowing ? '#059669' : '#0284c7';
  }
}

$('btn-3d-follow').onclick = () => toggle3DFollow();

$('btn-3d').onclick = () => {
  $('cesium-wrap').classList.add('on'); init3D();
  setTimeout(() => {
    if (viewer) {
      viewer.resize();
      toggle3DFollow(true);
    }
  }, 100);
  if (routeLL && viewer) {
    if (cesRoute) viewer.entities.remove(cesRoute);
    const dalt = +$('alt').value || 45;
    const heights = routeLL.flatMap(p => [p[1], p[0], (p[2] != null ? p[2] : dalt)]);
    cesRoute = viewer.entities.add({ polyline: {
      positions: heights.length ? Cesium.Cartesian3.fromDegreesArrayHeights(heights) : [],
      width: 4, material: Cesium.Color.fromCssColorString('#38bdf8'), clampToGround: false } });
  }
};
$('btn-2d').onclick = () => { $('cesium-wrap').classList.remove('on'); toggle3DFollow(false); };

// -------- Gazebo live video feed --------
$('btn-feed').onclick = () => {
  const panel = $('feed-panel'), img = $('feed-img'), on = !panel.classList.contains('on');
  panel.classList.toggle('on', on);
  if (on) {
    fetch('/api/gazebo/follow', { method: 'POST' }); // ensure Gazebo camera tracks the drone
  }
  img.src = on ? ('/video.mjpeg?' + Date.now()) : '';
};
$('feed-close').onclick = () => { $('feed-panel').classList.remove('on'); $('feed-img').src = ''; };

// -------- draggable + resizable windows (feed + 3D) --------
function makeDraggable(winId, onResize) {
  const win = $(winId), bar = win.querySelector('.win-bar');
  let sx, sy, sl, st, dragging = false;
  bar.addEventListener('mousedown', e => {
    if (e.target.closest('button, .win-x')) return;      // don't drag when hitting a control
    const r = win.getBoundingClientRect();
    win.style.left = r.left + 'px'; win.style.top = r.top + 'px';
    win.style.right = 'auto'; win.style.bottom = 'auto';
    sx = e.clientX; sy = e.clientY; sl = r.left; st = r.top; dragging = true;
    e.preventDefault();
  });
  window.addEventListener('mousemove', e => {
    if (!dragging) return;
    win.style.left = Math.max(0, sl + e.clientX - sx) + 'px';
    win.style.top  = Math.max(0, st + e.clientY - sy) + 'px';
  });
  window.addEventListener('mouseup', () => { dragging = false; });
  if (onResize && 'ResizeObserver' in window) new ResizeObserver(onResize).observe(win);
}
makeDraggable('feed-panel');
makeDraggable('cesium-wrap', () => { if (viewer) viewer.resize(); });

// ================= TELEMETRY =================
let droneMarker = null, trail = null, trailPts = [];
function droneIcon(h) { return L.divIcon({ className:'', iconSize:[34,34], iconAnchor:[17,17],
  html:`<svg width="34" height="34" viewBox="0 0 34 34" class="drone-icon" style="transform:rotate(${h}deg)">
    <polygon points="17,2 27,30 17,23 7,30" fill="#facc15" stroke="#111" stroke-width="1.5"/></svg>` }); }
const modeName = c => { const main=(c>>>16)&0xff, sub=(c>>>24)&0xff;
  const M={1:'MANUAL',2:'ALTCTL',3:'POSCTL',4:'AUTO',5:'ACRO',6:'OFFBOARD',7:'STAB'};
  const A={1:'READY',2:'TAKEOFF',3:'HOLD',4:'MISSION',5:'RTL',6:'LAND',8:'FOLLOW',9:'PRECLAND'};
  return main===4 ? 'AUTO.'+(A[sub]||sub) : (M[main]||('mode '+main)); };
async function tick() {
  let t; try { t = await (await fetch('/api/telemetry')).json(); } catch(e) { t = { connected:false }; }
  $('conn').textContent = t.connected ? 'LINK' : 'NO LINK'; $('conn').className = 'pill ' + (t.connected?'on':'off');
  $('mode').textContent  = (t.custom_mode!=null) ? modeName(t.custom_mode) : '—';
  $('armed').textContent = (t.armed==null)?'—':(t.armed?'ARMED':'disarmed'); $('armed').style.color = t.armed?'#7ff0ac':'#ff9c9c';
  $('relalt').textContent = (t.rel_alt!=null)?t.rel_alt.toFixed(1)+' m':'—';
  $('gspd').textContent   = (t.groundspeed!=null)?t.groundspeed.toFixed(1)+' m/s':'—';
  $('hdg').textContent    = (t.hdg!=null)?Math.round(t.hdg)+'°':'—';
  $('batt').textContent   = (t.battery_v!=null)?t.battery_v.toFixed(1)+' V'+(t.battery_pct!=null&&t.battery_pct>=0?` (${t.battery_pct}%)`:''):'—';
  $('gps').textContent    = (t.sats!=null)?`${t.sats} sats`+(t.fix?` / ${String(t.fix).replace('GPS_FIX_TYPE_','')}`:''):'—';
  $('ll').textContent     = (t.lat!=null)?`${t.lat.toFixed(6)}, ${t.lon.toFixed(6)}`:'—';

  // ---- UPDATE GLOBAL DRONE STATE ----
  if (t.lat != null && t.lon != null) {
    currentDroneLL = [t.lat, t.lon];
  }
  droneIsArmed = !!t.armed;
  droneRelAlt = (t.rel_alt != null) ? t.rel_alt : 0;

  if (t.lat!=null && t.lon!=null) {
    const ll = [t.lat, t.lon];
    droneMarker ? (droneMarker.setLatLng(ll), droneMarker.setIcon(droneIcon(t.hdg||0)))
                : (droneMarker = L.marker(ll, { icon: droneIcon(t.hdg||0), zIndexOffset:1000 }).addTo(map));
    const alt = (t.rel_alt!=null && t.rel_alt>0) ? t.rel_alt : 0;
    cesDronePos = Cesium.Cartesian3.fromDegrees(t.lon, t.lat, alt);
    cesDroneHdg = (t.hdg != null) ? t.hdg : cesDroneHdg;
    if (t.rel_alt!=null && t.rel_alt>0.5) {
      trailPts.push(ll); if (trailPts.length>2000) trailPts.shift();
      trail ? trail.setLatLngs(trailPts) : (trail = L.polyline(trailPts, { color:'#facc15', weight:2, opacity:.8 }).addTo(map));
      if (cesReady) { cesTrail.push(Cesium.Cartesian3.fromDegrees(t.lon, t.lat, alt)); if (cesTrail.length>2000) cesTrail.shift();
        if (!cesTrailEnt) cesTrailEnt = viewer.entities.add({ polyline: { positions: new Cesium.CallbackProperty(()=>cesTrail,false),
          width:2, material: Cesium.Color.YELLOW } }); }
    }
  }
}
setInterval(tick, 500); tick();
