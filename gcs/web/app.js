/* Maritime GCS — live map, draw/edit boundary, plan+upload mission, 2D+3D */
const $ = id => document.getElementById(id);
let BASE = { lat: 22.318725, lon: 91.813156 };
// CAAB rule we enforce (filled from /api/caab): the altitude ceiling.
let CAAB = { max_alt_ft: 200, max_alt_m: 60.96 };

// ================= LIVE DRONE STATE (updated every tick) =================
let currentDroneLL = null;   // [lat, lon] from telemetry
let droneIsArmed = false;
let droneRelAlt = 0;

// ================= 2D MAP =================
const map = L.map('map', { zoomControl: true, attributionControl: false }).setView([22.31, 91.81], 12);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19 }).addTo(map);
// map lives in a flex grid cell — make sure Leaflet picks up the real size
setTimeout(() => map.invalidateSize(), 0);
window.addEventListener('resize', () => map.invalidateSize());

// -------- theme (light / dark) toggle, persisted --------
(function initTheme() {
  const chk = $('theme-chk'); if (!chk) return;
  const q = new URLSearchParams(location.search).get('theme');
  if (q === 'light' || q === 'dark') localStorage.setItem('gcs-theme', q);
  const light = localStorage.getItem('gcs-theme') === 'light';
  document.body.classList.toggle('theme-light', light); chk.checked = light;
  chk.onchange = () => {
    const on = chk.checked;
    document.body.classList.toggle('theme-light', on);
    localStorage.setItem('gcs-theme', on ? 'light' : 'dark');
  };
})();

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

// -------- CAAB altitude ceiling (200 ft AGL) --------
// No-fly zones come from the KML exclusion layer; the CAAB rule we enforce here
// is only the altitude cap (the 18.5 km airport radius blankets all of Chattogram
// so it isn't drawn on the map).
fetch('/api/caab').then(r => r.json()).then(c => {
  CAAB = c;
  const capM = Math.floor(c.max_alt_m);   // whole-metre ceiling for the UI inputs
  ['alt', 'alt-slider', 'rtl-alt'].forEach(id => {
    const el = $(id); if (el) { el.max = capM; if (+el.value > capM) el.value = capM; }
  });
  const el = $('caab-info');
  if (el) el.innerHTML = `CAAB max altitude: ≤ ${c.max_alt_ft} ft (${capM} m) AGL`;
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

function wpAlt() { return Math.max(5, Math.min(CAAB.max_alt_m, Math.round(+$('alt').value || 45))); }
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
  const rtlAlt = Math.max(10, Math.min(CAAB.max_alt_m, +$('rtl-alt').value || 60));
  const isAirborne = droneIsArmed && droneRelAlt > 2.0;
  try {
    const r = await (await fetch('/api/mission/upload', { method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ route: routeLL, alt: wpAlt(), rtl, rtl_alt: rtlAlt, is_airborne: isAirborne, hold_s: 15.0, home_lat: BASE.lat, home_lon: BASE.lon }) })).json();
    if (r.ok) {
      const cap = r.alt_capped_to_m ? ` · <span style="color:#f59e0b;">⚠️ altitude capped to CAAB ${Math.floor(r.alt_capped_to_m)} m</span>` : '';
      $('mission-info').innerHTML = `✓ Mission uploaded (<b>${r.count}</b> items · RTL ${rtl ? ('@ '+Math.round(rtlAlt)+' m') : 'off'})${cap} · Click <b>Arm</b> then <b>Start</b>`;
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

// -------- ALTITUDE selector (target height for new WPs + Alt-Hold) --------
// The slider only SETS the target now — it no longer auto-commands the drone (that
// used to reposition-hold it, stopping the mission). Use "Alt Hold" to apply it live.
const altNum = $('alt'), altSlider = $('alt-slider');
function syncAlt(v) {
  v = Math.max(5, Math.min(Math.floor(CAAB.max_alt_m), Math.round(+v || 45)));
  altNum.value = v;
  altSlider.value = v;
}
altNum.oninput = () => syncAlt(altNum.value);
altSlider.oninput = () => syncAlt(altSlider.value);

// -------- ALT HOLD: climb/descend to the selected altitude & loiter here --------
$('btn-alt-hold').onclick = async () => {
  if (!currentDroneLL) { $('mission-info').innerHTML = '⚠️ No drone position yet (is SITL up & armed?)'; return; }
  const alt = wpAlt();
  $('mission-info').innerHTML = `⬍ Holding at current position → ${alt} m…`;
  try {
    const r = await (await cmdPost({ cmd:'reposition', lat: currentDroneLL[0], lon: currentDroneLL[1], alt })).json();
    $('mission-info').innerHTML = r.ok
      ? `⬍ <b>Alt-hold @ ${alt} m</b> (loitering here) — press <b>Proceed</b> to resume the mission`
      : `<span style="color:#ef4444;">⚠️ ${r.error || 'hold failed'}</span>`;
  } catch(e) { $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ ${e.message}</span>`; }
};

// -------- PROCEED: resume the uploaded mission from the current item --------
$('btn-proceed').onclick = async () => {
  $('mission-info').innerHTML = '▶ Resuming mission…';
  try {
    const r = await (await cmdPost({ cmd:'continue_mission' })).json();
    $('mission-info').innerHTML = r.ok
      ? '▶ <b>Proceeding</b> — mission resumed (AUTO.MISSION)'
      : `<span style="color:#ef4444;">⚠️ ${r.error || 'resume failed'}</span>`;
  } catch(e) { $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ ${e.message}</span>`; }
};

// -------- TARGET GROUND-SPEED (safe to apply live — doesn't stop the mission) --------
const spdNum = $('spd'), spdSlider = $('spd-slider');
let spdChangeTimeout = null;
function syncSpeed(v) {
  v = Math.max(1, Math.min(20, Math.round(+v || 5)));
  spdNum.value = v;
  spdSlider.value = v;
  clearTimeout(spdChangeTimeout);
  spdChangeTimeout = setTimeout(() => {
    cmdPost({ cmd: 'set_speed', speed: v });
    $('mission-info').innerHTML = `🚤 <b>Target speed → ${v} m/s</b> commanded`;
  }, 500);   // debounce so slider dragging doesn't spam commands
}
spdNum.oninput = () => syncSpeed(spdNum.value);
spdSlider.oninput = () => syncSpeed(spdSlider.value);

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
  cesDronePos = Cesium.Cartesian3.fromDegrees(BASE.lon, BASE.lat, 0);
  cesReady = true;
  viewer.camera.flyTo({ destination: Cesium.Cartesian3.fromDegrees(BASE.lon, BASE.lat - 0.025, 3000),
    orientation: { heading: 0, pitch: Cesium.Math.toRadians(-32), roll: 0 } });
}
let cesDronePos = null, cesDroneHdg = 0;
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
  const wrap = $('cesiumContainer');
  const on = !wrap.classList.contains('on');
  wrap.classList.toggle('on', on);
  $('ces-tools').classList.toggle('on', on);
  $('btn-3d').classList.toggle('active', on);
  if (!on) { toggle3DFollow(false); return; }        // back to the 2D map
  init3D();
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
$('btn-2d').onclick = () => {
  $('cesiumContainer').classList.remove('on');
  $('ces-tools').classList.remove('on');
  $('btn-3d').classList.remove('active');
  toggle3DFollow(false);
};

// -------- inline camera feeds (chase + fpv) --------
function makeCamToggle(btnId, imgId, url, onStart) {
  const btn = $(btnId), img = $(imgId);
  btn.onclick = () => {
    const on = !btn.classList.contains('active');
    btn.classList.toggle('active', on);
    btn.textContent = on ? '⏸ stop' : '▶ start';
    if (on && onStart) onStart();
    img.src = on ? (url + '?' + Date.now()) : '';
  };
}
makeCamToggle('btn-feed', 'feed-img', '/video.mjpeg',
  () => fetch('/api/gazebo/follow', { method: 'POST' }));   // chase cam tracks the drone
makeCamToggle('btn-feed2', 'feed2-img', '/video2.mjpeg');   // fpv (forward, angled down)
// ?feeds=on -> auto-start both camera feeds (handy for demos/screenshots)
if (new URLSearchParams(location.search).get('feeds') === 'on') {
  ['btn-feed', 'btn-feed2'].forEach(id => { const b = $(id); if (b && !b.classList.contains('active')) b.click(); });
}

// ================= TELEMETRY =================
let droneMarker = null, trail = null, trailPts = [];
function droneIcon(h) { return L.divIcon({ className:'', iconSize:[34,34], iconAnchor:[17,17],
  html:`<svg width="34" height="34" viewBox="0 0 34 34" class="drone-icon" style="transform:rotate(${h}deg)">
    <polygon points="17,2 27,30 17,23 7,30" fill="#facc15" stroke="#111" stroke-width="1.5"/></svg>` }); }
const modeName = c => { const main=(c>>>16)&0xff, sub=(c>>>24)&0xff;
  const M={1:'MANUAL',2:'ALTCTL',3:'POSCTL',4:'AUTO',5:'ACRO',6:'OFFBOARD',7:'STAB'};
  const A={1:'READY',2:'TAKEOFF',3:'HOLD',4:'MISSION',5:'RTL',6:'LAND',8:'FOLLOW',9:'PRECLAND'};
  return main===4 ? 'AUTO.'+(A[sub]||sub) : (M[main]||('mode '+main)); };
// -------- flight status panel (timer, distance-to-base, max alt, battery) --------
let flightStart = null, maxAltSeen = 0;
function updateStatus(t) {
  // flight timer: runs while armed, resets on disarm
  if (t.armed) { if (flightStart == null) flightStart = Date.now(); }
  else { flightStart = null; maxAltSeen = 0; }
  if (flightStart != null) {
    const s = Math.floor((Date.now() - flightStart) / 1000);
    $('st-time').textContent = String(Math.floor(s/60)).padStart(2,'0') + ':' + String(s%60).padStart(2,'0');
  } else {
    $('st-time').textContent = '00:00';
  }

  // distance from base
  if (t.lat != null && t.lon != null) {
    const km = haversineKm([BASE.lat, BASE.lon], [t.lat, t.lon]);
    $('st-dist').textContent = km < 1 ? Math.round(km * 1000) + ' m' : km.toFixed(2) + ' km';
  } else { $('st-dist').textContent = '—'; }

  // max altitude reached this flight
  if (t.rel_alt != null) maxAltSeen = Math.max(maxAltSeen, t.rel_alt);
  $('st-maxalt').textContent = maxAltSeen > 0 ? maxAltSeen.toFixed(1) + ' m' : '—';

  $('st-sats').textContent = (t.sats != null) ? t.sats : '—';

  // battery bar
  const pct = (t.battery_pct != null && t.battery_pct >= 0) ? t.battery_pct : null;
  const fill = $('st-batt-fill');
  if (pct != null) {
    $('st-batt').textContent = pct + '%' + (t.battery_v != null ? ` · ${t.battery_v.toFixed(1)} V` : '');
    fill.style.width = pct + '%';
    fill.className = pct <= 20 ? 'crit' : (pct <= 40 ? 'warn' : '');
  } else {
    $('st-batt').textContent = (t.battery_v != null) ? t.battery_v.toFixed(1) + ' V' : '—';
    fill.style.width = '0%'; fill.className = '';
  }
}

// Displayed heading: the vehicle yaw (hdg) doesn't track travel in this sim, so use
// course-over-ground from the NED velocity vector when moving; fall back to hdg slow.
let lastHeading = 0;
function dispHeading(t) {
  if (t.vx != null && t.vy != null && Math.hypot(t.vx, t.vy) > 0.6) {
    lastHeading = (Math.atan2(t.vy, t.vx) * 180 / Math.PI + 360) % 360;
  } else if (t.hdg != null) {
    lastHeading = t.hdg;
  }
  return lastHeading;
}

async function tick() {
  let t; try { t = await (await fetch('/api/telemetry')).json(); } catch(e) { t = { connected:false }; }
  $('conn').textContent = t.connected ? 'LINK' : 'NO LINK'; $('conn').className = 'pill ' + (t.connected?'on':'off');
  const H = t.connected ? dispHeading(t) : null;   // course-over-ground heading
  $('mode').textContent  = (t.custom_mode!=null) ? modeName(t.custom_mode) : '—';
  $('armed').textContent = (t.armed==null)?'—':(t.armed?'ARMED':'disarmed'); $('armed').style.color = t.armed?'#7ff0ac':'#ff9c9c';
  $('relalt').textContent = (t.rel_alt!=null)?t.rel_alt.toFixed(1)+' m':'—';
  $('gspd').textContent   = (t.groundspeed!=null)?t.groundspeed.toFixed(1)+' m/s':'—';
  $('hdg').textContent    = (H!=null)?Math.round(H)+'°':'—';
  $('batt').textContent   = (t.battery_v!=null)?t.battery_v.toFixed(1)+' V'+(t.battery_pct!=null&&t.battery_pct>=0?` (${t.battery_pct}%)`:''):'—';
  $('gps').textContent    = (t.sats!=null)?`${t.sats} sats`+(t.fix?` / ${String(t.fix).replace('GPS_FIX_TYPE_','')}`:''):'—';
  $('ll').textContent     = (t.lat!=null)?`${t.lat.toFixed(6)}, ${t.lon.toFixed(6)}`:'—';

  // ---- header dashboard readouts ----
  const gs = (t.groundspeed!=null) ? t.groundspeed : null;
  $('d-gspd').textContent  = (gs!=null)?gs.toFixed(1)+' m/s':'—';
  $('d-alt').textContent   = (t.rel_alt!=null)?t.rel_alt.toFixed(1)+' m':'—';
  $('d-climb').textContent = (t.climb!=null)?(t.climb>=0?'+':'')+t.climb.toFixed(1)+' m/s':'—';
  $('d-hdg').textContent   = (H!=null)?Math.round(H)+'°':'—';
  $('d-mode').textContent  = (t.custom_mode!=null)?modeName(t.custom_mode):'—';
  $('d-batt').textContent  = (t.battery_v!=null)?t.battery_v.toFixed(1)+' V':'—';

  // ---- orientation compass (rotates to travel heading) ----
  if (H != null) {
    $('cmp-needle').setAttribute('transform', `rotate(${H} 60 60)`);
    $('cmp-hdg').textContent = Math.round(H) + '°';
  }

  // ---- attitude indicator (roll + pitch, artificial horizon) ----
  if (t.roll != null && t.pitch != null) {
    const px = Math.max(-40, Math.min(40, t.pitch)) * 1.1;   // px offset per degree
    $('adi-move').setAttribute('transform', `rotate(${-t.roll} 50 50) translate(0 ${px})`);
  }

  // ---- velocity gauge (left panel) ----
  const v = (gs!=null) ? gs : 0;
  $('vel-big').textContent = v.toFixed(1);
  $('vel-fill').style.width = Math.max(0, Math.min(100, (v / 20) * 100)) + '%';  // scale 0–20 m/s

  // ---- flight status panel ----
  updateStatus(t);

  // ---- UPDATE GLOBAL DRONE STATE ----
  if (t.lat != null && t.lon != null) {
    currentDroneLL = [t.lat, t.lon];
  }
  droneIsArmed = !!t.armed;
  droneRelAlt = (t.rel_alt != null) ? t.rel_alt : 0;

  if (t.lat!=null && t.lon!=null) {
    const ll = [t.lat, t.lon];
    const iconHdg = (H != null) ? H : (t.hdg || 0);
    droneMarker ? (droneMarker.setLatLng(ll), droneMarker.setIcon(droneIcon(iconHdg)))
                : (droneMarker = L.marker(ll, { icon: droneIcon(iconHdg), zIndexOffset:1000 }).addTo(map));
    const alt = (t.rel_alt!=null && t.rel_alt>0) ? t.rel_alt : 0;
    // 3D is optional — never let a missing/slow Cesium break the 2D map + telemetry
    if (cesReady && window.Cesium) {
      cesDronePos = Cesium.Cartesian3.fromDegrees(t.lon, t.lat, alt);
      cesDroneHdg = (H != null) ? H : cesDroneHdg;
    }
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
