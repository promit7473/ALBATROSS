/* Maritime GCS — live map, draw/edit boundary, plan+upload mission, 2D+3D */
const $ = id => document.getElementById(id);
let BASE = { lat: 22.318725, lon: 91.813156 };
// CAAB rule we enforce (filled from /api/caab): the altitude ceiling.
let CAAB = { max_alt_ft: 200, max_alt_m: 60.96 };

// ================= LIVE DRONE STATE (updated every tick) =================
let currentDroneLL = null;   // [lat, lon] from telemetry
let droneIsArmed = false;
let droneRelAlt = 0;
let GCS_MODE = 'sitl';
const modeReady = fetch('/api/mode').then(r => r.json()).then(m => {
  GCS_MODE = (m && m.mode) || 'sitl';
  document.body.classList.toggle('mode-real', GCS_MODE === 'real');
  return GCS_MODE;
}).catch(() => GCS_MODE);

function gpsHasFix(t) {
  const fix = String(t && t.fix != null ? t.fix : '');
  if (fix.indexOf('NO_GPS') >= 0 || fix.indexOf('NO_FIX') >= 0 || fix === '0' || fix === '1') return false;
  if (t && typeof t.sats === 'number' && t.sats <= 0) return false;
  const n = parseInt(fix, 10);
  if (String(n) === fix) return n >= 2;
  return fix.startsWith('GPS_FIX_TYPE_');
}

function validLL(lat, lon) {
  return lat != null && lon != null && isFinite(lat) && isFinite(lon)
    && Math.abs(lat) > 0.01 && Math.abs(lon) > 0.01
    && Math.abs(lat) <= 90 && Math.abs(lon) <= 180;
}

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
    if (ring) makeOpLayer(ring);
    modeReady.then(mode => {
      // SITL: frame the Chattogram operating area. Real aircraft: wait for GPS
      // (or the operator geolocation) instead of locking the view on the KML.
      if (mode !== 'real' && opLayer) {
        try { map.fitBounds(opLayer.getBounds().pad(0.2)); } catch (e) {}
      }
    });
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

function setModeBtn(id) {
  ['btn-mode-plan', 'btn-mode-manual', 'btn-mode-twin'].forEach(k => {
    const b = $(k); if (b) b.classList.toggle('active', k === id);
  });
}
function sitlHome() {
  const ll = liveLL();
  return { lat: +ll[0].toFixed(7), lon: +ll[1].toFixed(7) };
}
async function startSim(kind) {
  const home = sitlHome();
  $('mission-info').innerHTML = kind === 'manual'
    ? `⏳ <b>Manual sim</b> at ${home.lat.toFixed(5)}, ${home.lon.toFixed(5)} — RC flies Gazebo, real props stay off (≈15s)…`
    : `⏳ <b>Planning sim</b> at ${home.lat.toFixed(5)}, ${home.lon.toFixed(5)} (≈15s)…`;
  const res = await (await fetch('/api/sitl/start', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode: kind, lat: home.lat, lon: home.lon })
  })).json();
  if (res.ok) {
    const h = res.home || home;
    const world = res.world || 'chattogram';
    const hlat = Number(h.lat), hlon = Number(h.lon);
    const where = (isFinite(hlat) && isFinite(hlon))
      ? `${world} at ${hlat.toFixed(5)}, ${hlon.toFixed(5)}`
      : world;
    $('mission-info').innerHTML = `🚀 <b>Simulator booting</b> — ${where}. Gazebo is the physics twin.`;
  } else {
    throw new Error(res.error || 'launch failed');
  }
}
$('btn-sitl-start').onclick = async () => {
  try { setModeBtn('btn-mode-plan'); await startSim('plan'); }
  catch (e) { $('mission-info').textContent = 'Error: ' + e.message; }
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
  if (GCS_MODE === 'real' && !currentDroneLL) {
    $('mission-info').textContent = 'Wait for live aircraft GPS before planning a real mission.';
    return;
  }
  const start = ((isAirborne || GCS_MODE === 'real') && currentDroneLL) ? currentDroneLL : [BASE.lat, BASE.lon];
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
async function cmdPost(payload) {
  const response = await fetch('/api/command', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload) });
  const result = await response.clone().json();
  if (!response.ok || !result.ok) throw new Error(result.error || 'Command could not be sent');
  return response;
}
const cmd = c => cmdPost({cmd: c});
async function sendFlightCommand(buttonId, command, message) {
  const button = $(buttonId);
  button.disabled = true;
  $('mission-info').textContent = 'Sending command…';
  try {
    await cmd(command);
    $('mission-info').textContent = message + ' — check live flight status for confirmation.';
    return true;
  } catch (e) {
    $('mission-info').textContent = 'Command failed: ' + e.message;
    return false;
  } finally {
    button.disabled = false;
  }
}

if ($('btn-mode-plan')) {
  $('btn-mode-plan').onclick = async () => {
    setModeBtn('btn-mode-plan');
    try { await startSim('plan'); } catch (e) { $('mission-info').textContent = 'Error: ' + e.message; }
  };
  $('btn-mode-manual').onclick = async () => {
    setModeBtn('btn-mode-manual');
    try { await startSim('manual'); } catch (e) { $('mission-info').textContent = 'Error: ' + e.message; }
  };
  $('btn-mode-twin').onclick = () => {
    setModeBtn('btn-mode-twin');
    $('mission-info').textContent = 'Aircraft 3D shows received GPS and attitude. Flight controls below act on the connected aircraft.';
    if ($('btn-3d') && !$('btn-3d').classList.contains('active')) $('btn-3d').click();
  };
}

$('btn-arm').onclick = () => {
  if (GCS_MODE === 'real' && !droneIsArmed && !confirm('Arm the real aircraft? Motors may spin.')) return;
  sendFlightCommand('btn-arm', droneIsArmed ? 'disarm' : 'arm', droneIsArmed ? 'Disarm request sent' : 'Arm request sent');
};
$('btn-fly').onclick = () => sendFlightCommand('btn-fly', 'start_mission', 'Mission start request sent');
$('btn-rtl').onclick = () => sendFlightCommand('btn-rtl', 'rtl', 'Return-to-launch request sent');

// -------- MANUAL <-> AUTO toggle (one button; flips PX4 flight mode) --------
// AUTO.*  -> POSCTL (pilot gets the sticks) ; otherwise -> AUTO.MISSION (resume).
// RC always has override priority on a real Pixhawk; this just picks the mode.
$('btn-toggle-mode').onclick = async () => {
  try {
    const r = await (await cmdPost({ cmd: 'toggle_mode' })).json();
    if (!r.ok) { $('mission-info').innerHTML = '<span style="color:#ef4444;">⚠️ Toggle failed</span>'; return; }
    // Only live telemetry confirms the mode; a successful POST means sent.
    $('mission-info').textContent = (r.to === 'manual' ? 'Manual mode' : 'Auto mission mode')
      + ' request sent — check live flight status for confirmation.';
  } catch(e) { $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ ${e.message}</span>`; }
};
function refreshToggleBtn(mainMode) {
  const btn = $('btn-toggle-mode');
  if (!btn) return;
  const inAuto = mainMode === 4;                                    // AUTO.*
  btn.textContent = inAuto ? '✋ Manual' : '🤖 Auto';
  btn.classList.toggle('target-auto', !inAuto);
  btn.classList.toggle('target-manual', inAuto);
}

$('btn-stop-mission').onclick = async () => {
  if (!await sendFlightCommand('btn-stop-mission', 'stop_mission', 'Hold and clear mission requests sent')) return;
  if (routeLine) { routeLine.remove(); routeLine = null; }
  routeLL = null;
};
$('btn-drop').onclick = () => {
  if (confirm('Deploy SAR Cargo / Buoy Release Mechanism?')) {
    sendFlightCommand('btn-drop', 'drop', 'Cargo release request sent');
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
      ? `Altitude hold request sent @ ${alt} m — check live flight status for confirmation`
      : `<span style="color:#ef4444;">⚠️ ${r.error || 'hold failed'}</span>`;
  } catch(e) { $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ ${e.message}</span>`; }
};

// -------- PROCEED: resume the uploaded mission from the current item --------
$('btn-proceed').onclick = async () => {
  $('mission-info').innerHTML = '▶ Resuming mission…';
  try {
    const r = await (await cmdPost({ cmd:'continue_mission' })).json();
    $('mission-info').innerHTML = r.ok
      ? 'Resume mission request sent — check live flight status for confirmation'
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
    cmdPost({ cmd: 'set_speed', speed: v }).catch(e => {
      $('mission-info').textContent = 'Speed command failed: ' + e.message;
    });
    $('mission-info').innerHTML = `🚤 <b>Target speed → ${v} m/s</b> commanded`;
  }, 500);   // debounce so slider dragging doesn't spam commands
}
spdNum.oninput = () => syncSpeed(spdNum.value);
spdSlider.oninput = () => syncSpeed(spdSlider.value);

// ================= 3D VIEW (Cesium) =================
let viewer = null, cesReady = false, cesDrone = null, cesRoute = null, cesTrail = [], cesTrailEnt = null;
let cesDronePos = null, cesDroneHdg = 0, cesRoll = 0, cesPitch = 0;
let cesFollowing = false;
let cesGpsView = false;
let cesGpsSats = [];      // { body, beam, locked, inView, ecef() }
let cesBuildingEnts = [];
let cesBuildingsAt = null;
let cesBuildingsToken = 0;
let cesBuildingsRetryAt = 0;
let lastGpsTel = { sats: 0, fix: null, gps_sats: null, h_acc_m: null };

function liveLL() {
  if (currentDroneLL) return currentDroneLL;
  try {
    const c = map.getCenter();
    if (c && validLL(c.lat, c.lng)) return [c.lat, c.lng];
  } catch (e) {}
  return [BASE.lat, BASE.lon];
}

function aglMeters(rel) {
  const a = (typeof rel === 'number' && isFinite(rel)) ? rel : 0;
  return Math.max(0, a);
}

function cesSafePos() {
  if (cesDronePos) return cesDronePos;
  const ll = liveLL();
  return Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 0);
}

function lookAtLive(duration) {
  if (!viewer) return;
  const ll = liveLL();
  const dest = Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 280);
  viewer.camera.flyTo({
    destination: dest,
    orientation: { heading: 0, pitch: Cesium.Math.toRadians(-42), roll: 0 },
    duration: duration == null ? 1.1 : duration
  });
}

async function enableWorldTerrain() {
  if (!viewer || !window.Cesium) return;
  const url = 'https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer';
  try {
    let provider = null;
    if (Cesium.ArcGISTiledElevationTerrainProvider && Cesium.ArcGISTiledElevationTerrainProvider.fromUrl) {
      provider = await Cesium.ArcGISTiledElevationTerrainProvider.fromUrl(url);
    } else if (Cesium.ArcGisMapServerImageryProvider) {
      provider = new Cesium.ArcGISTiledElevationTerrainProvider({ url });
    }
    if (provider) {
      viewer.terrainProvider = provider;
      viewer.scene.globe.depthTestAgainstTerrain = true;
      console.log('[3d] ArcGIS world terrain on');
    }
  } catch (e) {
    console.warn('[3d] terrain unavailable', e);
  }
}

function loadAreaBuildings(ll) {
  if (!viewer || !ll || !validLL(ll[0], ll[1])) return;
  if (Date.now() < cesBuildingsRetryAt) return;
  if (cesBuildingsAt && haversineKm(cesBuildingsAt, ll) < 1) return;
  const lat = ll[0], lon = ll[1];
  cesBuildingsAt = [lat, lon];
  const token = ++cesBuildingsToken;
  cesBuildingEnts.forEach(ent => { try { viewer.entities.remove(ent); } catch (e) {} });
  cesBuildingEnts = [];
  fetch(`/api/buildings?lat=${lat}&lon=${lon}&r=700`)
    .then(r => r.json())
    .then(d => {
      if (token !== cesBuildingsToken || !viewer) return;
      const list = (d && d.buildings) || [];
      list.forEach(b => {
        if (!b.ring || b.ring.length < 3) return;
        const flat = b.ring.flat();
        try {
          const ent = viewer.entities.add({
            polygon: {
              hierarchy: Cesium.Cartesian3.fromDegreesArray(flat),
              height: 0,
              extrudedHeight: b.height || 10,
              heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
              extrudedHeightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
              material: Cesium.Color.fromCssColorString('#cbd5e1').withAlpha(0.82),
              outline: true,
              outlineColor: Cesium.Color.fromCssColorString('#64748b').withAlpha(0.7)
            }
          });
          cesBuildingEnts.push(ent);
        } catch (e) {}
      });
      console.log('[3d] buildings', list.length);
    })
    .catch(e => {
      console.warn('[3d] buildings', e);
      if (token === cesBuildingsToken) {
        cesBuildingsAt = null;
        cesBuildingsRetryAt = Date.now() + 20000;
      }
    });
}

// GPS Walker 24/6 constellation (same geometry the real GPS network uses).
const GPS_PERIOD = 43082;             // seconds, ~12h sidereal

function gpsSatEcef(idx, tSec) {
  const a = 26561750;                 // m, GPS semi-major
  const inc = 55 * Math.PI / 180;
  const nPlanes = 6, per = 4;
  const plane = idx % nPlanes;
  const slot = Math.floor(idx / nPlanes);
  const raan = plane * (60 * Math.PI / 180);
  const mean0 = slot * (Math.PI / 2) + plane * (15 * Math.PI / 180);
  const n = 2 * Math.PI / GPS_PERIOD;      // mean motion, ~12h sidereal
  const M = mean0 + n * tSec;
  const u = M;                        // circular orbit
  const xOrb = a * Math.cos(u), yOrb = a * Math.sin(u);
  const x1 = xOrb, y1 = yOrb * Math.cos(inc), z1 = yOrb * Math.sin(inc);
  const cr = Math.cos(raan), sr = Math.sin(raan);
  return new Cesium.Cartesian3(x1 * cr - y1 * sr, x1 * sr + y1 * cr, z1);
}

function elevAzFrom(sat, rec) {
  const enu = Cesium.Transforms.eastNorthUpToFixedFrame(rec);
  const inv = Cesium.Matrix4.inverse(enu, new Cesium.Matrix4());
  const local = Cesium.Matrix4.multiplyByPoint(inv, sat, new Cesium.Cartesian3());
  const r = Math.sqrt(local.x * local.x + local.y * local.y);
  const el = Math.atan2(local.z, r) * 180 / Math.PI;
  const az = (Math.atan2(local.x, local.y) * 180 / Math.PI + 360) % 360;
  return { el, az };
}

function gpsOrbitRing(idx, n) {
  n = n || 96;
  const pts = [];
  for (let k = 0; k <= n; k++) pts.push(gpsSatEcef(idx, (k / n) * GPS_PERIOD));
  return pts;
}

function gpsTrackArc(idx, t0, dt, n) {
  n = n || 28;
  const pts = [];
  for (let k = 0; k <= n; k++) pts.push(gpsSatEcef(idx, t0 + dt * (k / n)));
  return pts;
}

function initGpsConstellation() {
  if (!viewer || cesGpsSats.length) return;
  for (let i = 0; i < 24; i++) {
    const id = 'G' + String(i + 1).padStart(2, '0');
    const now = Date.now() / 1000;
    const body = viewer.entities.add({
      position: gpsSatEcef(i, now),
      billboard: {
        image: '/sat.png',
        width: 56, height: 36,
        color: Cesium.Color.WHITE,
        alignedAxis: Cesium.Cartesian3.ZERO,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        scaleByDistance: new Cesium.NearFarScalar(1.0e5, 1.15, 2.6e7, 0.85)
      },
      label: {
        text: id, font: 'bold 12px monospace',
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK, outlineWidth: 3,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -28),
        disableDepthTestDistance: Number.POSITIVE_INFINITY, show: false
      }
    });
    const beam = viewer.entities.add({
      polyline: {
        positions: [gpsSatEcef(i, now), cesSafePos()],
        width: 1.5,
        material: new Cesium.PolylineGlowMaterialProperty({ glowPower: 0.15, color: Cesium.Color.LIME.withAlpha(0.0) }),
        arcType: Cesium.ArcType.NONE
      }
    });
    const orbit = viewer.entities.add({
      polyline: {
        positions: gpsOrbitRing(i),
        width: 1.2,
        material: Cesium.Color.SLATEGREY.withAlpha(0.25),
        arcType: Cesium.ArcType.NONE
      }
    });
    const trail = viewer.entities.add({
      polyline: {
        positions: gpsTrackArc(i, now - 1200, 1200),
        width: 2.0,
        material: Cesium.Color.SKYBLUE.withAlpha(0.35),
        arcType: Cesium.ArcType.NONE
      }
    });
    const traj = viewer.entities.add({
      polyline: {
        positions: gpsTrackArc(i, now, 2400),
        width: 3.0,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.18, color: Cesium.Color.CYAN.withAlpha(0.7)
        }),
        arcType: Cesium.ArcType.NONE
      }
    });
    cesGpsSats.push({ body, beam, orbit, trail, traj, idx: i, id });
  }
}

function updateGps3d(t) {
  lastGpsTel = t || lastGpsTel;
  const nLock = Math.max(0, Math.min(16, t && t.sats != null ? t.sats : 0));
  const rec = cesSafePos();
  const tSec = Date.now() / 1000;
  const ranked = [];
  cesGpsSats.forEach(s => {
    const p = gpsSatEcef(s.idx, tSec);
    const { el, az } = elevAzFrom(p, rec);
    s.el = el; s.az = az; s.pos = p;
    ranked.push(s);
    s.body.position = p;
  });
  ranked.sort((a, b) => b.el - a.el);
  const detail = (t && t.gps_sats) || [];
  ranked.forEach((s, k) => {
    const inView = s.el > 10;
    let locked = false;
    if (detail.length) {
      const d = detail.find(g => (g.prn % 32) === ((s.idx + 1) % 32));
      locked = !!(d && d.used && inView);
    } else {
      locked = inView && k < nLock;
    }
    s.inView = inView;
    s.locked = locked;
    const show = cesGpsView || inView;
    s.body.billboard.show = show;
    s.body.billboard.color = locked
      ? Cesium.Color.WHITE
      : (inView ? new Cesium.Color(0.72, 0.9, 1.0, 0.95) : new Cesium.Color(0.55, 0.55, 0.6, 0.45));
    s.body.billboard.width = locked ? 64 : 52;
    s.body.billboard.height = locked ? 40 : 33;
    s.body.label.text = locked ? (s.id + ' LOCK') : s.id;
    s.body.label.fillColor = locked ? Cesium.Color.LIME : Cesium.Color.WHITE;
    s.body.label.show = locked || (cesGpsView && inView);
    s.beam.polyline.positions = [s.pos, rec];
    s.beam.polyline.material = new Cesium.PolylineGlowMaterialProperty({
      glowPower: locked ? 0.25 : 0.08,
      color: (locked ? Cesium.Color.LIME : Cesium.Color.CYAN).withAlpha(inView ? (locked ? 0.85 : 0.25) : 0.0)
    });
    s.beam.polyline.width = locked ? 2.4 : 1.2;

    const showPath = cesGpsView || inView;
    s.orbit.polyline.show = cesGpsView || locked;
    s.orbit.polyline.material = (locked ? Cesium.Color.LIME : Cesium.Color.SLATEGREY)
      .withAlpha(cesGpsView ? (locked ? 0.45 : 0.18) : 0.35);
    s.orbit.polyline.width = locked ? 1.8 : 1.1;

    s.trail.polyline.show = showPath;
    s.trail.polyline.positions = gpsTrackArc(s.idx, tSec - 1500, 1500, 22);
    s.trail.polyline.material = (locked ? Cesium.Color.LIME : Cesium.Color.SKYBLUE)
      .withAlpha(inView || cesGpsView ? 0.4 : 0.15);

    s.traj.polyline.show = showPath;
    s.traj.polyline.positions = gpsTrackArc(s.idx, tSec, 2700, 28);
    s.traj.polyline.material = new Cesium.PolylineGlowMaterialProperty({
      glowPower: locked ? 0.28 : 0.12,
      color: (locked ? Cesium.Color.LIME : Cesium.Color.CYAN)
        .withAlpha(inView || cesGpsView ? (locked ? 0.95 : 0.55) : 0.12)
    });
    s.traj.polyline.width = locked ? 3.6 : 2.2;
  });
  drawSkyplot(ranked, t);
}

function drawSkyplot(ranked, t) {
  const c = $('gps-skyplot');
  if (!c) return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height, cx = W / 2, cy = H / 2, R = W / 2 - 8;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#071018';
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = '#1e3a4c'; ctx.lineWidth = 1;
  [0.33, 0.66, 1].forEach(f => { ctx.beginPath(); ctx.arc(cx, cy, R * f, 0, Math.PI * 2); ctx.stroke(); });
  ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
  ctx.fillStyle = '#64748b'; ctx.font = '10px sans-serif';
  ctx.fillText('N', cx - 3, cy - R + 12);
  ranked.forEach(s => {
    const el = Math.max(-10, s.el);
    const r = R * (1 - Math.max(0, Math.min(90, el)) / 90);
    const rad = s.az * Math.PI / 180;
    const x = cx + r * Math.sin(rad);
    const y = cy - r * Math.cos(rad);
    const col = s.locked ? '#22c55e' : (s.inView ? '#38bdf8' : '#475569');
    ctx.save();
    ctx.translate(x, y);
    ctx.fillStyle = col;
    ctx.fillRect(-6.5, -2.2, 4.2, 4.4);   // left wing
    ctx.fillRect(2.3, -2.2, 4.2, 4.4);    // right wing
    ctx.fillRect(-2.1, -2.8, 4.2, 5.6);   // bus
    ctx.restore();
  });
  const stats = $('gps-hud-stats');
  if (stats) {
    const fix = t && t.fix ? String(t.fix).replace('GPS_FIX_TYPE_', '') : 'NO_GPS';
    const acc = (t && t.h_acc_m != null && t.h_acc_m < 1e6) ? (t.h_acc_m.toFixed(1) + ' m') : '—';
    const lockN = ranked.filter(s => s.locked).length;
    const viewN = ranked.filter(s => s.inView).length;
    stats.innerHTML = `fix <b>${fix}</b><br>locked <b>${lockN}</b> / in view <b>${viewN}</b> / 24<br>hAcc ${acc}`;
  }
}

function init3D() {
  if (viewer) { lookAtLive(0.6); return; }
  const wrap = $('cesiumContainer');
  wrap.classList.add('on');          // must be visible *before* Viewer() or canvas is 0×0
  const ll = liveLL();
  cesDronePos = Cesium.Cartesian3.fromDegrees(ll[1], ll[0], aglMeters(droneRelAlt));
  try {
    Cesium.Ion.defaultAccessToken = undefined;
    const esri = new Cesium.UrlTemplateImageryProvider({
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      maximumLevel: 19 });
    viewer = new Cesium.Viewer('cesiumContainer', {
      baseLayer: new Cesium.ImageryLayer(esri),
      terrain: undefined,
      baseLayerPicker: false, geocoder: false, timeline: false,
      animation: false, sceneModePicker: false, homeButton: false,
      navigationHelpButton: false, infoBox: false, selectionIndicator: false,
      fullscreenButton: false, shouldAnimate: true
    });
    viewer.scene.globe.depthTestAgainstTerrain = false;
    viewer.scene.skyAtmosphere.show = true;
    viewer.scene.globe.enableLighting = true;
    viewer.clock.shouldAnimate = true;
    enableWorldTerrain();
    loadAreaBuildings(ll);
    initGpsConstellation();
  } catch (e) {
    console.error('Cesium init failed', e);
    $('mission-info').innerHTML = `<span style="color:#ef4444;">⚠️ 3D view failed: ${e.message}</span>`;
    return;
  }
  cesDrone = viewer.entities.add({
    show: new Cesium.CallbackProperty(() => currentDroneLL !== null, false),
    position: new Cesium.CallbackProperty(() => cesSafePos(), false),
    orientation: new Cesium.CallbackProperty(() => {
      try {
        return Cesium.Transforms.headingPitchRollQuaternion(
          cesSafePos(),
          new Cesium.HeadingPitchRoll(
            Cesium.Math.toRadians(cesDroneHdg),
            Cesium.Math.toRadians(cesPitch),
            Cesium.Math.toRadians(cesRoll)));
      } catch (e) {
        return Cesium.Quaternion.IDENTITY;
      }
    }, false),
    viewFrom: new Cesium.Cartesian3(-90, -70, 45),
    model: { uri: '/drone.glb', scale: 2.5, minimumPixelSize: 72, maximumScale: 80,
      heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND },
    label: { text: 'UAV', font: 'bold 12px sans-serif', pixelOffset: new Cesium.Cartesian2(0, -34),
      fillColor: Cesium.Color.YELLOW, showBackground: true, backgroundColor: Cesium.Color.BLACK.withAlpha(0.5),
      disableDepthTestDistance: Number.POSITIVE_INFINITY }
  });
  cesReady = true;
  viewer.resize();
  lookAtLive(0);
  requestAnimationFrame(() => { if (viewer) viewer.resize(); });
  setTimeout(() => { if (viewer) { viewer.resize(); lookAtLive(0.8); } }, 250);
}

function toggle3DFollow(force) {
  if (!viewer || !cesDrone) return;
  cesFollowing = (force !== undefined) ? force : !cesFollowing;
  viewer.trackedEntity = cesFollowing ? cesDrone : undefined;
  if (!cesFollowing) lookAtLive(0.6);
  const b = $('btn-3d-follow');
  if (b) {
    b.textContent = cesFollowing ? '🎥 Following (free cam)' : '🎥 Follow';
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
  document.body.classList.toggle('cesium-on', on);
  if (!on) { toggle3DFollow(false); cesGpsView = false; return; }
  init3D();
  setTimeout(() => { if (viewer) viewer.resize(); }, 120);
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
  document.body.classList.remove('cesium-on');
  cesGpsView = false;
  toggle3DFollow(false);
};
if ($('btn-gps-view')) {
  $('btn-gps-view').onclick = () => {
    if (!viewer) init3D();
    cesGpsView = !cesGpsView;
    $('btn-gps-view').classList.toggle('active', cesGpsView);
    if (cesGpsView) {
      toggle3DFollow(false);
      const ll = liveLL();
      const dest = Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 12_000_000);
      viewer.camera.flyTo({
        destination: dest,
        orientation: { heading: 0, pitch: Cesium.Math.toRadians(-90), roll: 0 },
        duration: 1.6
      });
    } else {
      lookAtLive(1.0);
    }
  };
}
if (new URLSearchParams(location.search).get('view') === '3d') {
  setTimeout(() => { if ($('btn-3d') && !$('btn-3d').classList.contains('active')) $('btn-3d').click(); }, 900);
}

// -------- real vs SITL layout (from /api/mode) --------
modeReady.then(mode => {
  if (mode === 'real') {
    $('btn-mode-twin').textContent = 'Aircraft 3D';
    $('btn-mode-twin').title = 'Live GPS and attitude from the real aircraft';
    $('btn-mode-plan').classList.add('sitl-only');
    $('btn-mode-manual').classList.add('sitl-only');
    const title = $('cam2-title');
    if (title) title.textContent = 'FRONT CAMERA · LIVE FPV';
    $('feed2-img').alt = 'Live front camera from the drone';
    const btn = $('btn-feed2');
    if (btn && !btn.classList.contains('active')) btn.click();
    locateOperator();
    setModeBtn('btn-mode-twin');
    setTimeout(() => {
      if ($('btn-3d') && !$('btn-3d').classList.contains('active')) $('btn-3d').click();
    }, 800);
  }
});

// Keep the 2D map on the aircraft (or the operator, until GPS locks).
let mapFollow = true;
let gcsMarker = null;
let lastFollowLL = null;
function setFollow(on) {
  mapFollow = !!on;
  const b = $('btn-locate');
  if (!b) return;
  b.classList.toggle('following', mapFollow);
  b.textContent = mapFollow ? '⌖ Following' : '⌖ Locate';
}
function recastMapAround(ll, forceZoom) {
  if (!ll) return;
  const want = 17;
  if (forceZoom || map.getZoom() < 15) map.setView(ll, want, { animate: true });
  else map.panTo(ll, { animate: true, duration: 0.35 });
  lastFollowLL = ll.slice();
}
function markOperator(ll) {
  if (currentDroneLL) return;
  if (!gcsMarker) {
    gcsMarker = L.circleMarker(ll, {
      radius: 6, color: '#a78bfa', fillColor: '#c4b5fd',
      fillOpacity: 1, weight: 2
    }).addTo(map).bindTooltip('GCS / operator');
  } else {
    gcsMarker.setLatLng(ll);
  }
  if (mapFollow) recastMapAround(ll, true);
}
function locateOperator() {
  const here = new URLSearchParams(location.search).get('here');
  if (here) {
    const p = here.split(',').map(Number);
    if (validLL(p[0], p[1])) { markOperator(p); return; }
  }
  if (!navigator.geolocation) return;
  navigator.geolocation.getCurrentPosition(pos => {
    markOperator([pos.coords.latitude, pos.coords.longitude]);
  }, () => {}, { enableHighAccuracy: true, timeout: 8000, maximumAge: 30000 });
}
map.on('dragstart', () => setFollow(false));
if ($('btn-locate')) {
  $('btn-locate').onclick = () => {
    setFollow(true);
    if (currentDroneLL) recastMapAround(currentDroneLL, true);
    else if (gcsMarker) recastMapAround([gcsMarker.getLatLng().lat, gcsMarker.getLatLng().lng], true);
    else locateOperator();
  };
  setFollow(true);
}

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
makeCamToggle('btn-feed2', 'feed2-img', '/video2.mjpeg');   // top-down / fpv (toggled)

// Resize the real camera using its lower-right corner, or use the size controls.
$('btn-camera-size').onclick = () => {
  const row = $('cam-row');
  row.style.width = ''; row.style.height = '';
  const large = row.classList.toggle('camera-large');
  $('btn-camera-size').textContent = large ? 'Restore' : 'Enlarge';
};
$('btn-camera-reset').onclick = () => {
  const row = $('cam-row');
  row.classList.remove('camera-large');
  row.style.width = ''; row.style.height = '';
  $('btn-camera-size').textContent = 'Enlarge';
};
$('btn-camera-fullscreen').onclick = async () => {
  try {
    if (document.fullscreenElement) await document.exitFullscreen();
    else await $('cam-fpv').requestFullscreen();
  } catch (e) {
    $('mission-info').textContent = 'Fullscreen unavailable; use Enlarge or drag the camera corner.';
  }
};
document.addEventListener('fullscreenchange', () => {
  $('btn-camera-fullscreen').textContent = document.fullscreenElement ? 'Exit ⛶' : '⛶';
});

// Camera health is independent of the MAVLink telemetry connection.
let fpvStreamError = false;
$('feed2-img').addEventListener('error', () => { fpvStreamError = true; });
$('feed2-img').addEventListener('load', () => { fpvStreamError = false; });
async function pollFrontCamera() {
  const status = $('fpv-status');
  const playing = $('btn-feed2').classList.contains('active');
  status.hidden = false;
  status.dataset.live = 'false';
  if (!playing) {
    status.textContent = 'Front camera paused';
  } else {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 3500);
    try {
      const response = await fetch('/api/camera/status', { signal: controller.signal, cache: 'no-store' });
      if (!response.ok) throw new Error('Camera status unavailable');
      const camera = await response.json();
      if (camera.fresh && !fpvStreamError) {
        status.textContent = '● LIVE · Front camera';
        status.dataset.live = 'true';
      } else {
        status.textContent = camera.bridge_connected
          ? 'Waiting for camera video — check the camera link'
          : 'Camera bridge unavailable — reconnecting…';
      }
      if (camera.bridge_connected && fpvStreamError && $('btn-feed2').classList.contains('active')) {
        $('feed2-img').src = '/video2.mjpeg?' + Date.now();
      }
    } catch (e) {
      status.textContent = 'Camera connection unavailable — reconnecting…';
    } finally { clearTimeout(timeout); }
  }
  setTimeout(pollFrontCamera, 2000);
}
modeReady.then(mode => { if (mode === 'real') pollFrontCamera(); });

// 2nd feed view toggle: Top-Down <-> FPV. Re-aims the same camera live (no
// restart, feed stays on :8092) via /api/command cam_view.
(function initCam2View() {
  let view = 'topdown';
  const btn = $('btn-feed2-view'), title = $('cam2-title');
  if (!btn) return;
  const apply = () => {
    title.textContent = view === 'topdown' ? 'TOP-DOWN CAM' : 'FPV CAM';
    btn.textContent = view === 'topdown' ? '🎥 FPV' : '⬇ Top-Down';
  };
  btn.onclick = () => {
    view = view === 'topdown' ? 'fpv' : 'topdown';
    apply();
    cmdPost({ cmd: 'cam_view', view }).catch(e => {
      $('mission-info').textContent = 'Camera command failed: ' + e.message;
    });
  };
  apply();
})();

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

// Body heading for the HUD, map arrow, and 3D model. Course-over-ground is a
// different angle (the nose yaws onto the path via MPC_YAW_MODE) and must not
// be drawn as heading. hdg is centidegrees/100 and is absent when unknown.
function aircraftHeading(t) {
  const hdg = t.hdg;
  if (typeof hdg === 'number' && isFinite(hdg) && hdg >= 0 && hdg < 360) return hdg;
  const yaw = t.yaw;
  if (typeof yaw === 'number' && isFinite(yaw)) return (yaw % 360 + 360) % 360;
  return null;
}

async function tick() {
  let t;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 5000);
  try {
    const response = await fetch('/api/telemetry', { signal: controller.signal, cache: 'no-store' });
    if (!response.ok) throw new Error('Telemetry unavailable');
    t = await response.json();
  } catch(e) { t = { connected:false }; }
  finally { clearTimeout(timeout); }
  if (!t.connected) t = { connected: false }; // never render cached flight state as live
  $('conn').textContent = t.connected ? 'LINK' : 'NO LINK'; $('conn').className = 'pill ' + (t.connected?'on':'off');
  const H = t.connected ? aircraftHeading(t) : null;
  $('mode').textContent  = (t.custom_mode!=null) ? modeName(t.custom_mode) : '—';
  if (t.custom_mode != null) refreshToggleBtn((t.custom_mode >>> 16) & 0xff);   // sync toggle label w/ live PX4 mode
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

  // ---- orientation compass (body heading, clockwise from north) ----
  if (H != null) {
    $('cmp-needle').setAttribute('transform', `rotate(${H} 60 60)`);
    $('cmp-hdg').textContent = Math.round(H) + '°';
  } else {
    $('cmp-hdg').textContent = '—';
  }

  // ---- attitude indicator (roll + pitch, artificial horizon) ----
  if (t.roll != null && t.pitch != null) {
    const px = Math.max(-40, Math.min(40, t.pitch)) * 1.1;   // px offset per degree
    $('adi-move').setAttribute('transform', `rotate(${-t.roll} 50 50) translate(0 ${px})`);
  }

  // ---- velocity gauge (left panel) ----
  const v = (gs!=null) ? gs : 0;
  $('vel-big').textContent = gs != null ? v.toFixed(1) : '—';
  $('vel-fill').style.width = Math.max(0, Math.min(100, (v / 20) * 100)) + '%';  // scale 0–20 m/s

  // ---- flight status panel ----
  updateStatus(t);

  // ---- UPDATE GLOBAL DRONE STATE ----
  // Real PX4 keeps publishing lat/lon with fix NO_GPS. Those numbers wander
  // and must not move the map or the 3D model.
  const hasPos = validLL(t.lat, t.lon) && (GCS_MODE !== 'real' || gpsHasFix(t));
  $('position-status').textContent = !t.connected
    ? 'No aircraft telemetry — position unavailable'
    : !hasPos ? 'Aircraft connected — waiting for GPS position'
    : `${GCS_MODE === 'real' ? 'Live aircraft GPS' : 'Simulator GPS'} · ${t.lat.toFixed(6)}, ${t.lon.toFixed(6)}`;
  const firstPosition = !currentDroneLL && hasPos;
  currentDroneLL = hasPos ? [t.lat, t.lon] : null;
  if (droneMarker) droneMarker.setOpacity(hasPos ? 1 : 0.35);
  if (validLL(t.home_lat, t.home_lon)) {
    BASE = { name: 'Home', lat: t.home_lat, lon: t.home_lon };
  } else if (hasPos && GCS_MODE === 'real' && BASE.name !== 'Home') {
    BASE = { name: 'Home', lat: t.lat, lon: t.lon };
  }
  droneIsArmed = !!t.armed;
  droneRelAlt = (t.rel_alt != null) ? t.rel_alt : 0;
  const alt = (t.rel_alt != null && t.rel_alt > 0) ? t.rel_alt : 0;

  if (hasPos) {
    const ll = [t.lat, t.lon];
    const iconHdg = (H != null) ? H : (t.hdg || 0);
    droneMarker ? (droneMarker.setLatLng(ll), droneMarker.setIcon(droneIcon(iconHdg)))
                : (droneMarker = L.marker(ll, { icon: droneIcon(iconHdg), zIndexOffset:1000 }).addTo(map));
    const firstLock = !lastFollowLL || !validLL(lastFollowLL[0], lastFollowLL[1]);
    if (mapFollow) {
      const jumped = firstLock || (lastFollowLL && haversineKm(lastFollowLL, ll) > 0.008);
      const offscreen = !map.getBounds().pad(-0.25).contains(ll);
      if (jumped || offscreen) recastMapAround(ll, firstLock);
      else lastFollowLL = ll.slice();
    }
    if (t.rel_alt!=null && t.rel_alt>0.5) {
      trailPts.push(ll); if (trailPts.length>2000) trailPts.shift();
      trail ? trail.setLatLngs(trailPts) : (trail = L.polyline(trailPts, { color:'#facc15', weight:2, opacity:.8 }).addTo(map));
      if (cesReady) { cesTrail.push(Cesium.Cartesian3.fromDegrees(t.lon, t.lat, alt)); if (cesTrail.length>2000) cesTrail.shift();
        if (!cesTrailEnt) cesTrailEnt = viewer.entities.add({ polyline: { positions: new Cesium.CallbackProperty(()=>cesTrail,false),
          width:2, material: Cesium.Color.YELLOW } }); }
    }
  }

  if (cesReady && window.Cesium) {
    try {
      if (hasPos) {
        cesDronePos = Cesium.Cartesian3.fromDegrees(t.lon, t.lat, aglMeters(t.rel_alt));
        loadAreaBuildings([t.lat, t.lon]);
        if (firstPosition) lookAtLive(0.8);
      }
      if (H != null) cesDroneHdg = H;
      if (t.roll != null) cesRoll = t.roll;
      if (t.pitch != null) cesPitch = t.pitch;
      updateGps3d(t);
    } catch (e) {}
  }
}
async function pollTelemetry() {
  try { await tick(); }
  catch (e) { console.error('Telemetry render failed:', e); }
  finally { setTimeout(pollTelemetry, 500); }
}
pollTelemetry();
