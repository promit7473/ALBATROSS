/* Docked workspace and deliberate, telemetry-confirmed flight actions. */
(() => {
  const workspace = $('col-center');
  let telemetry = { connected: false }, token = null, uploadedRevision = -1;
  let controlGeneration = 0;
  let busy = false, pending = null, quickPick = false, completed = false, missionStarted = false;
  let layout = { x: 68, y: 52 };
  const eventHistory = [];
  let lastTelemetryAt = null, lastMode = null, lastConnected = null;
  function setDisplaySize(large) {
    document.body.classList.toggle('display-large', large);
    $('display-size').setAttribute('aria-pressed', String(large));
    $('display-size').textContent = large ? 'Normal text' : 'Large text';
    try { localStorage.setItem('albatross-display-large', String(large)); } catch (_) {}
    requestAnimationFrame(resizeViews);
  }
  try { setDisplaySize(localStorage.getItem('albatross-display-large') === 'true'); } catch (_) {}
  $('display-size').onclick = () => setDisplaySize(!document.body.classList.contains('display-large'));
  $('workspace-fullscreen').onclick = () => $('btn-camera-fullscreen').onclick();
  function logEvent(text) {
    eventHistory.unshift(new Date().toLocaleTimeString() + ' · ' + text);
    if (eventHistory.length > 50) eventHistory.length = 50;
    $('event-log').innerHTML = '';
    eventHistory.forEach(entry => {
      const row = document.createElement('li'); row.textContent = entry; $('event-log').appendChild(row);
    });
  }
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, Number(v)));
  try {
    const saved = JSON.parse(localStorage.getItem('albatross-layout') || '{}');
    if (Number.isFinite(saved.x)) layout.x = clamp(saved.x, 35, 80);
    if (Number.isFinite(saved.y)) layout.y = clamp(saved.y, 25, 75);
  } catch (_) { /* A blocked/full storage must not stop flight UI initialization. */ }
  function setOverlayMinimized(panel, button, label, minimized) {
    $(panel).classList.toggle('overlay-minimized', minimized);
    $(button).textContent = minimized ? '+' : '−';
    $(button).setAttribute('aria-expanded', String(!minimized));
    $(button).setAttribute('aria-label', `${minimized ? 'Restore' : 'Minimize'} ${label}`);
    $(button).setAttribute('title', `${minimized ? 'Restore' : 'Minimize'} ${label}`);
  }
  const overlayPanels = [
    ['map-toolbar', 'minimize-map-tools', 'map tools'],
    ['flight-overlay', 'minimize-flight-controls', 'flight controls'],
  ];
  overlayPanels.forEach(([panel, button, label]) => {
    $(button).onclick = () => setOverlayMinimized(panel, button, label, !$(panel).classList.contains('overlay-minimized'));
  });
  function resizeViews() {
    const mapWidth = $('map-cell').getBoundingClientRect().width;
    if (mapWidth > 0) $('dash').style.setProperty('--map-overlay-width', Math.max(160, Math.min(360, mapWidth - 72)) + 'px');
    map.invalidateSize();
    if (viewer && cesReady) viewer.resize();
  }
  function applyLayout() {
    workspace.style.setProperty('--split-x', layout.x + '%');
    workspace.style.setProperty('--split-y', layout.y + '%');
    $('split-main').setAttribute('aria-valuenow', Math.round(layout.x));
    $('split-cameras').setAttribute('aria-valuenow', Math.round(layout.y));
    try { localStorage.setItem('albatross-layout', JSON.stringify(layout)); } catch (_) {}
    requestAnimationFrame(resizeViews);
  }
  function bindSplitter(id, axis, container, lo, hi) {
    const handle = $(id);
    handle.addEventListener('pointerdown', e => {
      if (e.button !== 0) return;
      handle.setPointerCapture(e.pointerId);
      handle.dataset.dragging = 'true';
      e.preventDefault();
    });
    handle.addEventListener('pointermove', e => {
      if (handle.dataset.dragging !== 'true') return;
      const r = $(container).getBoundingClientRect();
      layout[axis] = clamp(axis === 'x' ? (e.clientX-r.left)/r.width*100 : (e.clientY-r.top)/r.height*100, lo, hi);
      applyLayout();
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(name => handle.addEventListener(name, () => { handle.dataset.dragging = 'false'; }));
    handle.addEventListener('keydown', e => {
      if (!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','Home'].includes(e.key)) return;
      e.preventDefault();
      layout[axis] = e.key === 'Home' ? (axis === 'x' ? 68 : 52)
        : clamp(layout[axis] + (['ArrowLeft','ArrowUp'].includes(e.key) ? -2 : 2), lo, hi);
      applyLayout();
    });
  }
  bindSplitter('split-main', 'x', 'col-center', 35, 80);
  bindSplitter('split-cameras', 'y', 'cam-row', 25, 75);
  if (window.ResizeObserver) new ResizeObserver(resizeViews).observe(workspace);
  function updateWindowControls() {
    const expandIcon = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5"/></svg>';
    const restoreIcon = '<svg viewBox="0 0 24 24" aria-hidden="true"><rect x="4" y="8" width="12" height="12" rx="1"/><path d="M8 8V4h12v12h-4"/></svg>';
    for (const [panel, id] of [['map','maximize-map'],['fpv','btn-camera-size'],['gazebo','maximize-gazebo']]) {
      const expanded = workspace.classList.contains('max-' + panel);
      $(id).innerHTML = expanded ? restoreIcon : expandIcon;
      $(id).setAttribute('aria-label', `${expanded ? 'Restore' : 'Expand'} ${panel}`);
      $(id).setAttribute('title', `${expanded ? 'Restore' : 'Expand'} ${panel}`);
      $(id).setAttribute('aria-pressed', String(expanded));
    }
    for (const panel of ['fpv','gazebo']) {
      const collapsed = workspace.classList.contains('collapsed-' + panel);
      $('collapse-' + panel).textContent = collapsed ? '+' : '−';
      $('collapse-' + panel).setAttribute('aria-label', `${collapsed ? 'Restore' : 'Collapse'} ${panel}`);
      $('collapse-' + panel).setAttribute('title', `${collapsed ? 'Restore' : 'Collapse'} ${panel}`);
      $('collapse-' + panel).setAttribute('aria-expanded', String(!collapsed));
    }
  }
  function maximize(panel) {
    const was = workspace.classList.contains('max-' + panel);
    ['map','fpv','gazebo'].forEach(p => workspace.classList.remove('max-' + p));
    if (!was) workspace.classList.add('max-' + panel);
    updateWindowControls();
    requestAnimationFrame(resizeViews);
  }
  $('maximize-map').onclick = () => maximize('map');
  $('btn-camera-size').onclick = () => maximize('fpv');
  $('maximize-gazebo').onclick = () => maximize('gazebo');
  for (const panel of ['fpv','gazebo']) $('collapse-' + panel).onclick = () => {
    workspace.classList.toggle('collapsed-' + panel);
    workspace.classList.remove('collapsed-' + (panel === 'fpv' ? 'gazebo' : 'fpv'));
    updateWindowControls();
    requestAnimationFrame(resizeViews);
  };
  $('reset-layout').onclick = () => {
    workspace.classList.remove('max-map','max-fpv','max-gazebo','collapsed-fpv','collapsed-gazebo');
    updateWindowControls();
    overlayPanels.forEach(([panel, button, label]) => setOverlayMinimized(panel, button, label, false));
    layout = {x:68,y:52}; applyLayout();
  };
  updateWindowControls();
  $('btn-camera-reset').onclick = $('reset-layout').onclick;
  $('btn-camera-fullscreen').onclick = async () => {
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await $('dash').requestFullscreen();
    } catch (_) { message('Fullscreen unavailable. Use Expand instead.'); }
  };
  function drawer(which, force) {
    const on = force === undefined ? !document.body.classList.contains('drawer-' + which) : force;
    for (const name of ['mission','status']) {
      const active = name === which && on;
      document.body.classList.toggle('drawer-' + name, active);
      $('open-' + name).setAttribute('aria-expanded', String(active));
    }
    if (on) $('close-' + which).focus();
  }
  for (const name of ['mission','status']) {
    $('open-' + name).onclick = () => drawer(name);
    $('close-' + name).onclick = () => { drawer(name, false); $('open-' + name).focus(); };
  }
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape') { drawer('mission', false); quickPick = false; $('map').style.cursor = ''; }
  });
  function setView(view) {
    if (view === 'map') $('btn-2d').click();
    else {
      if (!$('cesiumContainer').classList.contains('on')) $('btn-3d').click();
      if (!cesReady) { message('3D unavailable. Check the internet connection and browser WebGL support.'); return; }
      if ((view === 'gps') !== cesGpsView) $('btn-gps-view').click();
    }
    document.body.classList.toggle('gps-selected', view === 'gps');
    ['map','aircraft','gps'].forEach(v => {
      $('view-' + v).classList.toggle('selected', v === view);
      $('view-' + v).setAttribute('aria-pressed', String(v === view));
    });
    requestAnimationFrame(resizeViews);
  }
  for (const v of ['map','aircraft','gps']) $('view-' + v).onclick = () => setView(v);
  $('pick-destination').onclick = () => {
    quickPick = !quickPick; setAddMode(false); setView('map');
    $('map').style.cursor = quickPick ? 'crosshair' : '';
    $('pick-destination').classList.toggle('selected', quickPick);
    message(quickPick ? 'Click the map to choose a destination. This replaces the local draft only.' : 'Destination selection cancelled.');
  };
  map.on('click', async e => {
    if (!quickPick) return;
    quickPick = false; $('map').style.cursor = ''; $('pick-destination').classList.remove('selected');
    waypoints.forEach(w => w.marker.remove()); waypoints = [];
    addWaypoint(e.latlng);
    await $('btn-plan').onclick();
    if (routeLL) message('Route ready. Review the endpoint, altitude and return settings in Mission.');
    else message($('mission-info').textContent);
    render();
  });
  function message(text) { $('flight-message').textContent = text; }
  if (window.MutationObserver) new MutationObserver(() => {
    message($('mission-info').textContent); render();
  }).observe($('mission-info'), { childList:true, characterData:true, subtree:true });
  function invalidate() {
    controlGeneration++;
    token = null; uploadedRevision = -1;
    render();
  }
  let aircraftPreview = null;
  $('read-aircraft-mission').onclick = async () => {
    const button = $('read-aircraft-mission'); button.disabled = true;
    $('aircraft-mission-info').textContent = 'Reading stored aircraft mission…';
    try {
      const r = await apiPost('/api/mission/read', {});
      if (aircraftPreview) aircraftPreview.remove();
      const points = r.items.filter(i => ['MAV_CMD_NAV_TAKEOFF','MAV_CMD_NAV_WAYPOINT','MAV_CMD_NAV_LAND'].includes(i.command?.type || i.command))
        .filter(i => validLL(i.x / 1e7, i.y / 1e7)).map(i => [i.x / 1e7, i.y / 1e7]);
      aircraftPreview = points.length ? L.polyline(points, {color:'#fbbf24',weight:3,dashArray:'8 8'}).addTo(map) : null;
      $('aircraft-mission-info').textContent = `${r.items.length} aircraft items read. Amber route is read-only; local draft is unchanged. Use QGC to review other mission commands.`;
      message('Aircraft mission read. Amber = aircraft; blue = local draft. Review before replacing it.');
    } catch(e) { $('aircraft-mission-info').textContent = 'Read failed: ' + e.message; }
    finally { button.disabled = false; }
  };
  window.workspaceRouteChanged = invalidate;
  window.workspaceMissionUploaded = (r, revision) => {
    if (r.verified && r.mission_token && revision === routeRevision) {
      token = r.mission_token; uploadedRevision = revision; missionStarted = false; completed = false;
    }
    render();
  };
  for (const id of ['chk-rtl','rtl-alt']) $(id).addEventListener('change', () => { routeRevision++; invalidate(); });
  window.workspaceTelemetry = t => {
    if (lastConnected !== !!t.connected) {
      logEvent(t.connected ? 'Aircraft link restored' : 'Aircraft link unavailable');
      lastConnected = !!t.connected;
    }
    if (t.connected) {
      lastTelemetryAt = Date.now();
      if (t.custom_mode != null && t.custom_mode !== lastMode) {
        logEvent('Aircraft mode: ' + modeName(t.custom_mode)); lastMode = t.custom_mode;
      }
    }

    if (!t.connected) {
      token = null; uploadedRevision = -1;
      if (pending) { pending = null; message('Aircraft link lost. Command outcome unknown; check aircraft state after reconnecting.'); }
    } else if (token && t.mission_token !== token) {
      token = null;
      message('Aircraft mission changed or verification expired. Review and upload again.');
    }
    if (missionStarted && telemetry.armed && t.connected && t.armed === false) completed = true;
    telemetry = t;
    if (pending && pending.accepted && pending.matches(t)) {
      logEvent(pending.label + ' confirmed by telemetry'); message(pending.label + ' confirmed by aircraft telemetry.'); pending = null;
    }
    render();
  };
  function render() {
    const t = telemetry, live = !!t.connected;
    const sensors = $('sensor-health-list');
    sensors.innerHTML = '';
    if (!live || !Array.isArray(t.sensor_health)) sensors.textContent = live ? 'Sensor diagnostics unavailable; server update required.' : 'No live aircraft connection';
    else t.sensor_health.forEach(sensor => {
      const row = document.createElement('div'); row.classList.add('sensor-health-row');
      row.dataset.state = sensor.state;
      const name = document.createElement('strong'); name.textContent = sensor.name;
      const detail = document.createElement('span');
      detail.textContent = `${sensor.state.toUpperCase()} · ${sensor.detail}${sensor.age_s == null ? '' : ' · ' + sensor.age_s + 's ago'}`;
      row.appendChild(name); row.appendChild(detail); sensors.appendChild(row);
    });
    const main = ((t.custom_mode || 0) >>> 16) & 255, sub = ((t.custom_mode || 0) >>> 24) & 255;
    const flyingMission = live && t.armed && main === 4 && sub === 4;
    const working = busy || !!pending;
    $('session-mode').textContent = GCS_MODE === 'real' ? 'REAL AIRCRAFT' : GCS_MODE === 'sitl' ? 'SIMULATION' : 'MODE UNKNOWN';
    $('header-link').textContent = live ? '● AIRCRAFT LINK' : ('○ NO LINK' + (lastTelemetryAt ? ' · last seen ' + Math.floor((Date.now()-lastTelemetryAt)/1000) + 's ago' : ''));
    $('header-link').style.color = live ? 'var(--positive)' : 'var(--caution)';
    $('header-gps').textContent = live ? `${t.sats ?? '—'} · ${String(t.fix || 'No fix').replace('GPS_FIX_TYPE_', '').replace('3D_FIX', '3D fix').replace('2D_FIX', '2D fix')}` : 'GPS unavailable';
    $('header-rc').textContent = live && t.rc_available ? (t.rc_rssi == null ? 'RC input · RSSI unknown' : 'RC receiving') : 'RC unverified';
    $('confirmed-mode').textContent = live && t.custom_mode != null ? modeName(t.custom_mode) : 'No link';
    $('control-manual').classList.toggle('selected', live && main === 3);
    $('control-auto').classList.toggle('selected', flyingMission);
    $('draft-summary').textContent = waypoints.length ? `${waypoints.length} waypoint${waypoints.length === 1 ? '' : 's'} · ${routeLL ? 'route ready' : 'plan required'}` : 'No destination selected';
    let readiness = !live ? 'Connect aircraft to enable flight controls'
      : GCS_MODE === 'unknown' ? 'Operating mode unavailable'
      : boundaryDirty ? 'Save the edited boundary'
      : !routeLL ? 'Choose destination or plan a route'
      : !t.position_hold_available ? 'Waiting for a valid position estimate'
      : !t.armed ? 'Ready to arm · starting requires a second press'
      : 'Ready · GO verifies the aircraft mission before starting';
    if (flyingMission) readiness = `Mission active${t.mission_seq != null ? ' · item ' + (t.mission_seq + 1) + (t.mission_total ? '/' + t.mission_total : '') : ''}`;
    if (completed) readiness = 'Flight ended · aircraft disarmed';
    if (pending) readiness = pending.label + ' · waiting for telemetry';
    if (busy) readiness = 'Request in progress…';
    // This is GUI workflow readiness, not an autopilot pre-arm health certification.
    const canPrepare = live && GCS_MODE !== 'unknown' && !!routeLL && !boundaryDirty && !!t.position_hold_available;
    const verified = !!token && uploadedRevision === routeRevision;
    let state = !live ? 'offline' : 'blocked';
    let status = 'Not ready to fly';
    let reason = readiness;
    if (live && flyingMission) { state = 'active'; status = 'Mission active'; }
    else if (live && working) { state = 'pending'; status = 'Confirming…'; }
    else if (canPrepare && !t.armed) {
      state = 'pending'; status = 'Ready to arm';
      reason = 'GUI checks passed. Aircraft must accept arming; GO verifies the mission before start.';
    } else if (canPrepare && t.armed && verified) {
      state = 'ready'; status = 'Ready to fly';
      reason = 'Armed, position available, and displayed mission verified. Press GO to start.';
    } else if (canPrepare && t.armed) {
      state = 'pending'; status = 'Verify mission';
      reason = 'Aircraft armed. GO uploads and verifies the displayed mission before starting.';
    }
    $('header-readiness-card').dataset.state = state;
    $('header-readiness-card').setAttribute('title', reason);
    $('header-readiness-link').textContent = live ? 'Connected' : 'Disconnected';
    $('header-readiness').textContent = status;
    const icon = state === 'ready' || state === 'active' ? '<path d="m7 12 3 3 7-7"/>'
      : state === 'offline' ? '<path d="m8 8 8 8m0-8-8 8"/>'
      : '<path d="M12 7v6m0 4h.01"/>';
    $('header-readiness-icon').innerHTML = '<circle cx="12" cy="12" r="9"/>' + icon;
    $('flight-readiness').textContent = readiness;
    $('mission-sync').textContent = token ? 'Aircraft readback verified' : routeLL ? 'Draft · upload required' : 'No verified mission';
    $('flight-go').textContent = busy ? 'WAIT' : pending ? 'PENDING' : flyingMission ? 'FLYING' : !t.armed ? 'ARM' : token && missionStarted ? 'RESUME' : 'GO';
    $('flight-go').disabled = working || !live || GCS_MODE === 'unknown' || !routeLL || boundaryDirty || !t.position_hold_available || flyingMission;
    $('flight-hold').disabled = !live || !t.armed || !t.position_hold_available;
    $('flight-home').disabled = !live || !t.armed || !validLL(t.home_lat, t.home_lon);
    $('control-manual').disabled = !live || (GCS_MODE === 'real' && !t.rc_available);
    $('control-auto').disabled = working || !live || !t.armed || !token || flyingMission;
    $('land-here').disabled = !live || !t.armed;
    $('emergency-open').disabled = !live || !t.armed;
  }
  const modeMatches = (main, sub) => t => t.connected && ((t.custom_mode >>> 16) & 255) === main && (sub == null || ((t.custom_mode >>> 24) & 255) === sub);
  const predicates = {
    arm:t=>t.connected && t.armed === true, disarm:t=>t.connected && t.armed === false,
    emergency_stop:t=>t.connected && t.armed === false,
    start:modeMatches(4,4), resume:modeMatches(4,4), hold:modeMatches(4,3), rtl:modeMatches(4,5), land:modeMatches(4,6), manual:modeMatches(3),
  };
  const labels = { arm:'Arming', disarm:'Disarming', start:'Mission start', resume:'Mission resume', hold:'Hover', rtl:'Return home', land:'Landing', manual:'RC manual', emergency_stop:'Motor stop' };
  async function flight(action, extra = {}) {
    const priority = ['hold','rtl','land','manual','emergency_stop','disarm'].includes(action);
    if ((busy || pending) && !priority) return false;
    controlGeneration++;
    const request = { label:labels[action], matches:predicates[action], accepted:false };
    pending = request;
    logEvent(labels[action] + ' requested');
    busy = true; render();
    try {
      const r = await apiPost('/api/flight', {action, mission_token:token, ...extra});
      if (!r.accepted) throw new Error('Aircraft acknowledgement missing');
      if (pending !== request) return false;
      logEvent(labels[action] + ' acknowledged by aircraft');
      request.accepted = true;
      if (action === 'manual' && r.mode_main != null) request.matches = modeMatches(r.mode_main);
      if (action === 'start' || action === 'resume') missionStarted = true;
      message(labels[action] + ' accepted. Waiting for aircraft state.');
      setTimeout(() => {
        if (pending !== request) return;
        pending = null; message('State confirmation timed out. Check aircraft and QGC before retrying.'); render();
      }, 8000);
      return true;
    } catch(e) {
      if (pending === request) { pending = null; logEvent(labels[action] + ' failed: ' + e.message); message('Action failed: ' + e.message); }
      return false;
    } finally { if (pending === request || !pending) busy = false; render(); }
  }
  $('flight-go').onclick = async () => {
    if ($('flight-go').disabled) return;
    if (!telemetry.armed) {
      if (GCS_MODE === 'real' && !confirm('Arm the real aircraft? Motors may spin. Starting requires another press.')) return;
      await flight('arm'); return;
    }
    const generation = controlGeneration;
    if (!token || uploadedRevision !== routeRevision) {
      if (telemetry.rel_alt > 2 && !confirm('Replace the aircraft mission while airborne with this displayed draft?')) return;
      busy = true; render();
      try { await $('btn-upload').onclick(); }
      finally { busy = false; render(); }
      if (!token || uploadedRevision !== routeRevision) { message('Mission upload was not verified. Review Mission details before retrying.'); return; }
    }
    if (generation !== controlGeneration) { message('GO cancelled by another flight action.'); return; }
    await flight(missionStarted ? 'resume' : 'start');
  };
  $('flight-hold').onclick = () => flight('hold');
  $('flight-home').onclick = () => { if (confirm('Return to aircraft home using PX4 RTL settings?')) return flight('rtl'); };
  $('control-manual').onclick = () => flight('manual');
  $('control-auto').onclick = () => flight('resume');
  $('land-here').onclick = () => { if (confirm('Request landing at the current aircraft location?')) return flight('land'); };
  // Legacy drawer actions must use the same confirmed flight path.
  $('btn-arm').onclick = () => {
    const action = telemetry.armed ? 'disarm' : 'arm';
    if (confirm(`${action === 'arm' ? 'Arm' : 'Disarm'} the connected aircraft?`)) return flight(action);
  };
  $('btn-fly').onclick = () => flight('start');
  $('btn-proceed').onclick = () => flight('resume');
  $('btn-rtl').onclick = $('flight-home').onclick;
  $('btn-toggle-mode').onclick = () => flight(((telemetry.custom_mode >>> 16) & 255) === 4 ? 'manual' : 'resume');
  $('emergency-open').onclick = () => {
    $('emergency-confirm').checked = false; $('emergency-send').disabled = true;
    $('emergency-dialog').showModal(); $('emergency-cancel').focus();
  };
  $('emergency-confirm').onchange = () => { $('emergency-send').disabled = !$('emergency-confirm').checked; };
  $('emergency-cancel').onclick = () => $('emergency-dialog').close();
  $('emergency-send').onclick = async () => {
    if (!$('emergency-confirm').checked) return;
    $('emergency-dialog').close();
    await flight('emergency_stop', {confirmation:'STOP MOTORS'});
  };
  modeReady.then(mode => {
    if (mode === 'real') {
      $('cam-chase').querySelector('.cam-body').innerHTML = '<div class="panel-info">Gazebo simulation is not running in real-aircraft mode.</div>';
      $('btn-feed').disabled = true;
      $('cam2-title').textContent = 'LIVE FPV · FRONT CAMERA';
    } else if (mode === 'sitl') {
      $('cam2-title').textContent = 'FPV · SIM CAMERA';
      // Same second Gazebo camera, now starting in FPV rather than top-down.
      apiPost('/api/command', {cmd:'cam_view',view:'fpv'}).catch(e => message('Sim camera: ' + e.message));
    }
    render();
  });
  applyLayout(); render();
})();
