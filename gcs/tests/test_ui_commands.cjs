// Exercise the actual UI helpers without a browser, network, or aircraft.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const test = require('node:test');
const source = fs.readFileSync(require('node:path').join(__dirname, '../web/app.js'), 'utf8');
const helpers = source.slice(source.indexOf('async function cmdPost('), source.indexOf("if ($('btn-mode-plan'))"));

function harness(fetch) {
  const elements = new Map();
  const $ = id => {
    if (!elements.has(id)) elements.set(id, { disabled: false, textContent: '' });
    return elements.get(id);
  };
  const context = vm.createContext({ fetch, $, console });
  vm.runInContext(helpers, context);
  return { context, $ };
}

test('failed server command reports error and restores button', async () => {
  const { context, $ } = harness(async () => new Response(JSON.stringify({ ok: false, error: 'No live heartbeat' }), { status: 503 }));
  const result = await vm.runInContext("sendFlightCommand('btn-fly', 'start_mission', 'Mission start request sent')", context);
  assert.equal(result, false);
  assert.equal($('btn-fly').disabled, false);
  assert.match($('mission-info').textContent, /No live heartbeat/);
});

test('successful transport is described as a request awaiting confirmation', async () => {
  const { context, $ } = harness(async () => new Response(JSON.stringify({ ok: true })));
  const result = await vm.runInContext("sendFlightCommand('btn-fly', 'start_mission', 'Mission start request sent')", context);
  assert.equal(result, true);
  assert.match($('mission-info').textContent, /check live flight status for confirmation/);
});

test('network loss reports failure without leaving button disabled', async () => {
  const { context, $ } = harness(async () => { throw new Error('Network disconnected'); });
  await vm.runInContext("sendFlightCommand('btn-rtl', 'rtl', 'RTL request sent')", context);
  assert.equal($('btn-rtl').disabled, false);
  assert.match($('mission-info').textContent, /Network disconnected/);
});

test('telemetry scheduler waits for the current poll to finish', async () => {
  const scheduler = source.slice(source.indexOf('async function pollTelemetry()'), source.lastIndexOf('pollTelemetry();'));
  let finish;
  const scheduled = [];
  const context = vm.createContext({
    tick: () => new Promise(resolve => { finish = resolve; }),
    setTimeout: (fn, ms) => scheduled.push({ fn, ms }), console,
  });
  vm.runInContext(scheduler, context);
  const pending = vm.runInContext('pollTelemetry()', context);
  assert.equal(scheduled.length, 0);
  finish();
  await pending;
  assert.equal(scheduled.length, 1);
  assert.equal(scheduled[0].ms, 500);
});
