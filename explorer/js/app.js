import { clamp } from './geometry.js';
import { armParameters } from './model.js';
import { parseScenario } from './scenario.js';
import { Simulation } from './simulation.js';
import { SceneView, ARM_COLOURS } from './view.js';

const $ = (id) => document.getElementById(id);
const root = $('simulator'),
  canvas = $('world'),
  view = new SceneView(canvas);
// A module binding lets integration checks inspect the actual running model without a global hook.
export let simulation = new Simulation();
let selectedIndex = 0,
  selectedObject = null,
  tool = 'target';
let drag = null,
  previousTime = null,
  frame = null,
  lastReadout = 0,
  loadSequence = 0;
let committedPreset = 'demo',
  customScenario = null,
  objectSignature = '',
  captureSignature = '';
const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
simulation.setPaused(reducedMotion.matches);
const selectedArm = () => simulation.central.arm(selectedIndex);
const parameterFields = {
  segmentCount: ['segment-count', 2, 40, 1],
  segmentLength: ['segment-length', 6, 30, 1],
  maximumBend: ['maximum-bend', 5, 180, 180 / Math.PI],
  turningSpeed: ['turning-speed', (0.25 * 180) / Math.PI, (4 * 180) / Math.PI, 180 / Math.PI],
};
let displayedParameters = {};

/** Display a user-triggered status without continuously announcing animation. */
function announce(message) {
  $('announcement').textContent = message;
}
function showError(error) {
  $('error').textContent = error.message;
  $('error').hidden = false;
}
function clearError() {
  $('error').hidden = true;
}
/** End pointer ownership before selection, scene or focus changes. */
function endDrag() {
  if (drag && canvas.hasPointerCapture(drag.pointerId))
    canvas.releasePointerCapture(drag.pointerId);
  drag = null;
}
/** Keep editing fields stable during animation and update only on selection/action. */
function syncArmFields() {
  const arm = selectedArm(),
    p = arm.controller.arm.parameters;
  $('selected-arm').value = String(selectedIndex);
  for (const [key, [id, , , factor]] of Object.entries(parameterFields)) {
    const value = String(Number((p[key] * factor).toPrecision(6)));
    $(id).value = value;
    displayedParameters[key] = value;
  }
  const target = arm.grip && arm.carryGoal ? arm.carryGoal : arm.controller.target;
  $('target-x').value = Number(target[0].toFixed(2));
  $('target-y').value = Number(target[1].toFixed(2));
}
function syncObjectFields() {
  const obj = simulation.environment.objects.find((obj) => obj.identifier === selectedObject);
  if (obj) {
    $('object-x').value = Number(obj.centre[0].toFixed(2));
    $('object-y').value = Number(obj.centre[1].toFixed(2));
  }
}
/** Refresh text and controls; this never mutates the model. */
function readout() {
  const central = simulation.central,
    arm = selectedArm();
  $('play').textContent = simulation.paused ? 'Play' : 'Pause';
  $('scene-play').textContent = simulation.paused ? 'Play' : 'Pause';
  $('scene-play').setAttribute(
    'aria-label',
    simulation.paused ? 'Play simulation' : 'Pause simulation',
  );
  $('step').disabled = !simulation.paused;
  $('playback-state').textContent = simulation.paused ? 'Paused' : 'Running';
  $('clock').textContent =
    `${central.simulationSeconds.toFixed(2)} s · ${central.captureCount} captures`;
  $('arm-state').textContent = arm.state;
  const goal = arm.grip
    ? arm.active
      ? 'Payload destination'
      : 'Food held'
    : arm.controller.status;
  $('arm-detail').textContent =
    `${goal} · ${arm.contacts.length} contacts${arm.grip ? ` · food ${arm.grip.objectId}` : ` · ${arm.controller.distance.toFixed(1)} px to target`}`;
  $('destination-label').textContent = `${arm.grip ? 'Food' : 'Tip'} destination · pixels`;
  root.querySelector('[data-action="retract"]').disabled = !arm.grip;
  root.querySelector('[data-action="release"]').disabled = !arm.grip;
  central.arms.forEach((arm, i) => {
    const button = $(`arm-${i}`);
    button.setAttribute('aria-pressed', String(i === selectedIndex));
    button.querySelector('span').textContent = arm.state;
  });
  const objects = simulation.environment.objects;
  if (!objects.some((obj) => obj.identifier === selectedObject)) selectedObject = null;
  const signature = objects.map((obj) => `${obj.identifier}:${obj.kind}`).join('|');
  if (signature !== objectSignature) {
    $('selected-object').replaceChildren(new Option('No object selected', ''));
    objects.forEach((obj) =>
      $('selected-object').add(new Option(`${obj.kind} ${obj.identifier}`, String(obj.identifier))),
    );
    objectSignature = signature;
  }
  $('selected-object').value = selectedObject === null ? '' : String(selectedObject);
  $('move-object').disabled = $('delete-object').disabled = selectedObject === null;
  if (captureSignature !== String(central.captureCount)) {
    $('capture-log').replaceChildren();
    const reports = central.captureReports.slice(-5).reverse();
    for (const report of reports) {
      const li = document.createElement('li');
      li.textContent = `${report.simulationSeconds.toFixed(2)} s · Arm ${report.armIndex + 1} captured food ${report.objectId}`;
      $('capture-log').append(li);
    }
    if (!reports.length) {
      const li = document.createElement('li');
      li.textContent = 'No captures yet.';
      $('capture-log').append(li);
    }
    captureSignature = String(central.captureCount);
    if (reports.length)
      announce(`Arm ${reports[0].armIndex + 1} captured food ${reports[0].objectId}.`);
  }
}
function draw() {
  view.draw(simulation, selectedIndex, selectedObject);
}
function refresh() {
  simulation.central.refreshSensing(simulation.environment.objects);
  readout();
  draw();
}
function selectArm(index) {
  endDrag();
  selectedIndex = index;
  syncArmFields();
  setTool(tool);
  refresh();
}
function setTool(value) {
  endDrag();
  tool = value;
  $('tool').value = value;
  $('canvas-help').textContent =
    value === 'target'
      ? `Click or drag to set Arm ${selectedIndex + 1}’s destination. Use the coordinate controls for keyboard placement.`
      : value === 'object'
        ? 'Select and drag an object to move it. Moving held food releases its grip.'
        : `Click once to place ${value.toLowerCase()}. Then the destination tool resumes.`;
}
/** Commit a fully validated scene atomically; a failed read never replaces it. */
function installScenario(scenario, preset) {
  const next = new Simulation(scenario);
  endDrag();
  simulation = next;
  selectedIndex = 0;
  selectedObject = null;
  objectSignature = captureSignature = '';
  previousTime = null;
  committedPreset = preset;
  $('world-title').textContent = scenario?.name ?? 'Original demonstration';
  $('scenario').value = preset;
  setTool('target');
  syncArmFields();
  clearError();
  refresh();
  announce('Scenario loaded, paused.');
}
/** Ignore stale async loads when another file or preset was selected meanwhile. */
async function loadPreset() {
  const sequence = ++loadSequence,
    preset = $('scenario').value;
  try {
    if (preset === 'custom') {
      installScenario(customScenario, preset);
      return;
    }
    if (preset === 'demo') {
      installScenario(null, preset);
      return;
    }
    const response = await fetch(new URL(`../scenarios/${preset}.json`, import.meta.url));
    if (!response.ok) throw new Error(`Could not load scenario (${response.status}).`);
    const text = await response.text();
    if (sequence !== loadSequence) return;
    installScenario(parseScenario(text), preset);
  } catch (error) {
    if (sequence === loadSequence) {
      $('scenario').value = committedPreset;
      showError(error);
    }
  }
}
function restart() {
  ++loadSequence;
  endDrag();
  simulation.restart();
  $('scenario').value = committedPreset;
  selectedIndex = 0;
  selectedObject = null;
  previousTime = null;
  objectSignature = captureSignature = '';
  setTool('target');
  syncArmFields();
  clearError();
  refresh();
  announce('Starting scene restored, paused.');
}
function playPause() {
  simulation.setPaused(!simulation.paused);
  previousTime = null;
  readout();
  announce(simulation.paused ? 'Paused.' : 'Playing.');
}
function armAction(action) {
  endDrag();
  const arm = selectedArm();
  if (action === 'reset') {
    arm.resetPose();
    simulation.accumulator = 0;
  } else if (action === 'idle') arm.idle();
  else if (action === 'release') arm.release();
  else if (action === 'retract') arm.retract();
  syncArmFields();
  refresh();
  announce(`Arm ${selectedIndex + 1}: ${action === 'reset' ? 'pose reset' : action}.`);
}
function coordinates(prefix) {
  const point = [$(prefix + '-x').valueAsNumber, $(prefix + '-y').valueAsNumber];
  if (!point.every(Number.isFinite)) throw new Error('Enter two finite coordinates.');
  return point;
}
function deleteObject() {
  if (selectedObject === null) return;
  endDrag();
  simulation.central.releaseObject(selectedObject);
  simulation.environment.remove(selectedObject);
  selectedObject = null;
  refresh();
  announce('Object deleted.');
}

for (let i = 0; i < 8; i++) {
  $('selected-arm').add(new Option(`Arm ${i + 1}`, String(i)));
  const button = document.createElement('button');
  button.type = 'button';
  button.id = `arm-${i}`;
  button.className = 'arm-button';
  button.style.setProperty('--arm-colour', ARM_COLOURS[i]);
  const title = document.createElement('strong');
  title.textContent = `Arm ${i + 1}`;
  button.append(title, document.createElement('span'));
  button.addEventListener('click', () => selectArm(i));
  $('arm-roster').append(button);
}
$('selected-arm').addEventListener('change', () => selectArm(Number($('selected-arm').value)));
$('scenario').addEventListener('change', loadPreset);
$('scenario-file').addEventListener('change', async () => {
  const file = $('scenario-file').files[0];
  if (!file) return;
  const sequence = ++loadSequence;
  try {
    // File.text() replaces malformed UTF-8 silently; Python rejects it. Preserve
    // a byte-order mark too, so the strict JSON reader rejects it as Python does.
    const text = new TextDecoder('utf-8', { fatal: true, ignoreBOM: true }).decode(
      await file.arrayBuffer(),
    );
    if (sequence !== loadSequence) return;
    const scenario = parseScenario(text);
    if (!$('scenario').querySelector('[value="custom"]'))
      $('scenario').add(new Option('Loaded local file', 'custom'));
    customScenario = scenario;
    installScenario(scenario, 'custom');
  } catch (error) {
    if (sequence === loadSequence) showError(new Error(`${file.name}: ${error.message}`));
  } finally {
    $('scenario-file').value = '';
  }
});
$('play').addEventListener('click', playPause);
$('scene-play').addEventListener('click', playPause);
$('step').addEventListener('click', () => {
  simulation.singleStep();
  refresh();
});
$('restart').addEventListener('click', restart);
root
  .querySelectorAll('[data-action]')
  .forEach((button) => button.addEventListener('click', () => armAction(button.dataset.action)));
$('tool').addEventListener('change', () => setTool($('tool').value));
$('selected-object').addEventListener('change', () => {
  endDrag();
  selectedObject = $('selected-object').value ? Number($('selected-object').value) : null;
  syncObjectFields();
  setTool('object');
  refresh();
});
$('delete-object').addEventListener('click', deleteObject);
$('target-form').addEventListener('submit', (event) => {
  event.preventDefault();
  try {
    endDrag();
    selectedArm().assignReach(coordinates('target'));
    clearError();
    refresh();
    announce('Destination assigned.');
  } catch (error) {
    showError(error);
  }
});
$('parameters-form').addEventListener('submit', (event) => {
  event.preventDefault();
  try {
    const p = selectedArm().controller.arm.parameters,
      overrides = {};
    for (const [key, [id, low, high, factor]] of Object.entries(parameterFields)) {
      if ($(id).value === displayedParameters[key]) continue;
      const value = $(id).valueAsNumber;
      if (!Number.isFinite(value) || (key === 'segmentCount' && !Number.isInteger(value)))
        throw new Error('Enter valid numerical arm parameters.');
      overrides[key] = clamp(value, low, high) / factor;
    }
    const parameters = armParameters({ ...p, ...overrides });
    if (
      ['segmentCount', 'segmentLength', 'maximumBend'].some((key) => parameters[key] !== p[key])
    ) {
      selectedArm().resetPose(parameters);
      simulation.accumulator = 0;
    } else selectedArm().controller.arm.parameters = parameters;
    endDrag();
    syncArmFields();
    clearError();
    refresh();
    announce('Parameters applied within the interactive ranges.');
  } catch (error) {
    showError(error);
  }
});
$('object-form').addEventListener('submit', (event) => {
  event.preventDefault();
  try {
    const operation = event.submitter?.value;
    if (!operation) return;
    const point = coordinates('object');
    endDrag();
    if (operation === 'move') {
      if (selectedObject === null) return;
      simulation.central.releaseObject(selectedObject);
      simulation.environment.move(selectedObject, point);
    } else selectedObject = simulation.environment.add(point, operation);
    clearError();
    refresh();
    announce(operation === 'move' ? 'Object moved.' : `${operation} added.`);
  } catch (error) {
    showError(error);
  }
});
canvas.addEventListener('pointerdown', (event) => {
  if (event.button !== 0 || drag) return;
  const point = view.point(event);
  if (tool === 'Food' || tool === 'Obstacle') {
    selectedObject = simulation.environment.add(point, tool);
    syncObjectFields();
    setTool('target');
    refresh();
    return;
  }
  if (tool === 'object') {
    selectedObject = simulation.environment.pick(point);
    syncObjectFields();
  }
  if (tool === 'object' && selectedObject === null) {
    refresh();
    return;
  }
  drag = { pointerId: event.pointerId, kind: tool };
  canvas.setPointerCapture(event.pointerId);
  if (tool === 'target') {
    selectedArm().assignReach(point);
    syncArmFields();
  }
  refresh();
});
canvas.addEventListener('pointermove', (event) => {
  if (!drag || drag.pointerId !== event.pointerId) return;
  const point = view.point(event);
  if (drag.kind === 'target') {
    selectedArm().assignReach(point);
    syncArmFields();
  } else if (selectedObject !== null) {
    simulation.central.releaseObject(selectedObject);
    simulation.environment.move(selectedObject, point);
    syncObjectFields();
  }
  refresh();
});
for (const name of ['pointerup', 'pointercancel', 'lostpointercapture'])
  canvas.addEventListener(name, endDrag);
window.addEventListener('blur', endDrag);
root.addEventListener('keydown', (event) => {
  if (
    event.altKey ||
    event.ctrlKey ||
    event.metaKey ||
    event.repeat ||
    event.target.closest('input,select,textarea,[contenteditable="true"]')
  )
    return;
  // Space on a button retains native activation instead of also toggling playback.
  if (event.key === ' ' && event.target.closest('button,summary,a')) return;
  const key = event.key.toLowerCase();
  let handled = true;
  if (/^[1-8]$/.test(key)) selectArm(Number(key) - 1);
  else if (key === ' ') playPause();
  else if (key === 'n') {
    simulation.singleStep();
    refresh();
  } else if (key === 'd') restart();
  else if ({ r: 'reset', x: 'idle', t: 'retract', g: 'release' }[key])
    armAction({ r: 'reset', x: 'idle', t: 'retract', g: 'release' }[key]);
  else if (key === 'f' || key === 'o') setTool(key === 'f' ? 'Food' : 'Obstacle');
  else if (key === 'delete') deleteObject();
  else handled = false;
  if (handled) event.preventDefault();
});
/** Separate browser presentation timing from the fixed model timestep. */
function animate(timestamp) {
  frame = null;
  if (document.hidden) return;
  if (previousTime !== null) simulation.advance((timestamp - previousTime) / 1000);
  previousTime = timestamp;
  draw();
  if (timestamp - lastReadout > 100) {
    readout();
    lastReadout = timestamp;
  }
  frame = requestAnimationFrame(animate);
}
document.addEventListener('visibilitychange', () => {
  endDrag();
  previousTime = null;
  if (document.hidden) {
    if (frame !== null) cancelAnimationFrame(frame);
    frame = null;
  } else if (frame === null) frame = requestAnimationFrame(animate);
});
reducedMotion.addEventListener('change', (event) => {
  if (event.matches) {
    simulation.setPaused(true);
    previousTime = null;
    readout();
  }
});
new ResizeObserver(draw).observe(canvas);
syncArmFields();
setTool('target');
refresh();
frame = requestAnimationFrame(animate);
