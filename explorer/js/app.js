/**
 * Browser application wiring: the HTML counterpart of input handling in app.py.
 *
 * Simulation owns fixed-step model advancement; SceneView draws it; this module
 * owns selection, forms, gestures and page lifecycle. Screen coordinates are
 * converted by the view, and human-facing degrees are converted at the form
 * boundary. Event handlers never calculate local joint movements themselves.
 */

import { clamp } from './geometry.js';
import { armParameters } from './model.js';
import { parseScenario } from './scenario.js';
import { Simulation } from './simulation.js';
import { SceneView, ARM_COLOURS } from './view.js';

/**
 * Look up a control in the fixed explorer page structure.
 * @param {string} id Element ID from index.html.
 * @returns {HTMLElement|null} Matching element; callers assume the required markup exists.
 */
const $ = (id) => document.getElementById(id);
const root = $('simulator'),
  canvas = $('world'),
  view = new SceneView(canvas);
// A module binding lets integration checks inspect the actual running model without a global hook.
export let simulation = new Simulation();
// Selection and drag ownership are presentation state, not arm assignments.
// Changing the selected arm must not change simulation update order or priority.
let selectedIndex = 0,
  selectedObject = null,
  tool = 'target';
let drag = null,
  previousTime = null,
  frame = null,
  lastReadout = 0,
  loadSequence = 0;
// Keep the last installed preset separate from a pending dropdown selection.
// Signatures avoid replacing object options and capture-log nodes every frame.
let committedPreset = 'demo',
  customScenario = null,
  objectSignature = '',
  captureSignature = '';
const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
// Only the first original demo may autoplay. Loaded setups and restarts pause;
// reduced motion also pauses this initial demonstration until the user chooses Play.
simulation.setPaused(reducedMotion.matches);
/**
 * Resolve the current UI selection to its live independently controlled arm.
 * @returns {import('./organism.js').ControlledArm} The selected arm instance.
 */
const selectedArm = () => simulation.central.arm(selectedIndex);
// Each entry holds [input ID, UI minimum, UI maximum, model-to-display factor].
// Model angles are radians; only the form uses degrees and degrees per second.
// These interactive limits are deliberately narrower than the scenario format.
const parameterFields = {
  segmentCount: ['segment-count', 2, 40, 1],
  segmentLength: ['segment-length', 6, 30, 1],
  maximumBend: ['maximum-bend', 5, 180, 180 / Math.PI],
  turningSpeed: ['turning-speed', (0.25 * 180) / Math.PI, (4 * 180) / Math.PI, 180 / Math.PI],
};
// Remember the actual displayed strings so Apply can distinguish edited fields
// from rounded display values and preserve wider file-supplied parameter ranges.
let displayedParameters = {};

/**
 * Publish a concise action/capture message through the polite live region.
 * @param {string} message Plain text suitable for assistive-technology announcement.
 * @returns {void} Does not announce the continually changing per-frame status.
 */
function announce(message) {
  $('announcement').textContent = message;
}
/**
 * Show a recoverable validation/loading failure without replacing the scene.
 * @param {Error} error Failure with a user-facing message, optionally including a filename.
 * @returns {void} Inserts plain text rather than interpreting file content as HTML.
 */
function showError(error) {
  $('error').textContent = error.message;
  $('error').hidden = false;
}
/**
 * Hide the preceding error after a successful action.
 * @returns {void} Leaves the running model untouched.
 */
function clearError() {
  $('error').hidden = true;
}
/**
 * Release a captured pointer and forget the active drag, if any.
 *
 * This shared, repeatable cleanup also handles cancellation and lost focus.
 * One gesture must never continue editing a newly selected arm or scene.
 * @returns {void} Does not undo the last accepted pointer edit.
 */
function endDrag() {
  if (drag && canvas.hasPointerCapture(drag.pointerId))
    canvas.releasePointerCapture(drag.pointerId);
  drag = null;
}
/**
 * Populate the selected arm's form after an explicit selection or model action.
 *
 * Display rounding must not round the actual parameters: the submit handler
 * compares against these strings before deciding which fields to replace.
 * @returns {void} Updates inputs and their comparison baseline, never model values.
 */
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
/**
 * Copy the selected object's current position into its coordinate form.
 * @returns {void} Leaves typed coordinates intact when there is no selected object.
 */
function syncObjectFields() {
  const obj = simulation.environment.objects.find((obj) => obj.identifier === selectedObject);
  if (obj) {
    $('object-x').value = Number(obj.centre[0].toFixed(2));
    $('object-y').value = Number(obj.centre[1].toFixed(2));
  }
}
/**
 * Refresh status, selection indicators and available actions from the live model.
 *
 * During animation this is throttled separately from canvas drawing. Editable
 * coordinate/parameter fields are deliberately not synchronised here, so a frame
 * cannot overwrite text while the user is typing. Stale object selection may be
 * cleared as UI housekeeping, but model state is never changed by this function.
 * @returns {void} Updates DOM readouts and announces newly observed captures.
 */
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
  // Movement does not change option labels. Rebuild only for changed identity
  // or kind membership, avoiding dropdown churn during carrying animations.
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
  // Use the cumulative count, not the bounded report array length, so new
  // captures remain visible after the model's 32-report history fills up.
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
/**
 * Render the current state with selection highlights, without advancing time.
 * @returns {void} Safe for ResizeObserver and explicit paused edits.
 */
function draw() {
  view.draw(simulation, selectedIndex, selectedObject);
}
/**
 * Reconcile sensing after an edit, then update text and drawing immediately.
 * @returns {void} May clear invalid grips/contacts but never moves an arm or advances dwell.
 */
function refresh() {
  simulation.central.refreshSensing(simulation.environment.objects);
  readout();
  draw();
}
/**
 * Change only the UI's inspected arm, ending any previous drag first.
 * @param {number} index Valid zero-based arm index supplied by the controls.
 * @returns {void} Refreshes fields and tool instructions without assigning a destination.
 */
function selectArm(index) {
  endDrag();
  selectedIndex = index;
  syncArmFields();
  setTool(tool);
  refresh();
}
/**
 * Choose how the next canvas gesture is interpreted and update its instructions.
 * @param {'target'|'object'|'Food'|'Obstacle'} value Destination, editing or placement tool.
 * @returns {void} Ends an existing drag; does not create an object or move a target.
 */
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
/**
 * Construct and install a complete paused scene, resetting presentation state.
 * @param {import('./scenario.js').Scenario|null} scenario Validated setup, or null for the demo.
 * @param {string} preset Selector value identifying this successfully installed setup.
 * @returns {void} Replaces the model only after construction has succeeded.
 * @throws {Error} If construction fails; callers report it while retaining the old scene.
 */
function installScenario(scenario, preset) {
  // Build before touching the live binding. A construction failure must leave
  // the existing experiment usable rather than exposing half a new scene.
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
/**
 * Load the selected bundled preset, retained local setup or original demo.
 *
 * A monotonically increasing sequence makes the latest request win, regardless
 * of network/file completion order. Failed current loads restore the committed
 * selector value and report an error; stale successes and failures are ignored.
 * @returns {Promise<void>} Resolves after installation, an ignored result or error display.
 */
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
    // Resolve against this module, not the website root, so a copied explorer
    // keeps working at a nested hosting URL without repository-relative paths.
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
/**
 * Restore the installed starting conditions, paused, without rereading any file.
 * @returns {void} Cancels pending loads/drags, resets selection and clears timing/readout caches.
 */
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
/**
 * Toggle playback and start the next frame with a fresh wall-clock timestamp.
 * @returns {void} Preserves poses and simulation time; discards partial frame accumulation.
 */
function playPause() {
  simulation.setPaused(!simulation.paused);
  previousTime = null;
  readout();
  announce(simulation.paused ? 'Paused.' : 'Playing.');
}
/**
 * Dispatch a selected-arm command through the existing model lifecycle.
 * @param {'reset'|'idle'|'release'|'retract'} action Action from a button or shortcut.
 * @returns {void} Refreshes forms/contacts; resetting one arm is not a scenario restart.
 */
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
/**
 * Read a coordinate form without imposing visible-world bounds.
 * @param {'target'|'object'} prefix Shared prefix of the two input IDs.
 * @returns {import('./geometry.js').Point} Finite logical world coordinates in pixels.
 * @throws {Error} If either input is empty, invalid or non-finite.
 */
function coordinates(prefix) {
  const point = [$(prefix + '-x').valueAsNumber, $(prefix + '-y').valueAsNumber];
  if (!point.every(Number.isFinite)) throw new Error('Enter two finite coordinates.');
  return point;
}
/**
 * Release ownership before removing the selected object from the scene.
 * @returns {void} No-op without a selection; refreshes contacts without advancing time.
 */
function deleteObject() {
  if (selectedObject === null) return;
  endDrag();
  simulation.central.releaseObject(selectedObject);
  simulation.environment.remove(selectedObject);
  selectedObject = null;
  refresh();
  announce('Object deleted.');
}

// Build controls once. Display labels are one-based, while option values and
// callbacks retain zero-based model indices. No arm is selected by its colour.
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
// Local files share the same load sequence as presets, but never leave the
// browser. Keep the last validated setup for switching back during this session.
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
    // Clearing the input lets choosing the same path again trigger change,
    // which is necessary to reload a JSON file edited outside the browser.
    $('scenario-file').value = '';
  }
});
// Both playback buttons call the same handler. Step uses the full simulation
// tick and is also guarded by Simulation, not just by the disabled button.
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
// Coordinate forms provide keyboard alternatives to canvas gestures. Prevent
// normal form navigation; report input failures without losing the experiment.
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
      // Unchanged displayed strings retain the exact original model value,
      // including valid scenario settings outside the narrower UI range.
      if ($(id).value === displayedParameters[key]) continue;
      const value = $(id).valueAsNumber;
      if (!Number.isFinite(value) || (key === 'segmentCount' && !Number.isInteger(value)))
        throw new Error('Enter valid numerical arm parameters.');
      overrides[key] = clamp(value, low, high) / factor;
    }
    // Validate the complete replacement before committing anything. Geometry
    // changes invalidate neighbour requests and grips, but speed alone does not.
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
    // The clicked submit button chooses add-food/add-obstacle/move. Without
    // an explicit operation, do not guess and accidentally create an object.
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
// Pointer events support mouse, pen and touch through one edit path. Only the
// primary button begins a gesture; a second pointer cannot steal an active drag.
canvas.addEventListener('pointerdown', (event) => {
  if (event.button !== 0 || drag) return;
  const point = view.point(event);
  // Placement is deliberately one-shot, then returns to target assignment.
  // This branch must not also retarget an arm to the newly created object.
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
  // Capture keeps a drag continuous outside the canvas. view.point clamps
  // pointer edits to visible-world bounds; coordinate forms can still go outside.
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
// Cleanup paths include cancellation and focus loss, not only a normal release.
// Otherwise returning to the page could resume a gesture the user already ended.
for (const name of ['pointerup', 'pointercancel', 'lostpointercapture'])
  canvas.addEventListener(name, endDrag);
window.addEventListener('blur', endDrag);
// Scope shortcuts to the explorer. Preserve text entry, modifier combinations,
// native Tab navigation and browser commands instead of copying desktop event rules.
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
/**
 * Present one browser frame and pass elapsed wall time to fixed-step playback.
 * @param {number} timestamp Monotonic requestAnimationFrame timestamp in milliseconds.
 * @returns {void} Schedules the next frame only while the document is visible.
 */
function animate(timestamp) {
  // This callback consumes the scheduled request. A null ID allows visibility
  // handling to resume exactly one animation chain rather than duplicate it.
  frame = null;
  if (document.hidden) return;
  if (previousTime !== null) simulation.advance((timestamp - previousTime) / 1000);
  previousTime = timestamp;
  draw();
  // Draw at the display rate, but refresh text at about 10 Hz. Live-region
  // announcements are further restricted to actions/new captures in readout().
  if (timestamp - lastReadout > 100) {
    readout();
    lastReadout = timestamp;
  }
  frame = requestAnimationFrame(animate);
}
// Hidden tabs do no animation work. Reset the timestamp on both transitions
// so time spent hidden is never interpreted as a long catch-up frame on return.
document.addEventListener('visibilitychange', () => {
  endDrag();
  previousTime = null;
  if (document.hidden) {
    if (frame !== null) cancelAnimationFrame(frame);
    frame = null;
  } else if (frame === null) frame = requestAnimationFrame(animate);
});
// A newly enabled reduced-motion preference pauses immediately. Turning that
// preference off does not silently resume an experiment the visitor paused.
reducedMotion.addEventListener('change', (event) => {
  if (event.matches) {
    simulation.setPaused(true);
    previousTime = null;
    readout();
  }
});
// Resize only redraws through the view; it must not advance a tick, rebuild
// arm geometry or overwrite typed form values. Seed readouts before the first frame.
new ResizeObserver(draw).observe(canvas);
syncArmFields();
setTool('target');
refresh();
frame = requestAnimationFrame(animate);
