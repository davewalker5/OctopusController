/**
 * Strict JSON starting conditions and repeatable construction, matching scenario.py.
 *
 * Parsing, validation and scene construction are separate so an invalid file
 * cannot partially alter a running experiment. Geometry stays in world pixels;
 * file degree fields are converted into model radians at validation time.
 *
 * @typedef {object} ArmSetup
 * @property {import('./model.js').ArmParameters} parameters Fully resolved frozen settings.
 * @property {import('./geometry.js').Point|null} target Initial tip destination, or null for idle.
 *
 * @typedef {object} Scenario
 * @property {string} name Nonblank display name, retained exactly as supplied.
 * @property {import('./geometry.js').Point} body Fixed body centre.
 * @property {import('./sensing.js').WorldObject[]} objects Frozen starting object records.
 * @property {ArmSetup[]} arms Eight resolved starting assignments, in arm-index order.
 */

import { armParameters } from './model.js';
import { CentralController, DEFAULT_PARAMETERS } from './organism.js';
import { Environment, ObjectKind, worldObject } from './sensing.js';

/**
 * Keep a numeric token's value and its original integer-versus-float notation.
 *
 * JSON.parse turns 20, 20.0 and 2e1 into the same Number. Python distinguishes
 * integer notation during schema validation, so decoding must retain this fact.
 */
class JsonNumber {
  /**
   * Record a token already matched by the JSON number grammar.
   * @param {string} token Numeric source spelling; range/finite checks occur later.
   */
  constructor(token) {
    this.value = Number(token);
    this.integer = /^-?\d+$/.test(token);
  }
}
/**
 * Decode JSON while retaining numeric notation and rejecting duplicate keys.
 *
 * This is an intermediate representation: numbers are JsonNumber objects until
 * schema validation unwraps them. It is intentionally not a JSON.parse replacement
 * for general application use. No running simulation is accessed or modified.
 *
 * @param {string} text Decoded file contents; byte-level UTF-8 checks belong in app.js.
 * @returns {unknown} Parsed values with JsonNumber leaves and prototype-free mappings.
 * @throws {Error} If JSON syntax is invalid or any object repeats a decoded key.
 */
export function decodeJSON(text) {
  let index = 0;
  /**
   * Skip only the four whitespace characters allowed by the JSON grammar.
   * @returns {void} Advances the shared parser cursor without accepting a BOM.
   */
  const whitespace = () => {
    while (/[\x20\t\r\n]/.test(text[index] ?? '\0')) index++;
  };
  /**
   * Report a syntax error at the current one-based character position.
   * @returns {never}
   * @throws {Error} Always; parsing stops without publishing a partial scenario.
   */
  const fail = () => {
    throw new Error(`Invalid JSON near character ${index + 1}`);
  };
  /**
   * Consume a quoted string beginning at the cursor, delegating escape checks.
   * @returns {string} Decoded text; advances the cursor beyond the closing quote.
   * @throws {Error} If quotes, escapes or unescaped control characters are invalid.
   */
  function string() {
    const start = index++;
    while (index < text.length) {
      const char = text[index++];
      // Skip an escaped character when looking for the closing quote. Native
      // JSON.parse then validates the complete slice, including Unicode escapes.
      if (char === '\\') index++;
      else if (char === '"') return JSON.parse(text.slice(start, index));
    }
    return fail();
  }
  /**
   * Recursively consume one object, array, string, literal or numeric token.
   * @returns {unknown} Intermediate decoded value; the enclosing caller checks delimiters.
   * @throws {Error} If syntax or object-key uniqueness is violated.
   */
  function value() {
    whitespace();
    const char = text[index];
    if (char === '"') return string();
    if (char === '{' || char === '[') {
      // No prototype means keys such as __proto__ are ordinary data. Check
      // duplicates after string decoding, so escaped spellings cannot hide one.
      const object = char === '{',
        result = object ? Object.create(null) : [];
      const close = object ? '}' : ']';
      index++;
      whitespace();
      if (text[index] === close) {
        index++;
        return result;
      }
      while (index < text.length) {
        if (object) {
          whitespace();
          if (text[index] !== '"') fail();
          const key = string();
          whitespace();
          if (Object.hasOwn(result, key)) throw new Error(`Duplicate JSON field '${key}'`);
          if (text[index++] !== ':') fail();
          result[key] = value();
        } else result.push(value());
        whitespace();
        if (text[index] === close) {
          index++;
          return result;
        }
        if (text[index++] !== ',') fail();
      }
      return fail();
    }
    for (const [token, decoded] of [
      ['true', true],
      ['false', false],
      ['null', null],
    ]) {
      if (text.startsWith(token, index)) {
        index += token.length;
        return decoded;
      }
    }
    // Match JSON's number grammar, not JavaScript numeric syntax. Leading
    // zeroes, missing fraction digits and other suffixes remain unconsumed
    // and are rejected by the enclosing delimiter/end-of-document check.
    const number = text.slice(index).match(/^-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?/);
    if (!number) return fail();
    index += number[0].length;
    return new JsonNumber(number[0]);
  }
  const result = value();
  whitespace();
  // Reject a second value or trailing junk after the one permitted document.
  if (index !== text.length) fail();
  return result;
}
/**
 * Require a JSON object and reject misspelled or unsupported field names.
 * @param {unknown} value Intermediate decoded value.
 * @param {string[]} allowed Exact case-sensitive keys accepted at this level.
 * @param {string} location Field path for a useful validation error.
 * @returns {Object<string, unknown>} The existing mapping, without copying or mutation.
 * @throws {Error} If the value is not an object or contains an unknown field.
 */
function mapping(value, allowed, location) {
  if (!value || typeof value !== 'object' || Array.isArray(value) || value instanceof JsonNumber)
    throw new Error(`${location} must be an object`);
  const unknown = Object.keys(value)
    .filter((key) => !allowed.includes(key))
    .sort();
  if (unknown.length) throw new Error(`${location}: unknown field '${unknown[0]}'`);
  return value;
}
/**
 * Require a JSON array, distinguishing omission handled by callers from null.
 * @param {unknown} value Intermediate decoded value.
 * @param {string} location Field path for error reporting.
 * @returns {unknown[]} The existing array.
 * @throws {Error} If the value is not an array.
 */
function array(value, location) {
  if (!Array.isArray(value)) throw new Error(`${location} must be an array`);
  return value;
}
/**
 * Unwrap a finite JSON number without accepting booleans or numeric strings.
 * @param {unknown} value Intermediate decoded value, expected to be JsonNumber.
 * @param {string} location Field path for error reporting.
 * @returns {number} Finite numeric measurement.
 * @throws {Error} If the type is wrong or conversion overflowed to infinity.
 */
function number(value, location) {
  if (!(value instanceof JsonNumber) || !Number.isFinite(value.value))
    throw new Error(`${location} must be a finite number`);
  return value.value;
}
/**
 * Require integer source notation and an inclusive schema range.
 * @param {unknown} value Intermediate decoded value.
 * @param {number} low Minimum permitted integer.
 * @param {number} high Maximum permitted integer.
 * @param {string} location Field path for error reporting.
 * @returns {number} Accepted integer value.
 * @throws {Error} If notation, type or range is invalid, including 20.0 or 2e1.
 */
function integer(value, low, high, location) {
  if (!(value instanceof JsonNumber) || !value.integer || value.value < low || value.value > high)
    throw new Error(`${location} must be an integer between ${low} and ${high}`);
  return value.value;
}
/**
 * Validate exactly two finite coordinates and detach them from the decoded input.
 * @param {unknown} value Intermediate coordinate array.
 * @param {string} location Field path for error reporting.
 * @returns {import('./geometry.js').Point} Frozen world position in pixels.
 * @throws {Error} If the array length or either coordinate is invalid.
 */
function point(value, location) {
  const a = array(value, location);
  if (a.length !== 2) throw new Error(`${location} must contain exactly two coordinates`);
  return Object.freeze(a.map((v) => number(v, location)));
}
/**
 * Apply partial overrides to inherited settings, converting degrees once.
 * @param {unknown} value Intermediate parameter mapping.
 * @param {import('./model.js').ArmParameters} base Validated settings to inherit.
 * @param {string} location Field path for error reporting.
 * @returns {import('./model.js').ArmParameters} New frozen, validated model settings.
 * @throws {Error} If fields, counts, measurements or converted settings are invalid.
 */
function parameters(value, base, location) {
  // Human-readable file names use degrees and snake_case; internal numerical
  // rules use radians and camelCase. Keep that translation at this boundary.
  const fields = {
    segment_count: 'segmentCount',
    segment_length: 'segmentLength',
    maximum_bend_degrees: 'maximumBend',
    turning_speed_degrees: 'turningSpeed',
  };
  const data = mapping(value, Object.keys(fields), location),
    overrides = {};
  for (const [key, raw] of Object.entries(data)) {
    if (key === 'segment_count')
      overrides[fields[key]] = integer(raw, 2, 100, `${location}.${key}`);
    else {
      const n = number(raw, `${location}.${key}`);
      if (n <= 0 || (key === 'maximum_bend_degrees' && n > 180))
        throw new Error(
          `${location}.${key} must be positive${key === 'maximum_bend_degrees' ? ' and at most 180' : ''}`,
        );
      overrides[fields[key]] = key.endsWith('_degrees') ? n * (Math.PI / 180) : n;
    }
  }
  // Merge only supplied fields. Model validation runs again after conversion,
  // including rejecting tiny degree values that underflow to zero radians.
  return armParameters({ ...base, ...overrides });
}
/**
 * Validate all version-1 starting conditions before any scene is replaced.
 * @param {string} text JSON document, preserving numeric spelling for strict validation.
 * @returns {Scenario} Frozen initial conditions, independent of later runtime edits.
 * @throws {Error} If JSON, schema, parameters or a food reference is invalid.
 */
export function parseScenario(text) {
  const data = mapping(
    decodeJSON(text),
    ['version', 'name', 'body', 'defaults', 'food', 'obstacles', 'arms'],
    'Scenario',
  );
  // Presence checks below distinguish an omitted default from explicit null.
  // The version also requires integer notation: true and 1.0 are not version 1.
  integer(data.version, 1, 1, 'Scenario.version');
  const name = Object.hasOwn(data, 'name') ? data.name : 'Untitled scenario';
  if (typeof name !== 'string' || !name.trim())
    throw new Error('Scenario.name must be a non-empty string');
  const body = Object.hasOwn(data, 'body')
    ? point(data.body, 'Scenario.body')
    : Object.freeze([400, 460]);
  const defaults = parameters(
    Object.hasOwn(data, 'defaults') ? data.defaults : {},
    DEFAULT_PARAMETERS,
    'defaults',
  );
  // Resolve shared parameters once. Then read food before assignments so
  // named references work regardless of key order in the input document.
  // Named food labels differ from the numeric IDs used by sensors and grips.
  const objects = [],
    foodPositions = new Map();
  for (const [collection, kind] of [
    ['food', ObjectKind.FOOD],
    ['obstacles', ObjectKind.OBSTACLE],
  ]) {
    const entries = array(Object.hasOwn(data, collection) ? data[collection] : [], collection);
    entries.forEach((raw, index) => {
      const location = `${collection}[${index}]`;
      const obj = mapping(
        raw,
        kind === ObjectKind.FOOD ? ['position', 'radius', 'id'] : ['position', 'radius'],
        location,
      );
      const position = point(obj.position, `${location}.position`);
      const radius = Object.hasOwn(obj, 'radius') ? number(obj.radius, `${location}.radius`) : 14;
      if (radius <= 0) throw new Error(`${location}.radius must be positive`);
      if (Object.hasOwn(obj, 'id')) {
        if (typeof obj.id !== 'string' || !obj.id.trim())
          throw new Error(`${location}.id must be a non-empty string`);
        if (foodPositions.has(obj.id))
          throw new Error(`${location}: duplicate food id '${obj.id}'`);
        // Preserve exact case and whitespace in nonblank IDs. Trimming here
        // would silently change which later references identify the same food.
        foodPositions.set(obj.id, position);
      }
      // buildScenario adds in this same order, reproducing stable numeric IDs.
      objects.push(worldObject(objects.length + 1, position, kind, radius));
    });
  }
  // Omitted arms still exist, inherit defaults and start idle. Explicit entries
  // replace one slot, independently of their order in the file.
  const arms = Array.from({ length: 8 }, () =>
      Object.freeze({ parameters: defaults, target: null }),
    ),
    seen = new Set();
  for (const raw of array(Object.hasOwn(data, 'arms') ? data.arms : [], 'arms')) {
    const arm = mapping(raw, ['number', 'parameters', 'target'], 'Arm');
    const n = integer(arm.number, 1, 8, 'Arm.number');
    if (seen.has(n)) throw new Error(`Duplicate arm number ${n}`);
    seen.add(n);
    const p = parameters(
      Object.hasOwn(arm, 'parameters') ? arm.parameters : {},
      defaults,
      `Arm ${n}.parameters`,
    );
    let target = null;
    // An absent target means idle; null, an empty mapping, or both target
    // forms are errors rather than alternative spellings of that default.
    if (Object.hasOwn(arm, 'target')) {
      const assignment = mapping(arm.target, ['food', 'position'], `Arm ${n}.target`);
      if (Object.keys(assignment).length !== 1)
        throw new Error(`Arm ${n}.target must specify exactly one of food or position`);
      if (Object.hasOwn(assignment, 'food')) {
        if (typeof assignment.food !== 'string' || !foodPositions.has(assignment.food))
          throw new Error(`Arm ${n} references unknown food`);
        // Resolve only the initial centre. This does not reserve ownership or
        // track food as it moves; normal contacts still determine capture.
        target = foodPositions.get(assignment.food);
      } else target = point(assignment.position, `Arm ${n}.target.position`);
    }
    // File/UI arm numbers are one-based; internal array indices are zero-based.
    arms[n - 1] = Object.freeze({ parameters: p, target });
  }
  // Publish only fully validated, frozen starting conditions. Runtime poses,
  // grips and capture history are deliberately not part of the scenario format.
  return Object.freeze({
    name,
    body,
    objects: Object.freeze(objects),
    arms: Object.freeze(arms),
  });
}
/**
 * Build fresh mutable models from a validated immutable starting scenario.
 * @param {Scenario} scenario Complete output from parseScenario.
 * @returns {{central: CentralController, environment: Environment}} New scene with initial sensing.
 * @throws {Error} If model construction rejects the supplied geometry.
 */
export function buildScenario(scenario) {
  const central = new CentralController(scenario.body),
    environment = new Environment();
  for (const obj of scenario.objects) environment.add(obj.centre, obj.kind, obj.radius);
  // Rebuild instead of reusing previous runtime instances: moved objects,
  // in-flight requests, grips, timers and capture reports must not survive restart.
  central.arms.forEach((arm, i) => {
    arm.resetPose(scenario.arms[i].parameters);
    if (scenario.arms[i].target === null) arm.idle();
    else arm.assignReach(scenario.arms[i].target);
  });
  // A loaded setup is displayed paused, so contacts must already match the
  // visible initial pose without requiring a movement tick.
  central.refreshSensing(environment.objects);
  return { central, environment };
}
/**
 * Construct the original Python demonstration, including Arm 3's detour.
 * @returns {{central: CentralController, environment: Environment}} Fresh active arms and scene.
 * Playback state is chosen by Simulation/app.js, not by this scene factory.
 */
export function buildDemo() {
  const central = new CentralController([400, 460]),
    environment = new Environment();
  environment.add(central.arm(0).controller.target, ObjectKind.FOOD);
  const base = central.arm(2).controller.arm.base;
  // Put the right-facing arm's destination beyond a nearby obstacle, not
  // inside it, so this example can demonstrate local steering rather than
  // only the immediate blocked-target diagnosis. Arm indices are zero-based.
  central.assignReach(2, [base[0] + 100, base[1]]);
  environment.add([base[0] + 60, base[1] + 67], ObjectKind.OBSTACLE);
  central.refreshSensing(environment.objects);
  return { central, environment };
}
