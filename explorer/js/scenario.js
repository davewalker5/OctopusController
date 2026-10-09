import { armParameters } from './model.js';
import { CentralController, DEFAULT_PARAMETERS } from './organism.js';
import { Environment, ObjectKind, worldObject } from './sensing.js';

/** Preserve numeric lexemes and duplicate keys which JSON.parse would discard. */
class JsonNumber {
  constructor(token) {
    this.value = Number(token);
    this.integer = /^-?\d+$/.test(token);
  }
}
/** Strict recursive JSON reader. Strings delegate escape validation to JSON.parse. */
export function decodeJSON(text) {
  let index = 0;
  const whitespace = () => {
    while (/[\x20\t\r\n]/.test(text[index] ?? '\0')) index++;
  };
  const fail = () => {
    throw new Error(`Invalid JSON near character ${index + 1}`);
  };
  function string() {
    const start = index++;
    while (index < text.length) {
      const char = text[index++];
      if (char === '\\') index++;
      else if (char === '"') return JSON.parse(text.slice(start, index));
    }
    return fail();
  }
  function value() {
    whitespace();
    const char = text[index];
    if (char === '"') return string();
    if (char === '{' || char === '[') {
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
    const number = text.slice(index).match(/^-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?/);
    if (!number) return fail();
    index += number[0].length;
    return new JsonNumber(number[0]);
  }
  const result = value();
  whitespace();
  if (index !== text.length) fail();
  return result;
}
function mapping(value, allowed, location) {
  if (!value || typeof value !== 'object' || Array.isArray(value) || value instanceof JsonNumber)
    throw new Error(`${location} must be an object`);
  const unknown = Object.keys(value)
    .filter((key) => !allowed.includes(key))
    .sort();
  if (unknown.length) throw new Error(`${location}: unknown field '${unknown[0]}'`);
  return value;
}
function array(value, location) {
  if (!Array.isArray(value)) throw new Error(`${location} must be an array`);
  return value;
}
function number(value, location) {
  if (!(value instanceof JsonNumber) || !Number.isFinite(value.value))
    throw new Error(`${location} must be a finite number`);
  return value.value;
}
function integer(value, low, high, location) {
  if (!(value instanceof JsonNumber) || !value.integer || value.value < low || value.value > high)
    throw new Error(`${location} must be an integer between ${low} and ${high}`);
  return value.value;
}
function point(value, location) {
  const a = array(value, location);
  if (a.length !== 2) throw new Error(`${location} must contain exactly two coordinates`);
  return Object.freeze(a.map((v) => number(v, location)));
}
function parameters(value, base, location) {
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
  return armParameters({ ...base, ...overrides });
}
/** Validate an entire file before making any runtime scene. @throws {Error} */
export function parseScenario(text) {
  const data = mapping(
    decodeJSON(text),
    ['version', 'name', 'body', 'defaults', 'food', 'obstacles', 'arms'],
    'Scenario',
  );
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
        foodPositions.set(obj.id, position);
      }
      objects.push(worldObject(objects.length + 1, position, kind, radius));
    });
  }
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
    if (Object.hasOwn(arm, 'target')) {
      const assignment = mapping(arm.target, ['food', 'position'], `Arm ${n}.target`);
      if (Object.keys(assignment).length !== 1)
        throw new Error(`Arm ${n}.target must specify exactly one of food or position`);
      if (Object.hasOwn(assignment, 'food')) {
        if (typeof assignment.food !== 'string' || !foodPositions.has(assignment.food))
          throw new Error(`Arm ${n} references unknown food`);
        target = foodPositions.get(assignment.food);
      } else target = point(assignment.position, `Arm ${n}.target.position`);
    }
    arms[n - 1] = Object.freeze({ parameters: p, target });
  }
  return Object.freeze({
    name,
    body,
    objects: Object.freeze(objects),
    arms: Object.freeze(arms),
  });
}
/** Fresh mutable scene from immutable initial conditions; never reuse runtime state. */
export function buildScenario(scenario) {
  const central = new CentralController(scenario.body),
    environment = new Environment();
  for (const obj of scenario.objects) environment.add(obj.centre, obj.kind, obj.radius);
  central.arms.forEach((arm, i) => {
    arm.resetPose(scenario.arms[i].parameters);
    if (scenario.arms[i].target === null) arm.idle();
    else arm.assignReach(scenario.arms[i].target);
  });
  central.refreshSensing(environment.objects);
  return { central, environment };
}
/** Original Python demonstration, including Arm 3's obstacle detour. */
export function buildDemo() {
  const central = new CentralController([400, 460]),
    environment = new Environment();
  environment.add(central.arm(0).controller.target, ObjectKind.FOOD);
  const base = central.arm(2).controller.arm.base;
  central.assignReach(2, [base[0] + 100, base[1]]);
  environment.add([base[0] + 60, base[1] + 67], ObjectKind.OBSTACLE);
  central.refreshSensing(environment.objects);
  return { central, environment };
}
