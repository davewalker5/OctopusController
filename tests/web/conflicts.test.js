/** Arbitration boundaries and lifecycle, independently asserted beyond parity. */
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { ConflictResolver } from '../../explorer/js/conflicts.js';
import { CentralController } from '../../explorer/js/organism.js';
import { Grip } from '../../explorer/js/grasping.js';
import { Environment, ObjectKind, worldObject } from '../../explorer/js/sensing.js';
import { buildScenario, parseScenario } from '../../explorer/js/scenario.js';

/** Supply complete local intention records so absent fields never affect ties. */
const intent = (fields = {}) => ({
  active: true,
  target: [100, 100],
  heldObject: null,
  goalObject: 1,
  candidate: null,
  avoiding: false,
  discovery: null,
  ...fields,
});

test('claims use ownership, eligible incumbency, then arm number; safety overrides claims', () => {
  const resolver = new ConflictResolver(),
    task = intent();
  resolver.resolve([intent({ active: false }), task], new Set(), false, 0.1);
  let decisions = resolver.resolve([task, task], new Set(), false, 0.2);
  assert.deepEqual(
    decisions.map((d) => d.action),
    ['Yield', 'Reach'],
  );
  const count = resolver.conflictCount;
  resolver.resolve([task, task], new Set(), false, 0.3);
  assert.equal(resolver.conflictCount, count);
  decisions = resolver.resolve([intent({ heldObject: 1 }), task], new Set(), false, 0.4);
  assert.deepEqual(
    decisions.map((d) => d.action),
    ['Maintain grip', 'Yield'],
  );
  decisions = resolver.resolve([intent({ avoiding: true }), task], new Set(), false, 0.5);
  assert.deepEqual(
    decisions.map((d) => d.action),
    ['Avoid hazard', 'Reach'],
  );
  assert.equal(decisions[0].priority, 80);
  decisions = resolver.resolve([task, task], new Set([1]), false, 0.6);
  assert.deepEqual(
    decisions.map((d) => d.action),
    ['Reach', 'Withdraw'],
  );
  assert.equal(decisions[1].priority, 100);
});

test('critical severity upgrade reaches central control and releases a remote carry once', () => {
  const central = new CentralController([400, 460]),
    environment = new Environment();
  central.arms.forEach((a) => a.idle());
  const owner = central.arm(0),
    reporter = central.arm(4);
  environment.add(owner.controller.arm.points[3], ObjectKind.FOOD);
  owner.grip = Grip.attach(environment.objects[0], 3, owner.controller.arm.points);
  owner.assignReach([300, 250]);
  const hazard = environment.add(reporter.controller.arm.points[3], ObjectKind.HAZARD, 14, 0.9);
  central.step(1 / 120, environment);
  assert.ok(owner.grip);
  const before = [...environment.objects[0].centre];
  const objects = environment.objects.map((o) =>
    o.identifier === hazard
      ? worldObject(o.identifier, o.centre, o.kind, o.radius, 1, o.detectionRange)
      : o,
  );
  central.update(1 / 120, objects);
  assert.equal(central.goal, 'Withdraw all arms');
  assert.ok(central.arms.every((a) => a.withdrawalRequested));
  assert.equal(owner.grip, null);
  assert.equal(owner.active, false);
  assert.equal(central.conflicts.releaseCount, 1);
  assert.equal(central.abandonedGoalCount, 1);
  assert.deepEqual(environment.objects[0].centre, before);
  assert.ok(central.priority.reports.some((r) => r.signal.reason === 'Critical severity'));
  central.update(1 / 120, objects);
  assert.equal(central.conflicts.releaseCount, 1);
  environment.remove(hazard);
  for (let i = 0; i < 120; i++) central.step(1 / 120, environment);
  assert.equal(central.goal, 'Continue assignments');
  assert.equal(owner.active, false);
  assert.equal(owner.grip, null);
});

test('named food beats an incidental lower ID throughout the grasp dwell', () => {
  const central = new CentralController([400, 460]),
    environment = new Environment();
  central.arms.forEach((a) => a.idle());
  const arm = central.arm(0),
    position = arm.controller.arm.points[3];
  environment.add(position, ObjectKind.FOOD);
  const goal = environment.add(position, ObjectKind.FOOD);
  arm.assignReach(position, goal);
  for (let i = 0; i < 30; i++) central.step(1 / 120, environment);
  assert.equal(arm.grip?.objectId, goal);
  assert.equal(central.captureCount, 1);
  assert.ok(central.conflicts.events.some((e) => e.decision.conflict === 'Discovery versus task'));
});

test('yielding preserves the assignment and pause; released claims resume on the next tick', () => {
  const scenario = parseScenario(readFileSync('scenarios/competing-arms.json', 'utf8'));
  const { central, environment } = buildScenario(scenario),
    loser = central.arm(3);
  const points = structuredClone(loser.controller.arm.points),
    target = [...loser.controller.target];
  central.step(0, environment);
  assert.equal(central.conflicts.events.length, 0);
  for (let i = 0; i < 30; i++) central.step(1 / 120, environment);
  assert.ok(loser.yielding && loser.active);
  assert.deepEqual(loser.controller.arm.points, points);
  assert.deepEqual(loser.controller.target, target);
  central.arm(2).idle();
  central.step(1 / 120, environment);
  assert.equal(loser.yielding, false);
  assert.notDeepEqual(loser.controller.arm.points, points);
  assert.equal(buildScenario(scenario).central.conflicts.conflictCount, 0);
});
