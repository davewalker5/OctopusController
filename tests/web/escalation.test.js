/** Priority signalling boundaries and lifecycle, independent of browser rendering. */
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  escalationPolicy,
  HazardSignaller,
  PriorityReceiver,
} from '../../explorer/js/escalation.js';
import { worldObject, ObjectKind } from '../../explorer/js/sensing.js';
import { parseScenario, buildScenario } from '../../explorer/js/scenario.js';

const points = [
  [0, 0],
  [20, 0],
  [40, 0],
];
const hazard = (severity, centre = [20, 5], identifier = 1) =>
  worldObject(identifier, centre, ObjectKind.HAZARD, 2, severity, 10);

test('local-only, report and withdrawal thresholds include exact boundaries', () => {
  for (const [severity, priority] of [
    [0.399, 0],
    [0.4, 50],
    [0.749, 50],
    [0.75, 100],
  ]) {
    const signaller = new HazardSignaller(escalationPolicy());
    const messages = signaller.update(points, [hazard(severity)], 0.1);
    assert.equal(messages[0]?.priority ?? 0, priority);
    assert.equal(signaller.update(points, [hazard(severity)], 0.1).length, 0);
  }
});

test('central knowledge is last-reported local knowledge, with upgrade and clearance', () => {
  const local = new HazardSignaller(escalationPolicy()),
    receiver = new PriorityReceiver();
  receiver.receive(2, local.update(points, [hazard(0.5)], 0.1), 0.1, false);
  assert.equal(receiver.goal, 'Continue assignments');
  assert.equal(local.update(points, [hazard(0.5, [21, 6])], 0.1).length, 0);
  assert.deepEqual(receiver.known.get('2:1').signal.position, [20, 5]);
  assert.ok(receiver.receive(2, local.update(points, [hazard(0.9)], 0.1), 0.3, false));
  assert.equal(receiver.interventionCount, 1);
  assert.equal(local.update(points, [], 0.59).length, 0);
  assert.equal(receiver.receive(2, local.update(points, [], 0.01), 0.9, false), false);
  assert.equal(receiver.known.size, 0);
  assert.equal(receiver.reportCount, 2);
  assert.equal(receiver.escalationCount, 1);
});

test('sustained actual medium exposure promotes while low severity stays local', () => {
  for (const severity of [0.2, 0.5]) {
    const local = new HazardSignaller(escalationPolicy({ exposure_seconds: 0.3 }));
    const reports = [];
    for (let i = 0; i < 3; i++)
      reports.push(...local.update(points, [hazard(severity, [20, 0])], 0.1));
    assert.equal(reports.length, severity === 0.2 ? 0 : 2);
    if (reports.length) assert.equal(reports.at(-1).reason, 'Sustained exposure');
  }
});

test('pause, clear and restart preserve the assignment and reset only at restart', () => {
  const scenario = parseScenario(readFileSync('scenarios/hazard-escalation.json', 'utf8'));
  const { central, environment } = buildScenario(scenario),
    arm = central.arm(2);
  const target = [...arm.controller.target];
  central.refreshSensing(environment.objects);
  central.step(0, environment);
  assert.equal(central.priority.known.size, 0);
  for (let i = 0; i < 70; i++) central.step(1 / 120, environment);
  assert.equal(arm.state, 'Withdrawing');
  assert.deepEqual(arm.controller.target, target);
  environment.remove(2);
  for (let i = 0; i < 1500; i++) central.step(1 / 120, environment);
  assert.equal(central.priority.goal, 'Continue assignments');
  assert.ok(arm.controller.distance <= 2);
  assert.deepEqual(arm.controller.target, target);
  assert.equal(buildScenario(scenario).central.priority.reportCount, 0);
});

test('invalid policies and durations are rejected without sending reports', () => {
  for (const settings of [
    { report_severity: 0 },
    { withdrawal_severity: 1.1 },
    { report_severity: 0.8 },
    { report_severity: true },
    { clear_seconds: 0 },
    { exposure_seconds: -1 },
    { exposure_seconds: null },
    { unknown: 1 },
  ]) {
    assert.throws(() => parseScenario(JSON.stringify({ version: 1, escalation: settings })));
  }
  const local = new HazardSignaller(escalationPolicy());
  for (const seconds of [-1, NaN, Infinity])
    assert.throws(() => local.update(points, [hazard(1)], seconds));
  assert.equal(local.notification, 'None');
});
