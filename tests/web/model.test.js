import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { Arm, armParameters, ReachController } from '../../explorer/js/model.js';
import { CentralController } from '../../explorer/js/organism.js';
import { Environment, ObjectKind, detectContacts } from '../../explorer/js/sensing.js';
import { parseScenario, buildScenario } from '../../explorer/js/scenario.js';
import { Simulation } from '../../explorer/js/simulation.js';
import { safeMotion, steerAroundObstacles } from '../../explorer/js/avoidance.js';
import { Grip } from '../../explorer/js/grasping.js';
import { wrapAngle, requestStart, turnSegment } from '../../explorer/js/segment-control.js';

const python = process.env.PYTHON ?? 'venv/bin/python';
/** Run the real Python implementation rather than checked-in stale expected values. */
function reference(args = [], input = undefined) {
  const result = spawnSync(python, ['tests/web/reference.py', ...args], {
    encoding: 'utf8',
    env: { ...process.env, PYTHONPATH: 'src' },
    input,
    maxBuffer: 20 * 1024 * 1024,
  });
  assert.equal(result.status, 0, result.stderr || result.error?.message);
  return JSON.parse(result.stdout);
}
/** Compare continuous state with absolute tolerance, discrete state exactly. */
function close(actual, expected, path = 'root') {
  if (typeof expected === 'number') {
    assert.ok(
      Number.isFinite(actual) && Math.abs(actual - expected) < 1e-7,
      `${path}: ${actual} != ${expected}`,
    );
    return;
  }
  if (expected === null || typeof expected !== 'object') {
    assert.equal(actual, expected, path);
    return;
  }
  assert.deepEqual(Object.keys(actual), Object.keys(expected), path);
  for (const key of Object.keys(expected)) close(actual[key], expected[key], `${path}.${key}`);
}
function snapshot(central, environment) {
  return {
    seconds: central.simulationSeconds,
    count: central.captureCount,
    reports: central.captureReports.map((r) => [r.armIndex, r.objectId, r.simulationSeconds]),
    objects: environment.objects.map((o) => [o.identifier, o.centre, o.kind, o.radius]),
    arms: central.arms.map((a) => ({
      angles: a.controller.arm.angles,
      points: a.controller.arm.points,
      target: a.controller.target,
      active: a.active,
      state: a.state,
      status: a.controller.status,
      grip: a.grip ? [a.grip.objectId, a.grip.segmentIndex, a.grip.offset] : null,
      contacts: a.contacts.map((c) => [c.segmentIndex, c.objectId, c.kind, c.position]),
      requests: a.controller.requests.map((r) => [r.start, r.heading]),
    })),
  };
}
function trace(name) {
  let central, environment;
  if (name !== 'grasp')
    ({ central, environment } = buildScenario(
      parseScenario(readFileSync(`scenarios/${name}.json`, 'utf8')),
    ));
  else {
    central = new CentralController([400, 460]);
    environment = new Environment();
    central.arms.forEach((a) => a.idle());
    for (const a of central.arms.slice(0, 2)) {
      a.controller = new ReachController(
        new Arm([400, 460], armParameters({ segmentCount: 4, segmentLength: 12 })),
        [440, 470],
      );
      a.active = true;
    }
    environment.add(central.arm(0).controller.arm.points[1], ObjectKind.FOOD, 4);
    central.refreshSensing(environment.objects);
  }
  // Clone each snapshot: angle arrays are replaced, but reports and scene evolve.
  const result = [structuredClone(snapshot(central, environment))];
  for (let tick = 1; tick <= 600; tick++) {
    if (name === 'grasp') {
      if (tick === 31) central.arm(0).assignReach([425, 435]);
      else if (tick === 80) environment.add([430, 442], ObjectKind.OBSTACLE, 9);
      else if (tick === 100) environment.remove(2);
      else if (tick === 160) central.arm(0).retract();
      else if (tick === 220) {
        environment.move(1, [450, 420]);
        central.refreshSensing(environment.objects);
      } else if (tick === 300) environment.remove(1);
    }
    central.step(1 / 120, environment);
    if ([1, 29, 30, 31, 60, 81, 101, 161, 221, 301, 600].includes(tick))
      result.push(structuredClone(snapshot(central, environment)));
  }
  return result;
}
test('Python parity: scenarios, ownership, carrying, obstacle edits and retraction', () => {
  const expected = reference();
  for (const [name, states] of Object.entries(expected.traces)) close(trace(name), states, name);
  const environment = new Environment();
  environment.add([15, 5], ObjectKind.OBSTACLE, 2);
  const obstacles = environment.objects,
    arm = new Arm([0, 0], armParameters({ segmentCount: 3, segmentLength: 10 }));
  const food = new Environment();
  food.add(arm.points[1], ObjectKind.FOOD, 2);
  const grip = Grip.attach(food.objects[0], 1, arm.points);
  close(
    {
      wrap: [-100, -Math.PI, 0, Math.PI, 100].map(wrapAngle),
      steer: [
        [0, 0],
        [30, 10],
        [-20, 0],
      ].map((goal) => steerAroundObstacles([0, 0], goal, 10, obstacles)),
      safe: [
        [1.5, 0, 0],
        [0, 0, 0],
        [-0.1, 0.02, 0.02],
      ].map((after) => safeMotion([0, 0], [0, 0, 0], after, 10, obstacles)),
      request: [
        [[0, 0], null],
        [[-20, 3], -3],
        [[2, 1], 2],
      ].map(([goal, neighbour]) => {
        const r = requestStart([0, 0], goal, 0.2, neighbour, 10, 1);
        return [r.start, r.heading];
      }),
      turn: [
        [[0, 0], null],
        [[-20, 3], 1],
        [[2, 1], null],
      ].map(([goal, bend]) => turnSegment([0, 0], goal, 0.2, -0.1, bend, 1.5, 1 / 120)),
      payload: [[1.5, 0, 0], arm.angles, [-0.2, 0.06, 0.06]].map((after) =>
        grip.safeMotion([0, 0], arm.angles, after, 10, 2, obstacles),
      ),
    },
    expected.geometry,
    'geometry',
  );
});
test('strict file validation agrees with Python, including lexemes and duplicate keys', () => {
  const cases = [
    '{"version":1}',
    '{"version":1,"name":"  "}',
    '{"version":1.0}',
    '{"version":true}',
    '{"version":1e0}',
    '{"version":1,"version":1}',
    '{"version":1,"defaults":{"segment_count":20.0}}',
    '{"version":1,"defaults":{"segment_count":20}}',
    '{"version":1,"defaults":{"segment_count":true}}',
    '{"version":1,"defaults":{"turning_speed_degrees":1e-323}}',
    '{"version":1,"body":[1e309,0]}',
    '{"version":1,"body":[-3.5,2]}',
    '{"version":1,"body":null}',
    '{"version":1,"defaults":null}',
    '{"version":1,"food":[{"position":[1,2],"radius":0}]}',
    '{"version":1,"food":[{"position":[1,2],"id":"a"},{"position":[1,2],"id":"a"}]}',
    '{"version":1,"arms":[{"number":1,"target":{"food":"missing"}}]}',
    '{"version":1,"arms":[{"number":1},{"number":1}]}',
    '{"version":1,"arms":[{"number":1,"target":{}}]}',
    '{"version":1,"arms":[{"number":1,"target":null}]}',
    '{"version":1,"arms":[{"number":1.0}]}',
    '{"version":1,"food":[{"position":[1,2],"radius":4,"radius":5}]}',
    '{"version":1,"__proto__":{}}',
    '{"version":1,}',
    '{"version":01}',
    '{"version":1,"name":"A \\uD83D\\uDC19"}',
    '{"version":1,"defaults":{"segment_count":100,"segment_length":35},"food":[{"id":" a ","position":[-50,900]}],"arms":[{"number":8,"target":{"food":" a "},"parameters":{"maximum_bend_degrees":180}}]}',
    readFileSync('scenarios/two-targets.json', 'utf8'),
    readFileSync('scenarios/behaviour-showcase.json', 'utf8'),
  ];
  const expected = reference(['validate'], JSON.stringify(cases));
  cases.forEach((text, i) => {
    let scenario;
    try {
      scenario = parseScenario(text);
    } catch (error) {
      assert.equal(expected[i].valid, false, `${text}: ${error.message}`);
      return;
    }
    assert.equal(expected[i].valid, true, text);
    assert.equal(scenario.name, expected[i].name);
    const { central, environment } = buildScenario(scenario);
    close(snapshot(central, environment), expected[i].snapshot, `file ${i}`);
  });
});
test('bundled scenarios remain identical to the source files', () => {
  for (const name of ['two-targets', 'behaviour-showcase'])
    assert.equal(
      readFileSync(`explorer/scenarios/${name}.json`, 'utf8'),
      readFileSync(`scenarios/${name}.json`, 'utf8'),
    );
});
test('pause, full single step, frame cap and restart preserve their distinct semantics', () => {
  const scenario = parseScenario(readFileSync('scenarios/two-targets.json', 'utf8')),
    sim = new Simulation(scenario);
  const initial = structuredClone(snapshot(sim.central, sim.environment));
  sim.advance(5);
  close(snapshot(sim.central, sim.environment), initial);
  sim.singleStep();
  assert.equal(sim.central.simulationSeconds, 1 / 120);
  sim.setPaused(false);
  sim.advance(5);
  assert.ok(Math.abs(sim.central.simulationSeconds - 13 / 120) < 1e-12);
  const time = sim.central.simulationSeconds;
  sim.singleStep();
  assert.equal(sim.central.simulationSeconds, time);
  sim.restart();
  assert.equal(sim.paused, true);
  close(snapshot(sim.central, sim.environment), initial);
  const a = new Simulation(scenario),
    b = new Simulation(scenario);
  a.setPaused(false);
  b.setPaused(false);
  for (let i = 0; i < 120; i++) a.advance(1 / 120);
  for (let i = 0; i < 60; i++) b.advance(1 / 60);
  close(snapshot(a.central, a.environment), snapshot(b.central, b.environment));
});
test('paused edits cancel interrupted dwell without moving the pose or advancing time', () => {
  const sim = new Simulation(parseScenario('{"version":1}')),
    arm = sim.central.arm(0),
    point = arm.controller.arm.points[1];
  const id = sim.environment.add(point, ObjectKind.FOOD, 3);
  arm.assignReach(point);
  sim.singleStep();
  assert.ok(arm.candidate);
  const angles = [...arm.controller.arm.angles],
    seconds = sim.central.simulationSeconds;
  sim.environment.move(id, [0, 0]);
  sim.central.refreshSensing(sim.environment.objects);
  assert.equal(arm.candidate, null);
  assert.equal(arm.contactSeconds, 0);
  assert.equal(sim.central.simulationSeconds, seconds);
  assert.deepEqual(arm.controller.arm.angles, angles);
});
test('sensing includes exact contact boundaries and degenerate segments', () => {
  const env = new Environment();
  env.add([5, 5], ObjectKind.FOOD, 2);
  assert.equal(
    detectContacts(
      [
        [0, 0],
        [10, 0],
      ],
      env.objects,
    ).length,
    1,
  );
  assert.equal(
    detectContacts(
      [
        [0, 0],
        [0, 0],
      ],
      env.objects,
    ).length,
    0,
  );
  assert.throws(() =>
    detectContacts(
      [
        [0, 0],
        [10, 0],
      ],
      env.objects,
      -1,
    ),
  );
});
test('invalid model durations do not mutate any arm', () => {
  const sim = new Simulation();
  const before = snapshot(sim.central, sim.environment);
  for (const duration of [-1, NaN, Infinity])
    assert.throws(() => sim.central.step(duration, sim.environment));
  close(snapshot(sim.central, sim.environment), before);
});
