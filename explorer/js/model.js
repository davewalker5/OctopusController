import { distance, posePoints, validatePoint, validateDuration } from './geometry.js';
import { wrapAngle, requestStart, turnSegment } from './segment-control.js';
import { ARM_CLEARANCE, safeMotion, steerAroundObstacles } from './avoidance.js';
import { ObjectKind, detectContacts } from './sensing.js';
/** Validate and freeze the geometry and motion limits for one arm. */
export function armParameters(overrides = {}) {
  const p = {
    segmentCount: 20,
    segmentLength: 18,
    maximumBend: Math.PI / 3,
    turningSpeed: 1.5,
    ...overrides,
  };
  if (!Number.isInteger(p.segmentCount) || p.segmentCount < 2 || p.segmentCount > 100)
    throw new Error('Segment count must be an integer between 2 and 100');
  for (const key of ['segmentLength', 'maximumBend', 'turningSpeed'])
    if (!Number.isFinite(p[key]) || p[key] <= 0)
      throw new Error('Length, bend and speed must be finite and positive');
  if (p.maximumBend > Math.PI) throw new Error('Maximum bend must not exceed 180 degrees');
  return Object.freeze(p);
}
export class Arm {
  /** Seed a gentle curve to avoid an arbitrary fold at a straight inward reach. */
  constructor(base, parameters = armParameters(), heading = 0) {
    validatePoint(base);
    if (!Number.isFinite(heading)) throw new Error('Initial heading must be finite');
    this.base = [...base];
    this.parameters = parameters;
    this.angles = [
      wrapAngle(heading),
      ...Array(parameters.segmentCount - 1).fill(Math.min(0.06, parameters.maximumBend)),
    ];
  }
  get points() {
    return posePoints(this.base, this.angles, this.parameters.segmentLength);
  }
}
export class ReachController {
  /** Seed requests from the initial pose; retargeting retains messages in flight. */
  constructor(arm, target) {
    this.arm = arm;
    const points = arm.points;
    let heading = 0;
    this.requests = arm.angles.map((angle, i) => {
      heading += angle;
      return { start: points[i], heading };
    });
    this.setTarget(target);
  }
  setTarget(target) {
    validatePoint(target);
    this.target = [...target];
    this.blocked = false;
    this.avoiding = false;
  }
  get distance() {
    return distance(this.arm.points.at(-1), this.target);
  }
  get status() {
    if (this.blocked) return 'Blocked';
    if (this.avoiding) return 'Avoiding';
    if (this.distance <= 2) return 'Reached';
    const p = this.arm.parameters;
    return distance(this.target, this.arm.base) > p.segmentCount * p.segmentLength
      ? 'Beyond reach'
      : 'Reaching';
  }
  /** Exchange old neighbour requests, propose bounded turns, then check the sweep. */
  update(seconds, objects = []) {
    validateDuration(seconds);
    if (seconds === 0) return;
    const obstacles = objects.filter((obj) => obj.kind === ObjectKind.OBSTACLE);
    this.blocked = this.avoiding = false;
    if (obstacles.some((obj) => distance(this.target, obj.centre) < obj.radius + ARM_CLEARANCE)) {
      this.blocked = true;
      return;
    }
    if (this.distance <= 2 && !detectContacts(this.arm.points, obstacles).length) return;
    const p = this.arm.parameters,
      positions = this.arm.points;
    // Read all old requests first: a target travels one neighbour per tick.
    const goals = [...this.requests.slice(1).map((request) => request.start), this.target].map(
      (goal, i) => {
        const [steered, changed] = steerAroundObstacles(
          positions[i],
          goal,
          p.segmentLength,
          obstacles,
        );
        this.avoiding ||= changed;
        return steered;
      },
    );
    let heading = 0;
    this.requests = this.arm.angles.map((angle, i) => {
      heading += angle;
      return requestStart(
        positions[i],
        goals[i],
        heading,
        this.requests[i + 1]?.heading ?? null,
        p.segmentLength,
        p.maximumBend,
      );
    });
    const before = [...this.arm.angles],
      proposed = [];
    let start = this.arm.base,
      parentHeading = 0;
    for (let i = 0; i < goals.length; i++) {
      const [goal, changed] = steerAroundObstacles(start, goals[i], p.segmentLength, obstacles);
      this.avoiding ||= changed;
      const angle = turnSegment(
        start,
        goal,
        parentHeading,
        before[i],
        i ? p.maximumBend : null,
        p.turningSpeed,
        seconds,
      );
      proposed.push(angle);
      parentHeading += angle;
      start = [
        start[0] + p.segmentLength * Math.cos(parentHeading),
        start[1] + p.segmentLength * Math.sin(parentHeading),
      ];
    }
    if (safeMotion(this.arm.base, before, proposed, p.segmentLength, obstacles))
      this.arm.angles = proposed;
    else this.blocked = true;
  }
}
