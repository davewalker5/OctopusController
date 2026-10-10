/**
 * Arm geometry and local reaching, corresponding to model.py.
 *
 * All coordinates are logical world pixels. Positive headings turn clockwise
 * because screen y increases downwards. Model angles and speeds use radians;
 * no browser rendering or event handling belongs in this module.
 *
 * @typedef {object} ArmParameters
 * @property {number} segmentCount Integer number of links, from 2 to 100.
 * @property {number} segmentLength Positive length per link in pixels.
 * @property {number} maximumBend Positive relative bend limit, at most pi radians.
 * @property {number} turningSpeed Positive maximum radians per joint per second.
 */

import { distance, posePoints, validatePoint, validateDuration } from './geometry.js';
import { wrapAngle, requestStart, turnSegment } from './segment-control.js';
import { ARM_CLEARANCE, safeMotion, steerAroundObstacles } from './avoidance.js';
import { ObjectKind, detectContacts } from './sensing.js';
/**
 * Resolve standalone defaults and validate geometry and motion limits.
 *
 * The standalone length is 18 pixels; organism.js supplies 12 for the eight-arm
 * scene. Angles are radians here: scenario.js and app.js handle degree conversion.
 *
 * @param {Partial<ArmParameters>} [overrides={}] Supplied fields replace their defaults.
 * @returns {ArmParameters} Frozen parameter record.
 * @throws {Error} If segment count, length, angular limit or speed is invalid.
 */
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
/**
 * A fixed base and connected rigid links with relative joint angles.
 *
 * Only the first angle is an absolute world heading. The remaining angles are
 * bends relative to the previous link, bounded by the arm's maximum bend.
 */
export class Arm {
  /**
   * Create an arm with a deterministic, gently curved initial pose.
   * @param {import('./geometry.js').Point} base Fixed attachment in world pixels.
   * @param {ArmParameters} [parameters] Validated geometry and motion settings.
   * @param {number} [heading=0] Initial absolute base heading, in clockwise radians.
   * @throws {Error} If the base or initial heading is not finite.
   */
  constructor(base, parameters = armParameters(), heading = 0) {
    validatePoint(base);
    if (!Number.isFinite(heading)) throw new Error('Initial heading must be finite');
    this.base = [...base];
    this.parameters = parameters;
    // A straight arm has no preferred fold direction for an inward target on
    // its own axis. A 0.06-radian bend seeds a consistent curve without noise,
    // and is reduced if the configured bend limit is smaller.
    this.angles = [
      wrapAngle(heading),
      ...Array(parameters.segmentCount - 1).fill(Math.min(0.06, parameters.maximumBend)),
    ];
  }
  /**
   * Reconstruct a fresh connected pose from the live joint angles.
   * @returns {import('./geometry.js').Point[]} Base, intermediate joints and final tip.
   */
  get points() {
    return posePoints(this.base, this.angles, this.parameters.segmentLength);
  }
}
/**
 * Schedule local reaching rules and guard their proposed movement.
 *
 * Only the final segment sees the target directly. Other segments see their
 * own attachment and the previous request from their immediate tip-side
 * neighbour. The controller assembles these local choices, rather than solving
 * joint angles from the distant tip. Swept checks may veto but never plan a turn.
 */
export class ReachController {
  /**
   * Assign an objective and seed neighbour messages from the existing pose.
   * @param {Arm} arm Arm whose angles this controller will update.
   * @param {import('./geometry.js').Point} target Desired tip position in world pixels.
   * @throws {Error} If target coordinates are invalid.
   */
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
  /**
   * Retarget without resetting the pose or messages already in flight.
   * @param {import('./geometry.js').Point} target Finite desired tip position in pixels.
   * @returns {void}
   * @throws {Error} If the new target is invalid.
   */
  setTarget(target) {
    validatePoint(target);
    this.target = [...target];
    this.blocked = false;
    this.avoiding = false;
  }
  /**
   * Measure current tip-to-target separation.
   * @returns {number} Distance in logical world pixels.
   */
  get distance() {
    return distance(this.arm.points.at(-1), this.target);
  }
  /**
   * Report constraints before reach progress; being in range is not a route guarantee.
   * @returns {string} Blocked, Avoiding, Reached, Beyond reach or Reaching.
   */
  get status() {
    if (this.blocked) return 'Blocked';
    if (this.avoiding) return 'Avoiding';
    // Two pixels is the common tip-arrival tolerance, not a biological measure.
    if (this.distance <= 2) return 'Reached';
    const p = this.arm.parameters;
    return distance(this.target, this.arm.base) > p.segmentCount * p.segmentLength
      ? 'Beyond reach'
      : 'Reaching';
  }
  /**
   * Exchange local requests and attempt one bounded turn per segment.
   *
   * Messages are retained even if clearance rejects the proposed pose. Later
   * requests can therefore evolve while the visible arm remains blocked.
   *
   * @param {number} seconds Finite non-negative simulation duration in seconds.
   * @param {import('./sensing.js').WorldObject[]} [objects=[]] Read-only scene snapshot.
   * @returns {void} Updates requests, status flags and, if safe, the arm's angles.
   * @throws {Error} If the duration is invalid, before any movement is attempted.
   */
  update(seconds, objects = []) {
    validateDuration(seconds);
    if (seconds === 0) return;
    const obstacles = objects.filter((obj) => obj.kind === ObjectKind.OBSTACLE);
    this.blocked = this.avoiding = false;
    // The tip cannot reach inside a solid circle plus arm clearance. Preserve
    // the assignment so a later target or scene edit can allow it to resume.
    if (obstacles.some((obj) => distance(this.target, obj.centre) < obj.radius + ARM_CLEARANCE)) {
      this.blocked = true;
      return;
    }
    // Arrival alone must not suppress an escape attempt if a newly moved
    // obstacle overlaps the current arm. Food never constrains movement.
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
    // The right-hand map completes before this.requests is replaced, so each
    // neighbour heading still comes from the previous tick, not this pass.
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
    // Pass the real attachment forwards from the fixed base. This propagates
    // chain geometry, not a central command for each segment's orientation.
    const before = [...this.arm.angles],
      proposed = [];
    let start = this.arm.base,
      parentHeading = 0;
    for (let i = 0; i < goals.length; i++) {
      // Upstream movement may have shifted the attachment since request
      // creation. Recheck that local approach before deciding the bounded turn.
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
      // Construct the next attachment at exactly one link length, preventing
      // gaps or stretching even when the requested endpoint is unattainable.
      start = [
        start[0] + p.segmentLength * Math.cos(parentHeading),
        start[1] + p.segmentLength * Math.sin(parentHeading),
      ];
    }
    // Commit only after checking the full swept arm. On rejection, retain the
    // new messages so neighbours can continue adjusting on later ticks.
    if (safeMotion(this.arm.base, before, proposed, p.segmentLength, obstacles))
      this.arm.angles = proposed;
    else this.blocked = true;
  }
}
