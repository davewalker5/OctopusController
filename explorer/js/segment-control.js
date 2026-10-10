/**
 * Local segment rules, corresponding to segment_control.py.
 *
 * A rule sees one link and its neighbours, never the distant arm tip. Positions
 * are logical world pixels; headings are clockwise radians and joint bends are
 * relative to the preceding link. Requests travel through ReachController.
 *
 * @typedef {{start: import('./geometry.js').Point, heading: number}} SegmentRequest
 * A desired start position and proposed absolute heading, not an immediate move.
 */

import { clamp } from './geometry.js';
/**
 * Choose the equivalent direction in the nominal interval [-pi, pi).
 * @param {number} angle Finite angle in radians.
 * @returns {number} Wrapped radians, subject to floating-point boundary rounding.
 */
export function wrapAngle(angle) {
  const turn = 2 * Math.PI;
  // JavaScript % is a signed remainder, unlike Python's positive modulo.
  // Add a turn only for negative remainders: an unconditional extra modulo
  // introduces rounding differences at the ±pi boundary in the parity tests.
  const remainder = (angle + Math.PI) % turn;
  return (remainder < 0 ? remainder + turn : remainder) - Math.PI;
}
/**
 * Ask the preceding neighbour for a useful attachment position.
 *
 * The tip uses the assigned target as its goal; every other segment receives
 * its tip-side neighbour's requested start. The arm validates inputs before
 * calling these internal rules. This function creates a message, not a pose.
 *
 * @param {import('./geometry.js').Point} start This segment's current attachment.
 * @param {import('./geometry.js').Point} goal Requested endpoint in world pixels.
 * @param {number} heading This segment's absolute heading in radians.
 * @param {number|null} neighbourHeading Neighbour's proposed heading, or null at the tip.
 * @param {number} length Positive segment length in pixels.
 * @param {number} maximumBend Allowed bend between neighbours, in radians.
 * @returns {SegmentRequest} Requested attachment and corresponding absolute heading.
 */
export function requestStart(start, goal, heading, neighbourHeading, length, maximumBend) {
  const dx = goal[0] - start[0],
    dy = goal[1] - start[1];
  // Within 1e-9 pixels the direction is undefined. Keep the current heading
  // instead of allowing atan2(0, 0) to introduce an arbitrary rightward turn.
  let direction = Math.hypot(dx, dy) > 1e-9 ? Math.atan2(dy, dx) : heading;
  // Respect the joining bend in the message itself, otherwise neighbours can
  // repeatedly request an impossible fold while the joints sit at their limits.
  if (neighbourHeading !== null)
    direction =
      neighbourHeading + clamp(wrapAngle(direction - neighbourHeading), -maximumBend, maximumBend);
  // Walk one full link backwards from the desired endpoint. Actual movement
  // may not fulfil this request because the base and turning limits still apply.
  return {
    start: [goal[0] - length * Math.cos(direction), goal[1] - length * Math.sin(direction)],
    heading: wrapAngle(direction),
  };
}
/**
 * Choose one bounded turn using the actual attachment and an adjacent request.
 * @param {import('./geometry.js').Point} start Attachment supplied by the base-side link.
 * @param {import('./geometry.js').Point} goal Requested endpoint in world pixels.
 * @param {number} parentHeading Base-side neighbour's absolute heading in radians.
 * @param {number} angle Current bend relative to the parent, in radians.
 * @param {number|null} maximumBend Relative bend limit, or null for the freely turning base.
 * @param {number} speed Maximum angular change per simulation second, in radians.
 * @param {number} seconds Validated non-negative simulation duration.
 * @returns {number} New relative angle; no input positions or angles are mutated.
 */
export function turnSegment(start, goal, parentHeading, angle, maximumBend, speed, seconds) {
  const dx = goal[0] - start[0],
    dy = goal[1] - start[1];
  if (Math.hypot(dx, dy) <= 1e-9) return angle;
  // Convert the world direction into a relative bend and take its shortest
  // angular difference, including requests that cross the ±pi boundary.
  const difference = wrapAngle(Math.atan2(dy, dx) - parentHeading - angle);
  // Neighbour messages take time to propagate. Exponential easing at 20/s
  // reduces overshoot while the other links catch up, independently of frame
  // rate. The separate speed bound limits large turns per simulation step.
  const change = difference * (1 - Math.exp(-20 * seconds));
  const updated = angle + clamp(change, -speed * seconds, speed * seconds);
  return maximumBend === null ? wrapAngle(updated) : clamp(updated, -maximumBend, maximumBend);
}
