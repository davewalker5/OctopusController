/**
 * Obstacle steering and swept clearance, corresponding to avoidance.py.
 *
 * Steering inspects a single link's attachment and requested endpoint. The
 * separate whole-arm guard may reject a proposed movement, but never chooses
 * joint angles or replaces the local rule with a globally planned route.
 */

import { closestPoint, distance, posePoints } from './geometry.js';
import { wrapAngle } from './segment-control.js';
// Effective arm radius in pixels; decorative stroke widths do not affect collisions.
export const ARM_CLEARANCE = 3;
/**
 * Redirect a segment's request around the nearest relevant circular boundary.
 * @param {import('./geometry.js').Point} start This segment's attachment, in pixels.
 * @param {import('./geometry.js').Point} goal Endpoint requested by its neighbour or target.
 * @param {number} length Positive link length, used to set the local lookahead.
 * @param {import('./sensing.js').WorldObject[]} obstacles Validated obstacles; filter food first.
 * @returns {[import('./geometry.js').Point, boolean]} Endpoint and whether it was redirected.
 */
export function steerAroundObstacles(start, goal, length, obstacles) {
  const dx = goal[0] - start[0],
    dy = goal[1] - start[1],
    gap = Math.hypot(dx, dy);
  // Below the geometric tolerance there is no reliable direction to steer.
  if (gap <= 1e-8) return [goal, false];
  // Only inspect the next three segment lengths. A distant obstacle must not
  // give a local rule advance knowledge of the entire arm's future route.
  const lookahead = Math.min(gap, length * 3);
  const end = [start[0] + (dx / gap) * lookahead, start[1] + (dy / gap) * lookahead];
  let nearest = null,
    nearestDistance = Infinity;
  for (const obj of obstacles) {
    // The 11-pixel inflation is 3 pixels of arm clearance plus an 8-pixel
    // steering margin: begin the detour before the hard collision boundary.
    if (distance(closestPoint(start, end, obj.centre), obj.centre) >= obj.radius + 11) continue;
    const gap = distance(start, obj.centre);
    if (gap < nearestDistance) {
      nearest = obj;
      nearestDistance = gap;
    }
  }
  if (!nearest) return [goal, false];
  // Always travel clockwise to avoid alternating sides due to tiny numerical
  // changes. Aim 0.4 radians ahead; 16 = clearance (3) + steering margin (8)
  // + waypoint margin (5), keeping the chord outside the blocked circle.
  const heading = Math.atan2(start[1] - nearest.centre[1], start[0] - nearest.centre[0]) + 0.4;
  const radius = nearest.radius + 16;
  return [
    [
      nearest.centre[0] + radius * Math.cos(heading),
      nearest.centre[1] + radius * Math.sin(heading),
    ],
    true,
  ];
}
/**
 * Check the complete angular sweep, rather than just the final pose.
 *
 * This is a conservative acceptance guard, not a route planner. Inputs have
 * already been validated and proposed angles satisfy bend and speed limits.
 *
 * @param {import('./geometry.js').Point} base Fixed attachment in pixels.
 * @param {number[]} before Current relative joint angles in radians.
 * @param {number[]} after Proposed angles, with the same length as before.
 * @param {number} length Positive segment length in pixels.
 * @param {import('./sensing.js').WorldObject[]} obstacles Obstacle-only snapshot.
 * @returns {boolean} Whether clearance is preserved or existing overlap is not worsened.
 */
export function safeMotion(base, before, after, length, obstacles) {
  if (!obstacles.length) return true;
  const previous = posePoints(base, before, length);
  const changes = after.map((angle, i) => angle - before[i]);
  // Only the base rotates freely. Wrapping a relative bend would interpolate
  // across the wrong side of its limit even if the endpoints looked acceptable.
  changes[0] = wrapAngle(changes[0]);
  let headingChange = 0,
    travel = 0;
  const bounds = changes.map((change) => {
    headingChange += change;
    // Arc length bounds endpoint travel. Summing upstream contributions also
    // covers movement inherited from rotations nearer the fixed attachment.
    travel += length * Math.abs(headingChange);
    return travel;
  });
  // Use at most two pixels of bounded tip travel per interval. Padding below
  // covers unsampled movement, so small obstacles cannot slip between samples.
  const samples = Math.max(1, Math.ceil(travel / 2));
  for (let sample = 0; sample < samples; sample++) {
    const fraction = (sample + 0.5) / samples;
    const points = posePoints(
      base,
      before.map((angle, i) => angle + fraction * changes[i]),
      length,
    );
    for (let i = 0; i < before.length; i++) {
      // Any point on this segment can move by at most half its interval
      // travel bound from the midpoint towards either edge of that interval.
      const padding = bounds[i] / (2 * samples);
      for (const obj of obstacles) {
        const oldDistance = distance(
          closestPoint(previous[i], previous[i + 1], obj.centre),
          obj.centre,
        );
        // A scene edit can place an obstacle over an existing arm. Permit a
        // retreat without teleporting the arm or accepting a deeper overlap.
        // The 1e-8-pixel tolerance below absorbs geometric round-off only.
        const minimum = Math.min(obj.radius + ARM_CLEARANCE, oldDistance);
        if (
          distance(closestPoint(points[i], points[i + 1], obj.centre), obj.centre) - padding <
          minimum - 1e-8
        )
          return false;
      }
    }
  }
  return true;
}
