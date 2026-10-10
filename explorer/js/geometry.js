/**
 * Shared geometry, corresponding to geometry.py in the Python implementation.
 *
 * World coordinates use logical pixels: x increases rightwards and y downwards.
 * Angles are clockwise radians from the positive x axis. Browser scaling belongs
 * in view.js and must never change these coordinates or segment lengths.
 *
 * @typedef {[number, number]} Point An [x, y] pair in logical world pixels.
 */
/**
 * Measure straight-line separation without modifying either point.
 * @param {Point} a First validated position.
 * @param {Point} b Second validated position.
 * @returns {number} Euclidean distance in pixels.
 */
export const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
/**
 * Restrict a scalar to an inclusive interval; callers supply ordered bounds.
 * @param {number} value Candidate value.
 * @param {number} low Lower limit, in the same units as value.
 * @param {number} high Upper limit, in the same units as value.
 * @returns {number} The original value or the nearest bound.
 */
export const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
/**
 * Reject coordinates that cannot describe a finite point before changing state.
 * @param {Point} point Candidate [x, y] coordinates.
 * @returns {void}
 * @throws {Error} If the value is not an array of exactly two finite numbers.
 */
export function validatePoint(point) {
  if (!Array.isArray(point) || point.length !== 2 || !point.every(Number.isFinite))
    throw new Error('A point must contain two finite coordinates');
}
/**
 * Validate an elapsed simulation duration; zero is permitted as a no-op update.
 * @param {number} seconds Duration in simulation seconds, not milliseconds.
 * @returns {void}
 * @throws {Error} If the duration is negative, infinite or not a finite number.
 */
export function validateDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0)
    throw new Error('Elapsed seconds must be finite and non-negative');
}
/**
 * Find the closest position on a finite segment, including its endpoints.
 * @param {Point} start Validated segment start.
 * @param {Point} end Validated segment end.
 * @param {Point} point Validated position to project.
 * @returns {Point} A fresh coordinate pair; start's coordinates for a zero-length segment.
 */
export function closestPoint(start, end, point) {
  const dx = end[0] - start[0],
    dy = end[1] - start[1];
  const squared = dx * dx + dy * dy;
  // The dot product gives the fraction along the infinite line. Clamping it
  // prevents sensing or collision checks from extending beyond the real link.
  // A collapsed link has no direction and is safely treated as one point.
  const fraction =
    squared > 0
      ? clamp(((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared, 0, 1)
      : 0;
  return [start[0] + fraction * dx, start[1] + fraction * dy];
}
/**
 * Reconstruct a candidate pose without modifying the live arm or input arrays.
 * @param {Point} base Fixed attachment in world pixels.
 * @param {number[]} angles Relative joint angles in radians; the first is absolute.
 * @param {number} length Validated positive length shared by all segments, in pixels.
 * @returns {Point[]} Fresh connected positions, including the base and final tip.
 */
export function posePoints(base, angles, length) {
  const points = [[...base]];
  let heading = 0;
  for (const angle of angles) {
    // Sum relative bends to recover this link's world heading. Passing its
    // endpoint to the next link keeps the chain connected and unstretched.
    heading += angle;
    const last = points.at(-1);
    points.push([last[0] + length * Math.cos(heading), last[1] + length * Math.sin(heading)]);
  }
  return points;
}
