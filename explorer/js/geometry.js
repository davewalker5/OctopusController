/** Geometry in logical world pixels, independent of the browser. */
export const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
export const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
/** Reject invalid coordinates before they enter the model. */
export function validatePoint(point) {
  if (!Array.isArray(point) || point.length !== 2 || !point.every(Number.isFinite))
    throw new Error('A point must contain two finite coordinates');
}
/** Reject invalid durations before changing any state. */
export function validateDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0)
    throw new Error('Elapsed seconds must be finite and non-negative');
}
/** Project a point onto a finite segment, including zero-length segments. */
export function closestPoint(start, end, point) {
  const dx = end[0] - start[0],
    dy = end[1] - start[1];
  const squared = dx * dx + dy * dy;
  const fraction =
    squared > 0
      ? clamp(((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared, 0, 1)
      : 0;
  return [start[0] + fraction * dx, start[1] + fraction * dy];
}
/** Reconstruct connected joints without mutating the live arm. */
export function posePoints(base, angles, length) {
  const points = [[...base]];
  let heading = 0;
  for (const angle of angles) {
    heading += angle;
    const last = points.at(-1);
    points.push([last[0] + length * Math.cos(heading), last[1] + length * Math.sin(heading)]);
  }
  return points;
}
