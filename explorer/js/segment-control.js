import { clamp } from './geometry.js';
/** Python-compatible positive modulo, including negative headings. */
export function wrapAngle(angle) {
  const turn = 2 * Math.PI;
  const remainder = (angle + Math.PI) % turn;
  return (remainder < 0 ? remainder + turn : remainder) - Math.PI;
}
/** Ask the base-side neighbour for an attachment, using only adjacent state. */
export function requestStart(start, goal, heading, neighbourHeading, length, maximumBend) {
  const dx = goal[0] - start[0],
    dy = goal[1] - start[1];
  let direction = Math.hypot(dx, dy) > 1e-9 ? Math.atan2(dy, dx) : heading;
  if (neighbourHeading !== null)
    direction =
      neighbourHeading + clamp(wrapAngle(direction - neighbourHeading), -maximumBend, maximumBend);
  return {
    start: [goal[0] - length * Math.cos(direction), goal[1] - length * Math.sin(direction)],
    heading: wrapAngle(direction),
  };
}
/** Apply a speed-limited local turn, easing small errors as in Python. */
export function turnSegment(start, goal, parentHeading, angle, maximumBend, speed, seconds) {
  const dx = goal[0] - start[0],
    dy = goal[1] - start[1];
  if (Math.hypot(dx, dy) <= 1e-9) return angle;
  const difference = wrapAngle(Math.atan2(dy, dx) - parentHeading - angle);
  const change = difference * (1 - Math.exp(-20 * seconds));
  const updated = angle + clamp(change, -speed * seconds, speed * seconds);
  return maximumBend === null ? wrapAngle(updated) : clamp(updated, -maximumBend, maximumBend);
}
