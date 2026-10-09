import { closestPoint, distance, posePoints } from './geometry.js';
import { wrapAngle } from './segment-control.js';
export const ARM_CLEARANCE = 3;
/** Follow the nearest blocking circular boundary clockwise, without global planning. */
export function steerAroundObstacles(start, goal, length, obstacles) {
  const dx = goal[0] - start[0],
    dy = goal[1] - start[1],
    gap = Math.hypot(dx, dy);
  if (gap <= 1e-8) return [goal, false];
  const lookahead = Math.min(gap, length * 3);
  const end = [start[0] + (dx / gap) * lookahead, start[1] + (dy / gap) * lookahead];
  let nearest = null,
    nearestDistance = Infinity;
  for (const obj of obstacles) {
    if (distance(closestPoint(start, end, obj.centre), obj.centre) >= obj.radius + 11) continue;
    const gap = distance(start, obj.centre);
    if (gap < nearestDistance) {
      nearest = obj;
      nearestDistance = gap;
    }
  }
  if (!nearest) return [goal, false];
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
/** Reject swept crossings; midpoint padding bounds motion between samples. */
export function safeMotion(base, before, after, length, obstacles) {
  if (!obstacles.length) return true;
  const previous = posePoints(base, before, length);
  const changes = after.map((angle, i) => angle - before[i]);
  changes[0] = wrapAngle(changes[0]);
  let headingChange = 0,
    travel = 0;
  const bounds = changes.map((change) => {
    headingChange += change;
    travel += length * Math.abs(headingChange);
    return travel;
  });
  const samples = Math.max(1, Math.ceil(travel / 2));
  for (let sample = 0; sample < samples; sample++) {
    const fraction = (sample + 0.5) / samples;
    const points = posePoints(
      base,
      before.map((angle, i) => angle + fraction * changes[i]),
      length,
    );
    for (let i = 0; i < before.length; i++) {
      const padding = bounds[i] / (2 * samples);
      for (const obj of obstacles) {
        const oldDistance = distance(
          closestPoint(previous[i], previous[i + 1], obj.centre),
          obj.centre,
        );
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
