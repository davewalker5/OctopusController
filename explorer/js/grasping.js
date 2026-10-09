import { distance, posePoints } from './geometry.js';
import { wrapAngle } from './segment-control.js';
import { ObjectKind } from './sensing.js';
/** Find the first adjacent contact pair in stable object/segment order. */
export function graspCandidate(contacts, unavailable) {
  const byObject = new Map();
  for (const c of contacts) {
    if (c.kind !== ObjectKind.FOOD || unavailable.has(c.objectId)) continue;
    if (!byObject.has(c.objectId)) byObject.set(c.objectId, new Set());
    byObject.get(c.objectId).add(c.segmentIndex);
  }
  for (const id of [...byObject.keys()].sort((a, b) => a - b)) {
    let run = 0,
      previous = -2;
    for (const segment of [...byObject.get(id)].sort((a, b) => a - b)) {
      run = segment === previous + 1 ? run + 1 : 1;
      if (run >= 2) return [id, segment];
      previous = segment;
    }
  }
  return null;
}
export class Grip {
  constructor(objectId, segmentIndex, offset) {
    this.objectId = objectId;
    this.segmentIndex = segmentIndex;
    this.offset = [...offset];
  }
  /** Capture the existing local offset without snapping the food. */
  static attach(obj, segment, points) {
    const start = points[segment],
      end = points[segment + 1];
    const heading = Math.atan2(end[1] - start[1], end[0] - start[0]);
    const dx = obj.centre[0] - end[0],
      dy = obj.centre[1] - end[1];
    return new Grip(obj.identifier, segment, [
      dx * Math.cos(heading) + dy * Math.sin(heading),
      -dx * Math.sin(heading) + dy * Math.cos(heading),
    ]);
  }
  /** Transform the stored attachment into world coordinates for this pose. */
  centre(points) {
    const start = points[this.segmentIndex],
      end = points[this.segmentIndex + 1];
    const heading = Math.atan2(end[1] - start[1], end[0] - start[0]);
    const [along, across] = this.offset;
    return [
      end[0] + along * Math.cos(heading) - across * Math.sin(heading),
      end[1] + along * Math.sin(heading) + across * Math.cos(heading),
    ];
  }
  /** Bound the swept payload circle, allowing retreat from existing overlap. */
  safeMotion(base, before, after, length, radius, objects) {
    const changes = after.map((angle, i) => angle - before[i]);
    changes[0] = wrapAngle(changes[0]);
    let headingChange = 0,
      travel = 0;
    for (const change of changes.slice(0, this.segmentIndex + 1)) {
      headingChange += change;
      travel += length * Math.abs(headingChange);
    }
    travel += Math.hypot(...this.offset) * Math.abs(headingChange);
    const samples = Math.max(1, Math.ceil(travel / 2));
    const oldCentre = this.centre(posePoints(base, before, length));
    for (let sample = 0; sample < samples; sample++) {
      const fraction = (sample + 0.5) / samples;
      const centre = this.centre(
        posePoints(
          base,
          before.map((angle, i) => angle + fraction * changes[i]),
          length,
        ),
      );
      for (const obj of objects) {
        if (obj.kind !== ObjectKind.OBSTACLE) continue;
        const minimum = Math.min(radius + obj.radius, distance(oldCentre, obj.centre));
        if (distance(centre, obj.centre) - travel / (2 * samples) < minimum - 1e-8) return false;
      }
    }
    return true;
  }
}
