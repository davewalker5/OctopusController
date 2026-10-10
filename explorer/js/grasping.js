/**
 * Contact qualification and rigid payload geometry, corresponding to grasping.py.
 *
 * This module neither advances dwell timers nor awards ownership; those duties
 * belong to ControlledArm and CentralController. Geometry uses pixels and radians.
 */

import { distance, posePoints } from './geometry.js';
import { wrapAngle } from './segment-control.js';
import { ObjectKind } from './sensing.js';
/**
 * Find food touched by at least two consecutive segments and not already owned.
 * @param {import('./sensing.js').Contact[]} contacts Current local observations.
 * @param {Set<number>} unavailable Object identities claimed by other arms.
 * @returns {[number, number]|null} Object ID and last segment of the first pair, or null.
 */
export function graspCandidate(contacts, unavailable) {
  const byObject = new Map();
  for (const c of contacts) {
    if (c.kind !== ObjectKind.FOOD || unavailable.has(c.objectId)) continue;
    if (!byObject.has(c.objectId)) byObject.set(c.objectId, new Set());
    byObject.get(c.objectId).add(c.segmentIndex);
  }
  // Sort numerically: JavaScript's default sort is lexical. Stable object and
  // segment order reproduces Python's tie-break when several grips are possible.
  for (const id of [...byObject.keys()].sort((a, b) => a - b)) {
    // -2 ensures segment zero starts a new run. Sets prevent repeated sensor
    // entries for one segment from being mistaken for adjacent contacts.
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
/**
 * Rigid attachment of a food circle to the end of a gripping segment.
 *
 * The stored offset uses the segment's along/across axes. It models a secure
 * latch, not pressure, friction, or continued contact from every original sensor.
 */
export class Grip {
  /**
   * Store an attachment already qualified by local contact rules.
   * @param {number} objectId Stable positive food identity.
   * @param {number} segmentIndex Zero-based gripping segment index.
   * @param {import('./geometry.js').Point} offset Along/across displacement in pixels.
   */
  constructor(objectId, segmentIndex, offset) {
    this.objectId = objectId;
    this.segmentIndex = segmentIndex;
    this.offset = [...offset];
  }
  /**
   * Capture the existing object-to-segment relationship without moving the object.
   * @param {import('./sensing.js').WorldObject} obj Food being captured.
   * @param {number} segment Valid gripping segment index, zero-based.
   * @param {import('./geometry.js').Point[]} points Current arm joints, including the tip.
   * @returns {Grip} Attachment that preserves the object's current centre.
   */
  static attach(obj, segment, points) {
    const start = points[segment],
      end = points[segment + 1];
    const heading = Math.atan2(end[1] - start[1], end[0] - start[0]);
    const dx = obj.centre[0] - end[0],
      dy = obj.centre[1] - end[1];
    // Rotate the world-space displacement by minus the segment heading to
    // record coordinates in the gripping segment's local frame.
    return new Grip(obj.identifier, segment, [
      dx * Math.cos(heading) + dy * Math.sin(heading),
      -dx * Math.sin(heading) + dy * Math.cos(heading),
    ]);
  }
  /**
   * Reconstruct the payload centre for another pose of the same arm geometry.
   * @param {import('./geometry.js').Point[]} points Joints containing the gripping segment.
   * @returns {import('./geometry.js').Point} Held object's world centre in pixels.
   */
  centre(points) {
    const start = points[this.segmentIndex],
      end = points[this.segmentIndex + 1];
    const heading = Math.atan2(end[1] - start[1], end[0] - start[0]);
    // Rotate the stored offset back into world space, then translate from
    // the gripping segment's endpoint. This is the inverse of attach().
    const [along, across] = this.offset;
    return [
      end[0] + along * Math.cos(heading) - across * Math.sin(heading),
      end[1] + along * Math.sin(heading) + across * Math.cos(heading),
    ];
  }
  /**
   * Check the carried circle's swept path against all obstacle circles.
   * @param {import('./geometry.js').Point} base Fixed arm attachment in pixels.
   * @param {number[]} before Current relative joint angles in radians.
   * @param {number[]} after Proposed relative angles with unchanged segment count.
   * @param {number} length Positive segment length in pixels.
   * @param {number} radius Positive radius of the held food circle in pixels.
   * @param {import('./sensing.js').WorldObject[]} objects Scene snapshot; food is ignored.
   * @returns {boolean} Whether the payload stays clear or avoids worsening prior overlap.
   */
  safeMotion(base, before, after, length, radius, objects) {
    const changes = after.map((angle, i) => angle - before[i]);
    // Match the arm guard: only the absolute base angle may take a wrapped
    // shortcut; constrained relative bends must interpolate directly.
    changes[0] = wrapAngle(changes[0]);
    let headingChange = 0,
      travel = 0;
    // Links beyond the gripping segment cannot move its attachment. Sum only
    // the upstream arc-length bounds, then include rotation of the offset itself.
    for (const change of changes.slice(0, this.segmentIndex + 1)) {
      headingChange += change;
      travel += length * Math.abs(headingChange);
    }
    travel += Math.hypot(...this.offset) * Math.abs(headingChange);
    // Two pixels is the maximum bounded travel per interval, as in avoidance.js.
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
        // Existing overlap may be caused by an edit. Midpoint padding covers
        // unsampled motion, while the 1e-8 tolerance absorbs round-off only.
        const minimum = Math.min(radius + obj.radius, distance(oldCentre, obj.centre));
        if (distance(centre, obj.centre) - travel / (2 * samples) < minimum - 1e-8) return false;
      }
    }
    return true;
  }
}
