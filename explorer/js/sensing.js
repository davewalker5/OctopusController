/**
 * Local contact detection and editable scene storage, corresponding to sensing.py.
 *
 * Object IDs survive movement; contacts refer to these identities rather than
 * array positions. Object snapshots are immutable, while the environment owns
 * replacement and deletion. Distances and radii are logical world pixels.
 *
 * @typedef {object} WorldObject
 * @property {number} identifier Positive, stable scene identity.
 * @property {import('./geometry.js').Point} centre Frozen world coordinates.
 * @property {'Food'|'Obstacle'} kind Sensor category.
 * @property {number} radius Positive circular radius in pixels.
 *
 * @typedef {object} Contact
 * @property {number} segmentIndex Zero-based observing segment.
 * @property {number} objectId Identity of the observed object.
 * @property {'Food'|'Obstacle'} kind Observed category.
 * @property {import('./geometry.js').Point} position Closest point on the segment.
 */

import { closestPoint, distance, validatePoint } from './geometry.js';
/** Object categories understood by the local sensors; values also label the interface. */
export const ObjectKind = Object.freeze({ FOOD: 'Food', OBSTACLE: 'Obstacle' });
/**
 * Create a validated, immutable circular scene object.
 * @param {number} identifier Positive integer identity, stable across object moves.
 * @param {import('./geometry.js').Point} centre World position in pixels; copied on creation.
 * @param {'Food'|'Obstacle'} kind Category distinguishable by a local sensor.
 * @param {number} [radius=14] Positive circle radius in pixels.
 * @returns {WorldObject} Frozen record with a separately copied, frozen centre.
 * @throws {Error} If identity, coordinates, radius or category are invalid.
 */
export function worldObject(identifier, centre, kind, radius = 14) {
  validatePoint(centre);
  if (!Number.isInteger(identifier) || identifier < 1) throw new Error('Invalid object identity');
  if (!Number.isFinite(radius) || radius <= 0)
    throw new Error('Object radius must be finite and positive');
  if (!Object.values(ObjectKind).includes(kind)) throw new Error('Invalid object kind');
  // Copy and freeze the coordinates too: freezing only the record would still
  // allow a caller's array to move the object while an old snapshot is in use.
  return Object.freeze({
    identifier,
    centre: Object.freeze([...centre]),
    kind,
    radius,
  });
}
/**
 * Read each segment independently, returning only its current observations.
 *
 * A sensing strip covers the complete link, including joints and the tip.
 * It is geometric detection only; obstacle constraints live in avoidance.js.
 *
 * @param {import('./geometry.js').Point[]} points Connected arm joints in world pixels.
 * @param {WorldObject[]} objects Immutable object records in scene insertion order.
 * @param {number} [sensorRadius=3] Non-negative sensing-strip radius in pixels.
 * @returns {Contact[]} Fresh contact records, ordered by segment and then scene order.
 * @throws {Error} If a point or the sensing radius is invalid.
 */
export function detectContacts(points, objects, sensorRadius = 3) {
  if (!Number.isFinite(sensorRadius) || sensorRadius < 0) throw new Error('Invalid sensor radius');
  points.forEach(validatePoint);
  const contacts = [];
  for (let index = 0; index < points.length - 1; index++) {
    for (const obj of objects) {
      // The finite-segment projection prevents detecting along an imaginary
      // extension of a link. Equality counts as touch; add the two radii to
      // include the sensing strip independently of decorative arm thickness.
      const position = closestPoint(points[index], points[index + 1], obj.centre);
      if (distance(obj.centre, position) <= obj.radius + sensorRadius)
        contacts.push(
          Object.freeze({
            segmentIndex: index,
            objectId: obj.identifier,
            kind: obj.kind,
            position: Object.freeze(position),
          }),
        );
    }
  }
  return contacts;
}
/** Editable world with stable, never-reused identities. */
export class Environment {
  /**
   * Start an empty scene. Removing an object never makes its identity reusable.
   */
  constructor() {
    this.entries = new Map();
    this.nextIdentifier = 1;
  }
  /**
   * Return a new array of immutable records in insertion/drawing order.
   * @returns {WorldObject[]} A snapshot whose membership cannot modify scene storage.
   */
  get objects() {
    return [...this.entries.values()];
  }
  /**
   * Place an object, allocating an identity only after validation succeeds.
   * @param {import('./geometry.js').Point} centre Object centre in world pixels.
   * @param {'Food'|'Obstacle'} kind Sensor category.
   * @param {number} [radius=14] Positive radius in pixels.
   * @returns {number} Newly allocated, stable scene identity.
   * @throws {Error} If the geometry or category is invalid.
   */
  add(centre, kind, radius = 14) {
    const obj = worldObject(this.nextIdentifier, centre, kind, radius);
    this.entries.set(obj.identifier, obj);
    this.nextIdentifier++;
    return obj.identifier;
  }
  /**
   * Move an object by replacing its record, leaving previous snapshots intact.
   * @param {number} identifier Identity of an existing object.
   * @param {import('./geometry.js').Point} centre New finite world coordinates.
   * @returns {void}
   * @throws {Error} If the object no longer exists or its new coordinates are invalid.
   */
  move(identifier, centre) {
    const obj = this.entries.get(identifier);
    if (!obj) throw new Error('Object no longer exists');
    this.entries.set(identifier, worldObject(identifier, centre, obj.kind, obj.radius));
  }
  /**
   * Delete an object; a subsequent sensing refresh clears its contacts.
   * @param {number} identifier Identity of an existing scene object.
   * @returns {void}
   * @throws {Error} If the object no longer exists.
   */
  remove(identifier) {
    if (!this.entries.delete(identifier)) throw new Error('Object no longer exists');
  }
  /**
   * Find the topmost circle under a pointer, matching the rendering order.
   * @param {import('./geometry.js').Point} position Pointer in logical world pixels.
   * @returns {number|null} Last drawn object's identity, or null over empty space.
   * @throws {Error} If the pointer coordinates are invalid.
   */
  pick(position) {
    validatePoint(position);
    // objects is a fresh array, so reversing it cannot reorder the real scene.
    return (
      this.objects.reverse().find((obj) => distance(position, obj.centre) <= obj.radius)
        ?.identifier ?? null
    );
  }
}
