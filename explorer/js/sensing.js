import { closestPoint, distance, validatePoint } from './geometry.js';
export const ObjectKind = Object.freeze({ FOOD: 'Food', OBSTACLE: 'Obstacle' });
/** Immutable object snapshots prevent scene edits from changing old readings. */
export function worldObject(identifier, centre, kind, radius = 14) {
  validatePoint(centre);
  if (!Number.isInteger(identifier) || identifier < 1) throw new Error('Invalid object identity');
  if (!Number.isFinite(radius) || radius <= 0)
    throw new Error('Object radius must be finite and positive');
  if (!Object.values(ObjectKind).includes(kind)) throw new Error('Invalid object kind');
  return Object.freeze({
    identifier,
    centre: Object.freeze([...centre]),
    kind,
    radius,
  });
}
/** Read contacts in segment order, then scene insertion order. */
export function detectContacts(points, objects, sensorRadius = 3) {
  if (!Number.isFinite(sensorRadius) || sensorRadius < 0) throw new Error('Invalid sensor radius');
  points.forEach(validatePoint);
  const contacts = [];
  for (let index = 0; index < points.length - 1; index++) {
    for (const obj of objects) {
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
  constructor() {
    this.entries = new Map();
    this.nextIdentifier = 1;
  }
  get objects() {
    return [...this.entries.values()];
  }
  /** Add validated geometry and return its identity. */
  add(centre, kind, radius = 14) {
    const obj = worldObject(this.nextIdentifier, centre, kind, radius);
    this.entries.set(obj.identifier, obj);
    this.nextIdentifier++;
    return obj.identifier;
  }
  /** Replace an object snapshot, retaining its identity and type. */
  move(identifier, centre) {
    const obj = this.entries.get(identifier);
    if (!obj) throw new Error('Object no longer exists');
    this.entries.set(identifier, worldObject(identifier, centre, obj.kind, obj.radius));
  }
  /** Remove an existing object. */
  remove(identifier) {
    if (!this.entries.delete(identifier)) throw new Error('Object no longer exists');
  }
  /** Select the last drawn circle beneath a pointer. */
  pick(position) {
    validatePoint(position);
    return (
      this.objects.reverse().find((obj) => distance(position, obj.centre) <= obj.radius)
        ?.identifier ?? null
    );
  }
}
