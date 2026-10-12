/**
 * Segment-local aversion, corresponding to hazards.py.
 *
 * Severity is an experimental value in (0, 1]; encounters at 0.4 or above
 * interrupt attraction with a timed recoil before local boundary steering.
 * Sensed hazards never become collision constraints or change the central target.
 * Positions and distances use world pixels, with x rightwards and y downwards.
 * Timers use simulation seconds, so callers must not advance them while paused.
 */
import { closestPoint, distance } from './geometry.js';
import { steerAroundObstacles } from './avoidance.js';
import { ObjectKind } from './sensing.js';
// Match Python's recoil threshold and the sensing strip's radius in world pixels.
const RECOIL_SEVERITY = 0.4,
  SENSOR_RADIUS = 3;
/**
 * One arm's aversive episode and per-segment observations.
 *
 * State and severity describe the current response; encounter and exposure totals
 * survive individual episodes until the controller is rebuilt. The arm-wide state
 * does not give unsensing segments knowledge of a hazard.
 */
export class HazardResponse {
  /** Start with no observations, active recoil or accumulated metrics. */
  constructor() {
    this.state = 'Reaching';
    this.severity = 0;
    this.encounters = 0;
    this.exposureSeconds = 0;
    // Remember identities across ticks so continued detection cannot restart
    // recoil indefinitely. Segment memories hold last-sensed geometry rather
    // than tracking unseen hazard movement.
    this.seen = new Set();
    this.remaining = 0;
    this.quiet = 0;
    this.local = new Map();
  }
  /**
   * Check proximity without changing episode state or advancing timers.
   *
   * Grasping uses this read-only query before starting a food-contact dwell.
   * Geometry and object records are assumed to have been validated by callers.
   *
   * @param {import('./geometry.js').Point[]} points Ordered world-pixel joints, including the tip.
   * @param {import('./sensing.js').WorldObject[]} objects Immutable environment records.
   * @returns {boolean} Whether any finite segment is within a hazard's sensing range.
   */
  nearby(points, objects) {
    // Project onto the full finite link: endpoint-only sensing misses hazards
    // beside its middle. The extra 3 pixels match the sensing-strip radius in
    // sensing.js; equality at the outer detection boundary counts as sensing.
    return points
      .slice(0, -1)
      .some((start, i) =>
        objects.some(
          (o) =>
            o.kind === ObjectKind.HAZARD &&
            distance(closestPoint(start, points[i + 1], o.centre), o.centre) <=
              o.radius + o.detectionRange + SENSOR_RADIUS,
        ),
      );
  }
  /**
   * Advance observations, episode metrics and the local aversive state.
   *
   * Call once per active movement tick with validated geometry and positive time.
   * ReachController validates duration and skips zero-time ticks. This mutates
   * only the response object, leaving the supplied pose and scene records intact.
   *
   * @param {import('./geometry.js').Point[]} points Ordered world-pixel joints, including the tip.
   * @param {import('./sensing.js').WorldObject[]} objects Immutable environment records.
   * @param {number} seconds Positive elapsed simulation time in seconds.
   * @returns {void} Updates state, severity, metrics and segment memories.
   */
  sense(points, objects, seconds) {
    const detected = new Set(),
      fresh = new Map();
    let severity = 0,
      exposed = false;
    // Keep geometry per segment, but aggregate identities and severity for the
    // arm's episode. Exposure is centreline penetration of the actual hazard,
    // rather than entry into its larger detection envelope (including the strip).
    for (let i = 0; i < points.length - 1; i++) {
      for (const obj of objects) {
        if (obj.kind !== ObjectKind.HAZARD) continue;
        const separation = distance(closestPoint(points[i], points[i + 1], obj.centre), obj.centre);
        if (separation <= obj.radius + obj.detectionRange + SENSOR_RADIUS) {
          if (!fresh.has(i)) fresh.set(i, []);
          fresh.get(i).push(obj);
          detected.add(obj.identifier);
          severity = Math.max(severity, obj.severity);
          exposed ||= separation < obj.radius;
        }
      }
    }
    // Several segments sensing the same hazard count as one encounter. Exposure
    // accrues once per tick, not once for each penetrating segment or hazard.
    const newIds = new Set([...detected].filter((id) => !this.seen.has(id)));
    this.encounters += newIds.size;
    if (exposed) this.exposureSeconds += seconds;
    if (newIds.size) {
      const strength = Math.max(
        ...objects.filter((o) => newIds.has(o.identifier)).map((o) => o.severity),
      );
      // Experimental tuning: below 0.4 use deflection only; otherwise recoil
      // for 0.25 + 0.9*s seconds. A new encounter can extend an existing timer
      // but must not shorten a stronger response already in progress.
      if (strength >= RECOIL_SEVERITY)
        this.remaining = Math.max(this.remaining, 0.25 + strength * 0.9);
    }
    detected.forEach((id) => this.seen.add(id));
    // Remove deleted hazards, but retain last-sensed positions for remaining
    // identities. Reading their new geometry here would reveal unseen movement.
    const existing = new Map(
      objects.filter((o) => o.kind === ObjectKind.HAZARD).map((o) => [o.identifier, o]),
    );
    this.local = new Map(
      [...this.local].map(([i, observations]) => [
        i,
        observations.filter((o) => existing.has(o.identifier)),
      ]),
    );
    // Fresh readings replace that segment's complete memory. Segments without
    // fresh readings retain theirs until the episode ends, supporting withdrawal
    // after they have moved outside the detection envelope.
    fresh.forEach((v, k) => this.local.set(k, v));
    // Keep the last sensed severity during recoil/reassessment so the inspector
    // can explain the response even when no hazard is currently within range.
    if (detected.size) {
      this.quiet = 0;
      this.severity = severity;
    } else this.quiet += seconds;
    if (!existing.size) this.remaining = 0;
    // Complete the timed recoil even after reaching safe separation. Resetting
    // quiet time gives a subsequent clear interval of 0.6 s for reassessment,
    // avoiding an immediate retrigger at the edge of the sensing envelope.
    if (this.remaining > 0) {
      this.quiet = 0;
      this.state = 'Recoiling';
      this.remaining = Math.max(0, this.remaining - seconds);
    } else if (detected.size) this.state = 'Avoiding hazard';
    else if (this.seen.size && this.quiet < 0.6) this.state = 'Reassessing';
    else {
      this.state = 'Reaching';
      this.severity = 0;
      // Rearm encounter detection, retaining lifetime metrics for diagnostics.
      this.seen.clear();
      this.local.clear();
    }
  }
  /**
   * Return a local endpoint request without changing the assigned target.
   *
   * Reads observations from the latest sense call without mutating inputs or
   * response state. This is a request, not a new joint position: ReachController
   * still enforces link lengths, turn limits and solid-obstacle collision checks.
   *
   * @param {number} index Zero-based index of the segment requesting a turn.
   * @param {import('./geometry.js').Point} start Segment attachment in world pixels.
   * @param {import('./geometry.js').Point} end Current segment endpoint in world pixels.
   * @param {import('./geometry.js').Point} goal Neighbour's requested endpoint or tip target.
   * @param {number} length Positive link length in pixels, scaling the response.
   * @returns {import('./geometry.js').Point} Original goal or an aversive endpoint request.
   */
  steer(index, start, end, goal, length) {
    const local = this.local.get(index) ?? [];
    if (!local.length || this.state === 'Reaching') return goal;
    // Mild hazards have no timed recoil: keep requesting outward escape while
    // inside their sensing strip. Stronger hazards retain their timed response.
    const penetrating = local.filter(
      (o) =>
        o.severity < RECOIL_SEVERITY &&
        distance(closestPoint(start, end, o.centre), o.centre) < o.radius + SENSOR_RADIUS,
    );
    if (this.state === 'Recoiling' || penetrating.length) {
      // Let the strongest local stimulus dominate instead of allowing opposing
      // repulsion vectors to cancel. Ties keep the first observation for a
      // deterministic result matching Python's max selection.
      const hazard = (this.state === 'Recoiling' ? local : penetrating).reduce((a, b) =>
        b.severity > a.severity ? b : a,
      );
      const site = closestPoint(start, end, hazard.centre);
      let dx = site[0] - hazard.centre[0],
        dy = site[1] - hazard.centre[1],
        d = Math.hypot(dx, dy);
      // A radial direction is undefined at the centre. Below the 1e-8-pixel
      // tolerance, choose a consistent perpendicular to the link instead. The
      // denominator floor also protects a degenerate zero-length link.
      if (d < 1e-8) {
        dx = end[1] - start[1];
        dy = start[0] - end[0];
        d = Math.max(Math.hypot(dx, dy), 1e-8);
      }
      // Replace attraction with an offset away from the stimulus, scaled from
      // just over one to four link lengths. This increases the requested recoil
      // with severity without bypassing the actual joint turn-speed limits.
      const strength = length * (1 + 3 * hazard.severity);
      return [end[0] + (dx / d) * strength, end[1] + (dy / d) * strength];
    }
    // After recoil (or for mild hazards), reuse boundary steering with a buffer
    // of 35% of detection range. Copies preserve the true hazard geometry; these
    // circles never enter the solid collision guard, so passage remains possible.
    return steerAroundObstacles(
      start,
      goal,
      length,
      local.map((o) => ({ ...o, radius: o.radius + o.detectionRange * 0.35 })),
    )[0];
  }
}
