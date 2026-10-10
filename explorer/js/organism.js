/**
 * High-level arm assignments and capture coordination, corresponding to organism.py.
 *
 * Each arm owns its local requests, observations, dwell timer and attachment.
 * The central layer supplies time and arbitrates completed ownership claims in
 * stable order. All geometry is in world pixels; model angles are radians.
 */

import { distance, validatePoint, validateDuration } from './geometry.js';
import { Arm, armParameters, ReachController } from './model.js';
import { detectContacts } from './sensing.js';
import { Grip, graspCandidate } from './grasping.js';
// Shorter 12-pixel links fit the eight-arm scene; standalone arms default to 18.
export const DEFAULT_PARAMETERS = armParameters({ segmentLength: 12 });
/**
 * Compare contact candidates by value, matching Python tuple equality.
 * @param {[number, number]|null} a Object ID and final segment of a qualifying pair.
 * @param {[number, number]|null} b Another candidate, or null for no candidate.
 * @returns {boolean} Whether the same object and pair are represented.
 */
const sameCandidate = (a, b) =>
  a === b || (a !== null && b !== null && a[0] === b[0] && a[1] === b[1]);
/**
 * One arm's independent assignment, local observations and grasp lifecycle.
 *
 * Reaching changes the tip goal; carrying translates a payload goal into a tip
 * goal. Contacts, ownership and activity remain separate so a blocked arm can
 * still hold food and an idle arm can still sense the environment.
 */
export class ControlledArm {
  /**
   * Create a radial arm with the shared scene defaults and an active reach.
   * @param {import('./geometry.js').Point} base Fixed attachment on the body, in pixels.
   * @param {number} heading Initial outward heading in clockwise radians.
   * @param {import('./geometry.js').Point} target Initial tip objective in pixels.
   * @throws {Error} If base, heading or target geometry is invalid.
   */
  constructor(base, heading, target) {
    this.initialHeading = heading;
    this.controller = new ReachController(new Arm(base, DEFAULT_PARAMETERS, heading), target);
    this.contacts = [];
    // Initialise all grip/dwell fields through the same clearing path used
    // by release(), then restore the initial active reach assignment.
    this.release();
    this.active = true;
  }
  /**
   * Derive the display state without replacing underlying assignment or grip data.
   *
   * Movement constraints take priority over carrying/contact progress. A blocked
   * carry still owns its object, and contact can be displayed for an idle arm.
   * @returns {string} Human-readable local activity state.
   */
  get state() {
    if (this.active && this.controller.blocked) return 'Blocked';
    if (this.active && this.controller.avoiding) return 'Avoiding';
    if (this.grip) return !this.active ? 'Holding' : this.retracting ? 'Retracting' : 'Carrying';
    if (this.candidate) return 'Grasping';
    if (this.contacts.length) return 'Contact';
    if (!this.active) return 'Idle';
    return this.controller.status === 'Reached' ? 'Reached' : 'Reaching';
  }
  /**
   * Assign a destination and restart active work without changing the current pose.
   * @param {import('./geometry.js').Point} target Desired tip or held-food centre in pixels.
   * @returns {void} Clears previous grasp dwell and stale movement flags.
   * @throws {Error} If the target does not contain finite coordinates.
   */
  assignReach(target) {
    validatePoint(target);
    if (this.grip) {
      this.carryGoal = [...target];
      this.retracting = false;
    } else this.controller.setTarget(target);
    this.controller.blocked = this.controller.avoiding = false;
    this.candidate = null;
    this.contactSeconds = 0;
    this.active = true;
  }
  /**
   * Cancel active work while retaining the pose, any grip and old neighbour messages.
   * @returns {void} Clears an incomplete grasp attempt; a new assignment can resume motion.
   */
  idle() {
    this.active = false;
    this.candidate = null;
    this.contactSeconds = 0;
  }
  /**
   * Release food and rebuild this arm from its initial heading.
   * @param {import('./model.js').ArmParameters} [parameters] Validated replacements, or current settings.
   * @returns {void} Preserves the controller's tip target and prior active/idle flag.
   */
  resetPose(parameters = this.controller.arm.parameters) {
    const active = this.active,
      previous = this.controller;
    this.release();
    this.active = active;
    // Old requests refer to the old geometry. Re-seeding the controller is
    // essential when link count, length or allowed bend changes. Other arms
    // keep their own poses and in-flight messages.
    this.controller = new ReachController(
      new Arm(previous.arm.base, parameters, this.initialHeading),
      previous.target,
    );
    this.contacts = [];
  }
  /**
   * Refresh local observations and reconcile external edits, including while paused.
   * @param {import('./sensing.js').WorldObject[]} objects Current immutable scene records.
   * @returns {void} Replaces contacts and clears invalid grips/candidates, but advances no time.
   */
  sense(objects) {
    if (this.grip) {
      const obj = objects.find((obj) => obj.identifier === this.grip.objectId);
      // A missing object or a centre displaced by more than 1e-6 pixels means
      // the scene was edited externally. Release rather than resurrecting it
      // or snapping a dragged object back to its attachment.
      if (!obj || distance(obj.centre, this.grip.centre(this.controller.arm.points)) > 1e-6)
        this.release();
    }
    // Replace, rather than accumulate, observations so a moved object stops
    // being sensed immediately. A lost adjacent pair cancels saved dwell time,
    // even when the user edits the scene without advancing a tick.
    this.contacts = detectContacts(this.controller.arm.points, objects);
    if (
      this.candidate &&
      !sameCandidate(graspCandidate(this.contacts, new Set()), this.candidate)
    ) {
      this.candidate = null;
      this.contactSeconds = 0;
    }
  }
  /**
   * Advance reaching for an active arm, then sense the resulting pose.
   * @param {number} seconds Finite non-negative simulation duration in seconds.
   * @param {import('./sensing.js').WorldObject[]} [objects=[]] Read-only scene snapshot.
   * @returns {void} Does not award grips or move held objects; use updateGrasp for that.
   * @throws {Error} If the duration is invalid.
   */
  update(seconds, objects = []) {
    validateDuration(seconds);
    if (this.active) this.controller.update(seconds, objects);
    this.sense(objects);
  }
  /**
   * Clear attachment, carry goal and grasp dwell, leaving food at its current position.
   * @returns {void} Idles the arm so release does not immediately trigger recapture.
   */
  release() {
    this.grip = this.carryGoal = this.candidate = null;
    this.retracting = false;
    this.contactSeconds = 0;
    this.active = false;
  }
  /**
   * Carry held food towards a point just outside the fixed arm attachment.
   *
   * The destination is 40 pixels outward along the initial arm heading. This
   * uses normal carrying rules; it neither prescribes a pose nor guarantees arrival.
   * @returns {void} Does nothing without a grip.
   */
  retract() {
    if (!this.grip) return;
    const base = this.controller.arm.base;
    this.assignReach([
      base[0] + 40 * Math.cos(this.initialHeading),
      base[1] + 40 * Math.sin(this.initialHeading),
    ]);
    this.retracting = true;
  }
  /**
   * Run one local tick of the contact, capture and carrying lifecycle.
   * @param {number} seconds Finite non-negative simulation duration in seconds.
   * @param {import('./sensing.js').Environment} environment Scene receiving payload moves.
   * @param {Set<number>} unavailable Food identities already held by other arms.
   * @returns {number|null} Newly captured object's ID exactly once, otherwise null.
   * @throws {Error} If the duration is invalid, before state is advanced.
   */
  updateGrasp(seconds, environment, unavailable) {
    validateDuration(seconds);
    if (seconds === 0) return null;
    // Take the snapshot when this arm is visited, not once for all arms. A
    // payload moved by an earlier arm is visible to the later arms this tick.
    const objects = environment.objects;
    this.sense(objects);
    if (this.grip) {
      this.carry(seconds, environment);
      return null;
    }
    const candidate = this.active ? graspCandidate(this.contacts, unavailable) : null;
    if (!candidate) {
      this.candidate = null;
      this.contactSeconds = 0;
      this.update(seconds, objects);
      return null;
    }
    // Hold the pose while the same object and adjacent pair establish a grip.
    // Changing either restarts the dwell; no reaching update runs on this path.
    if (!sameCandidate(candidate, this.candidate)) {
      this.candidate = candidate;
      this.contactSeconds = 0;
    }
    this.contactSeconds += seconds;
    this.controller.blocked = this.controller.avoiding = false;
    // Capture after 0.25 simulation seconds. The tiny time tolerance avoids
    // delaying the boundary by a tick due to repeated floating-point additions.
    if (this.contactSeconds + 1e-12 < 0.25) return null;
    const [id, segment] = candidate;
    this.grip = Grip.attach(
      objects.find((obj) => obj.identifier === id),
      segment,
      this.controller.arm.points,
    );
    // Successful capture holds in place. A later destination or Retract action
    // explicitly starts carrying; CentralController records the new ownership.
    this.candidate = null;
    this.active = false;
    return id;
  }
  /**
   * Attempt to move a latched object using the existing local reach controller.
   * @param {number} seconds Validated positive simulation duration in seconds.
   * @param {import('./sensing.js').Environment} environment Mutable scene containing the payload.
   * @returns {void} Commits a safe arm/payload move or retains the previous pose and grip.
   */
  carry(seconds, environment) {
    const grip = this.grip;
    if (!grip) return;
    const obj = environment.objects.find((obj) => obj.identifier === grip.objectId);
    if (!obj) {
      this.release();
      return;
    }
    if (!this.active || !this.carryGoal) return;
    const points = this.controller.arm.points,
      centre = grip.centre(points);
    // The same two-pixel arrival tolerance applies to the payload centre.
    // Stop motion without dropping the grip when its destination is reached.
    if (distance(centre, this.carryGoal) <= 2) {
      this.active = false;
      this.controller.blocked = this.controller.avoiding = false;
      return;
    }
    // The local solver accepts a tip objective, not a payload objective.
    // Correct for the current tip-to-food offset each tick so the food itself
    // approaches the requested destination without prescribing joint angles.
    const tip = points.at(-1);
    this.controller.setTarget([
      this.carryGoal[0] + tip[0] - centre[0],
      this.carryGoal[1] + tip[1] - centre[1],
    ]);
    const arm = this.controller.arm,
      before = [...arm.angles];
    this.controller.update(seconds, environment.objects);
    if (
      grip.safeMotion(
        arm.base,
        before,
        arm.angles,
        arm.parameters.segmentLength,
        obj.radius,
        environment.objects,
      )
    )
      environment.move(grip.objectId, grip.centre(arm.points));
    else {
      // The arm sweep can be clear while the wider payload hits an obstacle.
      // Roll back angles together with the uncommitted object move, retaining
      // the new neighbour requests so a later tick can try another local turn.
      arm.angles = before;
      this.controller.blocked = true;
    }
    this.sense(environment.objects);
  }
}
/**
 * Route assignments, schedule independent arms and collect exclusive capture claims.
 *
 * Model arm indices are zero-based; UI and scenario numbers are one-based.
 * Shared scheduling does not mean central calculation of each joint's movement.
 */
export class CentralController {
  /**
   * Arrange eight independently controlled arms around a fixed body.
   * @param {import('./geometry.js').Point} centre Body centre in world pixels.
   * @throws {Error} If the body coordinates are invalid.
   */
  constructor(centre) {
    validatePoint(centre);
    this.centre = [...centre];
    this.simulationSeconds = 0;
    this.captureCount = 0;
    this.captureReports = [];
    // Number from the top clockwise. Attachments lie on a 32-pixel body; the
    // initial demo targets sit 180 pixels from each base, offset by -0.3 radians
    // from its outward heading. Scenarios can later replace these assignments.
    this.arms = Array.from({ length: 8 }, (_, index) => {
      const heading = -Math.PI / 2 + (index * 2 * Math.PI) / 8;
      const base = [centre[0] + 32 * Math.cos(heading), centre[1] + 32 * Math.sin(heading)];
      return new ControlledArm(base, heading, [
        base[0] + 180 * Math.cos(heading - 0.3),
        base[1] + 180 * Math.sin(heading - 0.3),
      ]);
    });
  }
  /**
   * Resolve a model arm index without accepting negative or fractional values.
   * @param {number} index Zero-based integer index from 0 to 7.
   * @returns {ControlledArm} The live independently controlled arm.
   * @throws {Error} If the index is invalid.
   */
  arm(index) {
    if (!Number.isInteger(index) || index < 0 || index > 7)
      throw new Error('Arm index must be an integer between 0 and 7');
    return this.arms[index];
  }
  /**
   * Route a high-level destination without specifying segment angles.
   * @param {number} index Zero-based arm index from 0 to 7.
   * @param {import('./geometry.js').Point} target Tip or payload destination in pixels.
   * @returns {void}
   * @throws {Error} If the arm index or target is invalid.
   */
  assignReach(index, target) {
    this.arm(index).assignReach(target);
  }
  /**
   * Cancel one arm's active task without interrupting the other seven.
   * @param {number} index Zero-based arm index from 0 to 7.
   * @returns {void} Retains an existing grip.
   * @throws {Error} If the arm index is invalid.
   */
  assignIdle(index) {
    this.arm(index).idle();
  }
  /**
   * Run reach/sensing against a snapshot without advancing central time or ownership.
   *
   * A read-only snapshot cannot receive object movements. Use step() with an
   * Environment for normal playback of the full grasp/carry lifecycle.
   * @param {number} seconds Finite non-negative simulation duration.
   * @param {import('./sensing.js').WorldObject[]} [objects=[]] Read-only scene snapshot.
   * @returns {void}
   * @throws {Error} If the duration is invalid, before any arm changes.
   */
  update(seconds, objects = []) {
    validateDuration(seconds);
    this.arms.forEach((arm) => arm.update(seconds, objects));
  }
  /**
   * Refresh every arm after a scene edit without advancing motion or simulation time.
   * @param {import('./sensing.js').WorldObject[]} objects Shared current scene snapshot.
   * @returns {void} Reconciles edited grips and contacts, including during pause.
   */
  refreshSensing(objects) {
    this.arms.forEach((arm) => arm.sense(objects));
  }
  /**
   * Advance the complete simulation, resolving captures in stable arm order.
   * @param {number} seconds Finite non-negative simulation duration in seconds.
   * @param {import('./sensing.js').Environment} environment Shared editable scene.
   * @returns {void} Advances central time, arms, held objects and capture reporting.
   * @throws {Error} If the duration is invalid, before any state changes.
   */
  step(seconds, environment) {
    validateDuration(seconds);
    if (seconds === 0) return;
    this.simulationSeconds += seconds;
    const owners = new Set(this.arms.filter((arm) => arm.grip).map((arm) => arm.grip.objectId));
    this.arms.forEach((arm, index) => {
      const captured = arm.updateGrasp(seconds, environment, owners);
      if (captured !== null) {
        // Reserve a completed claim before visiting the next arm. Two local
        // grips must never independently move the same food object.
        owners.add(captured);
        this.captureCount++;
        this.captureReports.push({
          armIndex: index,
          objectId: captured,
          simulationSeconds: this.simulationSeconds,
        });
        // Bound retained history as Python's deque(maxlen=32) does, while the
        // cumulative capture count continues across evicted reports.
        if (this.captureReports.length > 32) this.captureReports.shift();
      }
    });
    // Earlier arms may have sensed before a later arm moved its payload.
    // Refresh all observations so the rendered contacts match the final scene.
    this.refreshSensing(environment.objects);
  }
  /**
   * Detach food before an explicit scene edit moves or deletes it.
   * @param {number} id Stable object identity; unowned or absent IDs are harmless.
   * @returns {void} Any owning arm releases and becomes idle.
   */
  releaseObject(id) {
    this.arms.forEach((arm) => {
      if (arm.grip?.objectId === id) arm.release();
    });
  }
}
