import { distance, validatePoint, validateDuration } from './geometry.js';
import { Arm, armParameters, ReachController } from './model.js';
import { detectContacts } from './sensing.js';
import { Grip, graspCandidate } from './grasping.js';
export const DEFAULT_PARAMETERS = armParameters({ segmentLength: 12 });
const sameCandidate = (a, b) =>
  a === b || (a !== null && b !== null && a[0] === b[0] && a[1] === b[1]);
export class ControlledArm {
  constructor(base, heading, target) {
    this.initialHeading = heading;
    this.controller = new ReachController(new Arm(base, DEFAULT_PARAMETERS, heading), target);
    this.contacts = [];
    this.release();
    this.active = true;
  }
  get state() {
    if (this.active && this.controller.blocked) return 'Blocked';
    if (this.active && this.controller.avoiding) return 'Avoiding';
    if (this.grip) return !this.active ? 'Holding' : this.retracting ? 'Retracting' : 'Carrying';
    if (this.candidate) return 'Grasping';
    if (this.contacts.length) return 'Contact';
    if (!this.active) return 'Idle';
    return this.controller.status === 'Reached' ? 'Reached' : 'Reaching';
  }
  /** Assign a tip destination, or a payload destination when holding food. */
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
  idle() {
    this.active = false;
    this.candidate = null;
    this.contactSeconds = 0;
  }
  /** Rebuild geometry while preserving assignment and active/idle setting. */
  resetPose(parameters = this.controller.arm.parameters) {
    const active = this.active,
      previous = this.controller;
    this.release();
    this.active = active;
    this.controller = new ReachController(
      new Arm(previous.arm.base, parameters, this.initialHeading),
      previous.target,
    );
    this.contacts = [];
  }
  /** Refresh contacts and reconcile edits without advancing dwell or movement. */
  sense(objects) {
    if (this.grip) {
      const obj = objects.find((obj) => obj.identifier === this.grip.objectId);
      if (!obj || distance(obj.centre, this.grip.centre(this.controller.arm.points)) > 1e-6)
        this.release();
    }
    this.contacts = detectContacts(this.controller.arm.points, objects);
    if (
      this.candidate &&
      !sameCandidate(graspCandidate(this.contacts, new Set()), this.candidate)
    ) {
      this.candidate = null;
      this.contactSeconds = 0;
    }
  }
  update(seconds, objects = []) {
    validateDuration(seconds);
    if (this.active) this.controller.update(seconds, objects);
    this.sense(objects);
  }
  /** Drop in place and idle to prevent automatic recapture. */
  release() {
    this.grip = this.carryGoal = this.candidate = null;
    this.retracting = false;
    this.contactSeconds = 0;
    this.active = false;
  }
  retract() {
    if (!this.grip) return;
    const base = this.controller.arm.base;
    this.assignReach([
      base[0] + 40 * Math.cos(this.initialHeading),
      base[1] + 40 * Math.sin(this.initialHeading),
    ]);
    this.retracting = true;
  }
  /** Advance capture dwell or payload motion; report a new claim only once. */
  updateGrasp(seconds, environment, unavailable) {
    validateDuration(seconds);
    if (seconds === 0) return null;
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
    if (!sameCandidate(candidate, this.candidate)) {
      this.candidate = candidate;
      this.contactSeconds = 0;
    }
    this.contactSeconds += seconds;
    this.controller.blocked = this.controller.avoiding = false;
    if (this.contactSeconds + 1e-12 < 0.25) return null;
    const [id, segment] = candidate;
    this.grip = Grip.attach(
      objects.find((obj) => obj.identifier === id),
      segment,
      this.controller.arm.points,
    );
    this.candidate = null;
    this.active = false;
    return id;
  }
  /** Translate a payload goal into a tip goal, committing only a safe sweep. */
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
    if (distance(centre, this.carryGoal) <= 2) {
      this.active = false;
      this.controller.blocked = this.controller.avoiding = false;
      return;
    }
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
      arm.angles = before;
      this.controller.blocked = true;
    }
    this.sense(environment.objects);
  }
}
export class CentralController {
  constructor(centre) {
    validatePoint(centre);
    this.centre = [...centre];
    this.simulationSeconds = 0;
    this.captureCount = 0;
    this.captureReports = [];
    this.arms = Array.from({ length: 8 }, (_, index) => {
      const heading = -Math.PI / 2 + (index * 2 * Math.PI) / 8;
      const base = [centre[0] + 32 * Math.cos(heading), centre[1] + 32 * Math.sin(heading)];
      return new ControlledArm(base, heading, [
        base[0] + 180 * Math.cos(heading - 0.3),
        base[1] + 180 * Math.sin(heading - 0.3),
      ]);
    });
  }
  arm(index) {
    if (!Number.isInteger(index) || index < 0 || index > 7)
      throw new Error('Arm index must be an integer between 0 and 7');
    return this.arms[index];
  }
  assignReach(index, target) {
    this.arm(index).assignReach(target);
  }
  assignIdle(index) {
    this.arm(index).idle();
  }
  update(seconds, objects = []) {
    validateDuration(seconds);
    this.arms.forEach((arm) => arm.update(seconds, objects));
  }
  refreshSensing(objects) {
    this.arms.forEach((arm) => arm.sense(objects));
  }
  /** Visit arms in stable order; later arms see earlier claims and object moves. */
  step(seconds, environment) {
    validateDuration(seconds);
    if (seconds === 0) return;
    this.simulationSeconds += seconds;
    const owners = new Set(this.arms.filter((arm) => arm.grip).map((arm) => arm.grip.objectId));
    this.arms.forEach((arm, index) => {
      const captured = arm.updateGrasp(seconds, environment, owners);
      if (captured !== null) {
        owners.add(captured);
        this.captureCount++;
        this.captureReports.push({
          armIndex: index,
          objectId: captured,
          simulationSeconds: this.simulationSeconds,
        });
        if (this.captureReports.length > 32) this.captureReports.shift();
      }
    });
    this.refreshSensing(environment.objects);
  }
  releaseObject(id) {
    this.arms.forEach((arm) => {
      if (arm.grip?.objectId === id) arm.release();
    });
  }
}
