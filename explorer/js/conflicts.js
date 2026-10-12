/**
 * Explicit arbitration over reported intentions, matching conflicts.py.
 * Priorities are experimental: withdrawal 100, avoidance 80, grip 60, reach 50,
 * discovery 20, idle 0. Positions use world pixels and times simulation seconds.
 * Neither this policy nor the receiver constructs arm poses or reads a scene map.
 * ControlledArm applies the selected action, including safety releases, and its
 * local controller determines the joint movements.
 *
 * @typedef {object} ArmIntent
 * @property {boolean} active Whether a movement task is assigned; idle owners still hold food.
 * @property {number[]} target Fixed tip or payload destination in world pixels.
 * @property {number|null} heldObject Owned food ID, or null when unladen.
 * @property {number|null} goalObject Explicitly assigned food ID, or null for a coordinate task.
 * @property {number|null} candidate Locally qualified grasp candidate ID, or null.
 * @property {boolean} avoiding Local aversion, independent of central withdrawal requests.
 * @property {number|null} discovery Incidental food ignored for the current task, or null.
 */
// Priorities label a fixed hierarchy, not additive weights. Discovery is a
// competing opportunity and cannot replace an explicit food task in this policy.
export const WITHDRAWAL = 100,
  AVOIDANCE = 80,
  GRIP = 60,
  REACH = 50,
  DISCOVERY = 20,
  IDLE = 0;
/**
 * Resolve immutable critical-severity and shared-target settings.
 * The caller compares outstanding P100 reports with critical_severity (0, 1]
 * before supplying the broadcast flag. target_tolerance is an inclusive distance
 * in world pixels, not a physical arm collision radius. Inputs are not changed.
 * @param {object} overrides Optional critical_severity and target_tolerance fields.
 * @returns {object} Frozen validated policy.
 * @throws {Error} For unknown, non-finite or out-of-range settings.
 */
export function conflictPolicy(overrides = {}) {
  const policy = { critical_severity: 0.95, target_tolerance: 2 };
  for (const key of Object.keys(overrides))
    if (!Object.hasOwn(policy, key)) throw new Error(`Unknown conflict field: ${key}`);
  Object.assign(policy, overrides);
  // Number.isFinite rejects booleans and numeric strings without coercion,
  // matching the strict Python policy validation.
  if (!Object.values(policy).every(Number.isFinite))
    throw new Error('Conflict settings must be finite numbers');
  if (
    !(policy.critical_severity > 0 && policy.critical_severity <= 1 && policy.target_tolerance >= 0)
  )
    throw new Error('Require 0 < critical_severity <= 1 and target_tolerance >= 0');
  return Object.freeze(policy);
}
/**
 * Construct an immutable selected behaviour and explanation.
 * @param {string} action Selected behaviour.
 * @param {number} priority Experimental priority value.
 * @param {string} conflict Competing intention category, or empty.
 * @param {string} reason Why the alternative lost.
 * @param {number|null} winner Winning arm index for shared resources.
 * @param {string} scope Local or Central reasoning scope; neither specifies a joint pose.
 * @returns {object} Inspectible decision record.
 */
export function conflictDecision(
  action,
  priority,
  conflict = '',
  reason = '',
  winner = null,
  scope = 'Local',
) {
  return Object.freeze({ action, priority, conflict, reason, winner, scope });
}
/** Stable arbitration, bounded outcome history and cumulative diagnostic counts. */
export class ConflictResolver {
  /**
   * Start empty reservations, decisions and bounded transition history.
   * @param {object} policy Validated immutable configuration; priorities remain fixed.
   */
  constructor(policy = conflictPolicy()) {
    this.policy = policy;
    this.decisions = [];
    this.events = [];
    this.conflictCount = 0;
    // The caller increments this after an actual release, not on every tick
    // that repeats a Withdraw decision.
    this.releaseCount = 0;
    this.wholeWithdrawal = false;
    // Preserve the old resource too: precedence cannot follow an arm when it
    // is reassigned to a different resource.
    this.incumbents = new Map();
  }
  /**
   * Select one action per arm before movement without mutating input intentions.
   * The caller applies the decision: ordinary reaches survive withdrawal, but
   * a released carry is abandoned. No pose, route or grip is changed here.
   * @param {ArmIntent[]} intents Local summaries in stable arm-index order.
   * @param {Set<number>} withdrawing Arms with reported high-priority threats.
   * @param {boolean} whole Whether a critical report requests all arms to retreat.
   * @param {number} seconds Validated central simulation time for event logging.
   * @returns {object[]} Immutable decisions; updates reservations and bounded history.
   */
  resolve(intents, withdrawing, whole, seconds) {
    // Resolve local priorities first. Broadcast withdrawal includes idle arms
    // so the absence of an assignment cannot suppress a safety command.
    const decisions = intents.map((intent, index) => {
      if (whole || withdrawing.has(index)) {
        const conflict =
          intent.heldObject !== null
            ? 'Hold versus withdraw'
            : intent.active
              ? 'Approach versus avoidance'
              : '';
        return conflictDecision(
          'Withdraw',
          WITHDRAWAL,
          conflict,
          intent.heldObject !== null
            ? 'Withdrawal 100 overrides grip 60; release food'
            : intent.active
              ? 'Withdrawal 100 suspends reach 50'
              : 'Withdrawal 100 overrides idle 0',
          null,
          'Central',
        );
      }
      // Active avoidance can redirect a carry while retaining its grip. An
      // inactive owner instead maintains its hold unless withdrawal overrides it.
      if (intent.active && intent.avoiding)
        return conflictDecision(
          'Avoid hazard',
          AVOIDANCE,
          'Approach versus avoidance',
          'Local avoidance 80 overrides reach 50',
        );
      if (intent.heldObject !== null) return conflictDecision('Maintain grip', GRIP);
      if (intent.active)
        return conflictDecision(
          'Reach',
          REACH,
          intent.discovery !== null ? 'Discovery versus task' : '',
          intent.discovery !== null
            ? 'Assigned food task 50 overrides incidental discovery 20'
            : '',
        );
      return conflictDecision('Idle', IDLE);
    });
    // Group from an anchor rather than chaining progressively more distant
    // coordinates. Named food and locally sensed contacts use stable identities.
    const groups = [];
    intents.forEach((intent, index) => {
      // Retreat relinquishes a claim; an idle owner still reserves its food.
      // An idle arm with only an old destination reserves nothing.
      if (decisions[index].action === 'Withdraw' || !(intent.active || intent.heldObject !== null))
        return;
      const group = groups.find((g) => this.sameResource(intent, intents[g[0]]));
      if (group) group.push(index);
      else groups.push([index]);
    });
    const incumbents = new Map();
    for (const group of groups) {
      // Local safety can redirect a carry without relinquishing its owned food.
      const eligible = group.filter(
        (i) => decisions[i].action !== 'Avoid hazard' || intents[i].heldObject !== null,
      );
      if (!eligible.length) continue;
      /**
       * Rank ownership before incumbency, then use arm index for reproducible ties.
       * @param {number} i Eligible arm index in the stable intention array.
       * @returns {number[]} Lexicographic rank; zero outranks one in the first two fields.
       */
      const rank = (i) => [
        Number(intents[i].heldObject === null),
        Number(!(this.incumbents.has(i) && this.sameResource(intents[i], this.incumbents.get(i)))),
        i,
      ];
      // Compare fields in order, matching Python tuple ordering. Sorting only
      // this derived index list leaves the input intentions unchanged.
      eligible.sort((a, b) => {
        const x = rank(a),
          y = rank(b);
        return x[0] - y[0] || x[1] - y[1] || x[2] - y[2];
      });
      const winner = eligible[0];
      // Even an uncontested claim earns precedence over a later newcomer.
      incumbents.set(winner, intents[winner]);
      if (group.length < 2) continue;
      // Preserve the selected action/priority while explaining central contention.
      decisions[winner] = Object.freeze({
        ...decisions[winner],
        conflict: 'Competing arms',
        reason: 'Existing owner, then eligible incumbent, then lowest arm number wins',
        winner,
        scope: 'Central',
      });
      // Waiting inhibits task motion, never local safety steering. Assignments
      // remain with their arms and are reconsidered on the next tick.
      for (const index of group) {
        if (index === winner || decisions[index].action === 'Avoid hazard') continue;
        decisions[index] = conflictDecision(
          'Yield',
          IDLE,
          'Competing arms',
          `Arm ${winner + 1} keeps the resource claim; this arm waits`,
          winner,
          'Central',
        );
      }
    }
    // Replace rather than merge so old, withdrawn or wholly avoiding claims
    // lose their reservation immediately.
    this.incumbents = incumbents;
    this.wholeWithdrawal = whole;
    decisions.forEach((decision, index) => {
      const previous = this.decisions[index];
      // Compare fields, not record identity, so a stable decision is logged once.
      const changed =
        !previous ||
        ['action', 'priority', 'conflict', 'reason', 'winner', 'scope'].some(
          (k) => previous[k] !== decision[k],
        );
      // Include clearance transitions, but count only decisions with a conflict.
      // Routine actions and repeated frames must not flood diagnostic history.
      if (changed && (decision.conflict || previous?.conflict)) {
        this.events.push(Object.freeze({ simulationSeconds: seconds, armIndex: index, decision }));
        this.conflictCount += Number(Boolean(decision.conflict));
      }
    });
    // Keep cumulative totals when older entries leave the bounded display log.
    this.events = this.events.slice(-64);
    this.decisions = decisions;
    return decisions;
  }
  /**
   * Compare local resource claims without looking up food positions centrally.
   * @param {ArmIntent} first Prospective claimant.
   * @param {ArmIntent} second Stable anchor for a resource group.
   * @returns {boolean} Same reported food or nearby coordinate assignment.
   */
  sameResource(first, second) {
    // Actual ownership outranks a named task, which outranks tentative contact.
    // IDs are positive integers; null explicitly denotes an absent claim.
    const a = first.heldObject ?? first.goalObject ?? first.candidate;
    const b = second.heldObject ?? second.goalObject ?? second.candidate;
    // Two IDs are authoritative even at the same location. When either is
    // absent, compare destinations so coordinate-only tasks can also contend
    // with named resources without looking up food in an environment map.
    if (a !== null && b !== null) return a === b;
    return (
      Math.hypot(first.target[0] - second.target[0], first.target[1] - second.target[1]) <=
      this.policy.target_tolerance
    );
  }
}
