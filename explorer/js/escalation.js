/**
 * Arm-local priority signals and report-only central knowledge, matching escalation.py.
 * Positions use world pixels; timers use simulation seconds. Central knowledge
 * contains last-reported observations, never a live copy of environment objects.
 * Coordinates follow the screen (x rightwards, y downwards). The signaller
 * classifies observations and the receiver requests broad behaviour; neither
 * computes a retreat pose or moves an arm.
 *
 * @typedef {object} EscalationPolicy
 * @property {number} report_severity Lower reporting threshold in (0, 1].
 * @property {number} withdrawal_severity Higher withdrawal threshold, at most 1.
 * @property {number} exposure_seconds Positive continuous penetration time for promotion.
 * @property {number} clear_seconds Positive continuous absence from sensing for clearance.
 *
 * @typedef {object} HazardSignal
 * @property {number} objectId Stable identity of the locally observed hazard.
 * @property {import('./geometry.js').Point} position Frozen last-reported centre in pixels.
 * @property {number} severity Severity observed when this report was emitted.
 * @property {0|50|100} priority Clear, informational report, or withdrawal request.
 * @property {number} affectedSegments Number of sensing links at report time.
 * @property {number} exposureSeconds Continuous exposure accumulated at report time.
 * @property {string} reason Severity, Sustained exposure, or Clear.
 *
 * @typedef {object} HazardReport
 * @property {number} armIndex Zero-based source arm, distinct from the hazard identity.
 * @property {number} simulationSeconds Central simulation time when the signal was received.
 * @property {HazardSignal} signal Immutable local observation or clearance.
 */
import { closestPoint, distance, validateDuration } from './geometry.js';
import { ObjectKind } from './sensing.js';

/**
 * Resolve experimental policy thresholds and reject invalid values.
 * Settings retain the scenario JSON names so policy conversion has one contract.
 * Input overrides are copied, not modified; each call produces its own policy.
 *
 * @param {Partial<EscalationPolicy>} [overrides={}] Threshold and duration overrides.
 * @returns {EscalationPolicy} Immutable validated policy.
 * @throws {Error} For unknown, non-finite, unordered or non-positive settings.
 */
export function escalationPolicy(overrides = {}) {
  const policy = {
    report_severity: 0.4,
    withdrawal_severity: 0.75,
    exposure_seconds: 2,
    clear_seconds: 0.6,
  };
  // Reject misspellings before merging, rather than silently keeping a default
  // that makes an experiment behave differently from its stated configuration.
  for (const key of Object.keys(overrides)) {
    if (!Object.hasOwn(policy, key)) throw new Error(`Unknown escalation field: ${key}`);
  }
  Object.assign(policy, overrides);
  // Number.isFinite avoids coercing booleans or numeric strings. Strictly
  // ordered severity thresholds leave a non-empty medium band; positive timers
  // prevent immediate exposure promotion or clearance in the same tick.
  if (!Object.values(policy).every(Number.isFinite))
    throw new Error('Escalation settings must be finite numbers');
  if (
    !(
      0 < policy.report_severity &&
      policy.report_severity < policy.withdrawal_severity &&
      policy.withdrawal_severity <= 1
    )
  )
    throw new Error('Require 0 < report_severity < withdrawal_severity <= 1');
  if (policy.exposure_seconds <= 0 || policy.clear_seconds <= 0)
    throw new Error('Escalation durations must be positive');
  // Policy can be shared safely across arms; mutable episode state cannot.
  return Object.freeze(policy);
}

/** One arm's local observations, episode timers and deduplicated notifications. */
export class HazardSignaller {
  /**
   * Start empty per-hazard episodes and current-sensing readouts for one arm.
   * @param {EscalationPolicy} policy Validated immutable scene policy.
   */
  constructor(policy) {
    this.policy = policy;
    // The scene arbiter supplies its critical threshold. A crossing must be
    // reported even when priority was already 100, or central knowledge stays stale.
    this.criticalSeverity = 1;
    // Episodes are local bookkeeping, never central knowledge. Quiet resets on
    // detection; exposure resets outside the region; reported latches priority
    // until a clear interval completes, even if severity drops or position changes.
    this.episodes = new Map();
    this.detectedCount = 0;
    this.severity = 0;
  }
  /**
   * Describe emitted notifications without consulting the central receiver.
   * This is not a current sensor value or a separate delivery acknowledgement:
   * previous reports remain outstanding until this signaller clears them.
   * @returns {string} Withdrawal requested, Reported, Local only, or None.
   */
  get notification() {
    const priorities = [...this.episodes.values()]
      .filter((e) => e.reported)
      .map((e) => e.reported.priority);
    if (priorities.includes(100)) return 'Withdrawal requested';
    return priorities.length ? 'Reported' : this.detectedCount ? 'Local only' : 'None';
  }
  /**
   * Read finite segments and emit new reports, upgrades and clearances only.
   * Medium hazards promote after continuous actual exposure, not mere proximity.
   * High priority latches until clear to avoid chattering when severity changes.
   * Exposure tracks the hazard regardless of severity, but promotion requires
   * its current severity to reach the reporting threshold. Only this signaller's
   * episodes and readouts change; the supplied pose and scene remain untouched.
   * @param {import('./geometry.js').Point[]} points Validated ordered world-pixel joints.
   * @param {import('./sensing.js').WorldObject[]} objects Immutable scene snapshot.
   * @param {number} seconds Non-negative simulation duration; zero changes nothing.
   * @returns {HazardSignal[]} Frozen reports in scene order, then clearances in
   * episode insertion order. The returned array itself is a fresh mutable array.
   * @throws {Error} For negative or non-finite time.
   */
  update(points, objects, seconds) {
    // Validate/skip time before changing state, so a paused refresh cannot
    // accrue exposure, clear central knowledge or emit another notification.
    validateDuration(seconds);
    if (seconds === 0) return [];
    const observations = new Map();
    for (const obj of objects) {
      if (obj.kind !== ObjectKind.HAZARD) continue;
      // Project onto finite links: checking only endpoints would miss hazards
      // beside the middle of a segment whose endpoints are both out of range.
      const distances = points
        .slice(0, -1)
        .map((a, i) => distance(closestPoint(a, points[i + 1], obj.centre), obj.centre));
      // Three pixels matches sensing.js's strip width, not decorative arm width.
      // Detection includes boundary equality; actual exposure instead requires
      // strict centreline penetration of the unexpanded hazard region.
      const count = distances.filter((d) => d <= obj.radius + obj.detectionRange + 3).length;
      if (count)
        observations.set(obj.identifier, [obj, count, Math.min(...distances) < obj.radius]);
    }
    // Current sensing may be empty while an earlier report is still latched.
    // These readouts must not be interpreted as the receiver's active knowledge.
    this.detectedCount = observations.size;
    this.severity = Math.max(0, ...[...observations.values()].map((o) => o[0].severity));
    const messages = [];
    for (const [identifier, [obj, count, exposed]] of observations) {
      if (!this.episodes.has(identifier))
        this.episodes.set(identifier, { quiet: 0, exposure: 0, reported: null });
      const episode = this.episodes.get(identifier);
      episode.quiet = 0;
      // Accrue once per hazard per tick, not once per exposed segment. Leaving
      // the region resets continuous exposure even if proximity persists.
      episode.exposure = exposed ? episode.exposure + seconds : 0;
      let priority = 0,
        reason = 'Severity';
      // 50 informs and 100 requests withdrawal. A local zero emits nothing;
      // it is not a Clear signal. The 1e-12-second tolerance prevents floating-
      // point accumulation error from delaying a threshold crossing by one tick.
      if (obj.severity >= this.policy.report_severity) {
        priority = 50;
        if (obj.severity >= this.policy.withdrawal_severity) priority = 100;
        else if (episode.exposure + 1e-12 >= this.policy.exposure_seconds) {
          priority = 100;
          reason = 'Sustained exposure';
        }
      }
      // Continued sensing neither spams reports nor discloses new coordinates.
      // An upgrade is a fresh observation, and low severity never promotes by time.
      // Comparing with the last report also prevents an in-episode downgrade;
      // only a completed quiet interval releases the notification latch.
      const criticalUpgrade =
        priority === 100 &&
        episode.reported !== null &&
        obj.severity >= this.criticalSeverity &&
        episode.reported.severity < this.criticalSeverity;
      if (priority > (episode.reported?.priority ?? 0) || criticalUpgrade) {
        if (criticalUpgrade) reason = 'Critical severity';
        // Copy and freeze coordinates as well as the record: historical reports
        // must not follow a moved hazard through a shared mutable position array.
        episode.reported = Object.freeze({
          objectId: identifier,
          position: Object.freeze([...obj.centre]),
          severity: obj.severity,
          priority,
          affectedSegments: count,
          exposureSeconds: episode.exposure,
          reason,
        });
        messages.push(episode.reported);
      }
    }
    // Both removal and movement out of range become absence from local sensing.
    // Map iteration permits deleting the current entry without skipping the next.
    for (const [identifier, episode] of this.episodes) {
      if (observations.has(identifier)) continue;
      episode.exposure = 0;
      episode.quiet += seconds;
      if (episode.quiet + 1e-12 >= this.policy.clear_seconds) {
        // Low-only encounters need no Clear because central never knew them.
        // Keep the old observation's fields for traceability; there is no fresh
        // location to report for a hazard that is no longer sensed.
        if (episode.reported)
          messages.push(Object.freeze({ ...episode.reported, priority: 0, reason: 'Clear' }));
        // Forgetting this episode rearms a future encounter with the same ID.
        this.episodes.delete(identifier);
      }
    }
    return messages;
  }
}

/** Central priority and knowledge derived exclusively from explicit messages. */
export class PriorityReceiver {
  /** Start empty knowledge and bounded histories, with cumulative metrics. */
  constructor() {
    // Keys combine arm and hazard identities: one arm clearing a stimulus must
    // not erase another arm's outstanding observation of that same hazard.
    this.known = new Map();
    // Active knowledge is independent of bounded display history. Evicting an
    // old record must not clear a hazard or reduce cumulative diagnostics.
    this.reports = [];
    this.messages = [];
    this.reportCount = 0;
    this.escalationCount = 0;
    this.interventionCount = 0;
  }
  /**
   * Derive intention from outstanding messages without inspecting the scene.
   * @returns {string} Withdraw affected arms if any high-priority report remains;
   * otherwise Continue assignments, including when medium reports exist.
   */
  get goal() {
    return [...this.known.values()].some((r) => r.signal.priority === 100)
      ? 'Withdraw affected arms'
      : 'Continue assignments';
  }
  /**
   * Check whether any report still requests withdrawal for the specified arm.
   * @param {number} index Zero-based arm identity.
   * @returns {boolean} True until all this arm's high-priority reports clear.
   */
  withdrawing(index) {
    return [...this.known.values()].some((r) => r.armIndex === index && r.signal.priority === 100);
  }
  /**
   * Receive an arm's batch before issuing one broad command, never a movement plan.
   * Batch processing prevents a clear message from resuming work while another
   * report still requests withdrawal. Retained history is capped at 64 entries.
   * The caller supplies trusted signaller output once per tick and validated
   * central time. Inputs are read-only; knowledge, histories and counters change.
   * The arm layer acts on the returned command and retains movement autonomy.
   * @param {number} armIndex Zero-based source arm identity.
   * @param {HazardSignal[]} signals New local observations, upgrades and clear messages.
   * @param {number} seconds Current central simulation time for logging.
   * @param {boolean} holding Whether the separate arbiter must resolve a held grip.
   * @returns {boolean} Whether to suspend the arm's ordinary assignment.
   */
  receive(armIndex, signals, seconds, holding) {
    const before = this.withdrawing(armIndex);
    for (const signal of signals) {
      // Receipt time uses the simulation clock, never wall time, for repeatable
      // replay and pause behaviour. Retain clears in history as well as reports.
      const report = Object.freeze({ armIndex, simulationSeconds: seconds, signal });
      this.reports.push(report);
      const key = `${armIndex}:${signal.objectId}`;
      // Only non-clear reports increment the totals. An upgrade is another
      // report, but clearing just removes its active knowledge entry.
      if (signal.priority === 0) this.known.delete(key);
      else {
        this.known.set(key, report);
        this.reportCount++;
        this.escalationCount += Number(signal.priority === 100);
      }
      const label =
        signal.priority === 0
          ? 'CLEAR'
          : `P${signal.priority} severity ${signal.severity.toFixed(2)} ${signal.reason}`;
      this.messages.push(
        Object.freeze({
          simulationSeconds: seconds,
          text: `Arm ${armIndex + 1} -> Central: H${signal.objectId} ${label}`,
        }),
      );
      if (signal.priority === 50)
        this.messages.push(
          Object.freeze({
            simulationSeconds: seconds,
            text: `Central -> Arm ${armIndex + 1}: continue locally`,
          }),
        );
    }
    // Decide once for the complete batch. Two high-priority hazards on one arm
    // require one intervention; clearing just one must not resume its assignment.
    const after = this.withdrawing(armIndex);
    if (after !== before) {
      if (after) this.interventionCount++;
      // Holding changes the explanatory command, not the priority. This layer
      // never releases a grip; the arbiter acts after all arms have reported.
      const action = after
        ? holding
          ? 'resolve grip versus withdrawal'
          : 'withdraw locally'
        : 'resume assignment';
      this.messages.push(
        Object.freeze({
          simulationSeconds: seconds,
          text: `Central -> Arm ${armIndex + 1}: ${action}`,
        }),
      );
    }
    // Match Python's deque(maxlen=64). History limits affect neither active
    // knowledge nor the cumulative counts used for longer experiments.
    this.reports = this.reports.slice(-64);
    this.messages = this.messages.slice(-64);
    return after;
  }
}
