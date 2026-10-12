"""Local priority messages and report-only central knowledge, independent of rendering.

Distances are world pixels and durations are simulation seconds. The central
receiver never reads scene objects: locations are immutable, last-reported local
observations, not a live map. Coordinates follow the screen (x rightwards,
y downwards). Severity thresholds are experimental policy, not biological claims.
The signaller classifies observations; the receiver requests broad behaviour.
Neither calculates a retreat pose or moves an arm.
"""

import logging
from collections import deque
from dataclasses import dataclass, replace
from math import dist, isfinite

from octopus_controller.geometry import Point, closest_point
from octopus_controller.sensing import SENSOR_RADIUS, ObjectKind, WorldObject


@dataclass(frozen=True)
class EscalationPolicy:
    """Share immutable thresholds without sharing any arm's mutable episode state.

    Severity thresholds lie in (0, 1], with reporting below withdrawal. Exposure
    is continuous time inside a hazard; clear time is continuous absence from
    sensing range. The two timers therefore measure different conditions.
    """

    report_severity: float = 0.4
    withdrawal_severity: float = 0.75
    exposure_seconds: float = 2.0
    clear_seconds: float = 0.6

    def __post_init__(self) -> None:
        """Reject non-finite, unordered or non-positive policy settings.

        :raises ValueError: If thresholds cannot describe the three priority levels.
        """
        values = (
            self.report_severity,
            self.withdrawal_severity,
            self.exposure_seconds,
            self.clear_seconds,
        )
        # bool is an int subclass in Python, but True/False are not meaningful
        # experimental thresholds. Exact numeric types also match JSON validation.
        if any(type(v) not in (int, float) or not isfinite(v) for v in values):
            raise ValueError("Escalation settings must be finite numbers")
        # Strict ordering leaves a non-empty medium band; positive durations
        # prevent immediate exposure promotion or clearance in the same tick.
        if not 0 < self.report_severity < self.withdrawal_severity <= 1:
            raise ValueError("Require 0 < report_severity < withdrawal_severity <= 1")
        if self.exposure_seconds <= 0 or self.clear_seconds <= 0:
            raise ValueError("Escalation durations must be positive")


@dataclass(frozen=True)
class HazardSignal:
    """Capture an observation at reporting time, detached from later sensing.

    Position is the observed hazard centre in world pixels. Segment count and
    exposure seconds describe that observation, not a continuously refreshed
    central estimate. A Clear message retains those fields for traceability;
    priority zero means the previous report is no longer outstanding.
    """

    object_id: int
    position: Point
    severity: float
    priority: int  # 50: informational; 100: withdrawal requested; 0: clear.
    affected_segments: int
    exposure_seconds: float
    reason: str


@dataclass
class _Episode:
    """Track one hazard for one arm; this mutable record never leaves the arm.

    Quiet time resets on any detection. Exposure resets when no segment is
    inside the region. The last emitted report latches the highest priority
    until clearance, even if current severity falls or the object moves.
    """

    quiet: float = 0.0
    exposure: float = 0.0
    reported: HazardSignal | None = None


class HazardSignaller:
    """Detect and deduplicate one arm's messages, including while idle or holding."""

    def __init__(self, policy: EscalationPolicy) -> None:
        """Start an empty local history using the scene's immutable policy.

        :param policy: Validated thresholds shared by the arms.
        """
        self.policy = policy
        # Set by the scene arbiter; crossing this level must refresh an existing
        # P100 report so the centre can distinguish a newly critical threat.
        self.critical_severity = 1.0
        self._episodes: dict[int, _Episode] = {}

        # These readouts describe this tick's sensing, unlike reported priority,
        # which may remain outstanding throughout the quiet clearance interval.
        self.detected_count = 0
        self.severity = 0.0

    @property
    def notification(self) -> str:
        """Describe emitted local notifications without consulting the receiver.

        This is not a fresh sensor value or a separate delivery acknowledgement.
        Reports remain outstanding until this signaller emits their clearances.

        :return: Withdrawal requested, Reported, Local only or None as display text.
        """
        priorities = [e.reported.priority for e in self._episodes.values() if e.reported]
        if 100 in priorities:
            return "Withdrawal requested"
        return "Reported" if priorities else "Local only" if self.detected_count else "None"

    def update(
        self, points: list[Point], objects: tuple[WorldObject, ...], seconds: float
    ) -> tuple[HazardSignal, ...]:
        """Read finite segments and emit only new reports, upgrades or clearances.

        Medium exposure must be continuously inside the actual hazard for the
        configured duration to promote it. High priority stays latched until a
        clear interval, preventing severity changes from chattering commands.
        Low severity never escalates solely because it persists. Exposure tracks
        the hazard regardless of severity, but promotion requires its current
        severity to reach the reporting threshold. Inputs are read-only; only
        this signaller's episodes and current-sensing readouts are changed.

        :param points: Validated ordered world-pixel joints, including the tip.
        :param objects: Immutable environment snapshot, read locally by this arm.
        :param seconds: Non-negative simulation duration; zero changes nothing.
        :return: New reports in scene order, then clears in episode insertion order.
        :raises ValueError: If time is negative or non-finite.
        """
        if not isfinite(seconds) or seconds < 0:
            raise ValueError("Elapsed seconds must be finite and non-negative")

        if seconds == 0:
            return ()

        # Validate/skip time before changing any state. In particular, a paused
        # refresh must not clear knowledge, accrue exposure or emit new reports.
        observations = {}
        for obj in objects:
            if obj.kind is not ObjectKind.HAZARD:
                continue

            # Project onto each finite link so a hazard beside its middle is
            # detectable even when both endpoints lie outside sensing range.
            distances = [
                dist(closest_point(a, b, obj.centre), obj.centre)
                for a, b in zip(points, points[1:])
            ]

            # Sensing includes the strip radius and accepts boundary equality.
            # Exposure instead requires strict centreline penetration of the
            # actual region. Several exposed links still accrue just one tick
            # of exposure time for this hazard, not a sum over all those links.
            count = sum(d <= obj.radius + obj.detection_range + SENSOR_RADIUS for d in distances)
            if count:
                observations[obj.identifier] = (obj, count, min(distances) < obj.radius)

        # Current local sensing can be empty while a previous high-priority
        # report remains latched. Do not mistake these readouts for central state.
        self.detected_count = len(observations)
        self.severity = max((o[0].severity for o in observations.values()), default=0.0)
        messages = []
        for identifier, (obj, count, exposed) in observations.items():
            episode = self._episodes.setdefault(identifier, _Episode())
            episode.quiet = 0.0

            # Leaving the region breaks continuous exposure even if the hazard
            # remains nearby and its notification stays outstanding.
            episode.exposure = episode.exposure + seconds if exposed else 0.0
            priority = 0
            reason = "Severity"

            # Priority values are message categories: 50 informs, 100 requests
            # withdrawal. A local zero here emits nothing; it is not a Clear.
            # The 1e-12-second allowance prevents accumulated floating-point
            # rounding from postponing a threshold crossing by an extra tick.
            if obj.severity >= self.policy.report_severity:
                priority = 50
                if obj.severity >= self.policy.withdrawal_severity:
                    priority = 100
                elif episode.exposure + 1e-12 >= self.policy.exposure_seconds:
                    priority = 100
                    reason = "Sustained exposure"

            # Do not resend the same report every frame or continually disclose
            # moved positions. The centre learns only from explicit messages.
            # Comparing against the last report also prevents downgrades within
            # an episode; only a completed quiet interval releases that latch.
            critical_upgrade = (
                priority == 100
                and episode.reported is not None
                and obj.severity >= self.critical_severity > episode.reported.severity
            )
            if (
                priority > (episode.reported.priority if episode.reported else 0)
                or critical_upgrade
            ):
                if critical_upgrade:
                    reason = "Critical severity"
                episode.reported = HazardSignal(
                    identifier, obj.centre, obj.severity, priority, count, episode.exposure, reason
                )
                messages.append(episode.reported)

        # Snapshot the items because completed episodes are deleted in this loop.
        # Missing sensing covers removal as well as movement out of range: the
        # centre learns about either only through this arm's timed clearance.
        for identifier, episode in list(self._episodes.items()):
            if identifier in observations:
                continue
            episode.exposure = 0.0
            episode.quiet += seconds
            if episode.quiet + 1e-12 >= self.policy.clear_seconds:
                # Never send a clearance for a low-only encounter the receiver
                # never knew about. Preserve the old observation rather than
                # inventing a new position for a hazard no longer sensed.
                if episode.reported:
                    messages.append(replace(episode.reported, priority=0, reason="Clear"))

                # Forgetting the episode rearms reporting for a future encounter
                # with the same stable object identity.
                del self._episodes[identifier]
        return tuple(messages)


@dataclass(frozen=True)
class HazardReport:
    """Pair a local signal with its zero-based arm source and receipt time.

    Simulation time is supplied by the controller, never read from a wall clock,
    so replay and pause behaviour remain deterministic.
    """

    arm_index: int
    simulation_seconds: float
    signal: HazardSignal


@dataclass(frozen=True)
class PriorityMessage:
    """One readable direction of the communication exchange, retained in history."""

    simulation_seconds: float
    text: str


class PriorityReceiver:
    """Central priority and knowledge derived exclusively from received messages."""

    def __init__(self) -> None:
        """Start with no central hazard knowledge, reports or interventions."""
        # Key by arm AND hazard: clearing one arm's observation must not erase
        # another arm's outstanding report about the same environmental object.
        self.known: dict[tuple[int, int], HazardReport] = {}

        # Bound display history independently of active knowledge. Eviction must
        # neither clear a hazard nor reduce the lifetime diagnostic counters.
        self.reports: deque[HazardReport] = deque(maxlen=64)
        self.messages: deque[PriorityMessage] = deque(maxlen=64)
        self.report_count = 0
        self.escalation_count = 0
        self.intervention_count = 0

    @property
    def goal(self) -> str:
        """Derive the broad intention without querying the environment.

        :return: Withdraw affected arms while any high-priority report is active;
            otherwise Continue assignments, including when medium reports exist.
        """
        return (
            "Withdraw affected arms"
            if any(report.signal.priority == 100 for report in self.known.values())
            else "Continue assignments"
        )

    def withdrawing(self, arm_index: int) -> bool:
        """Whether any outstanding report requests withdrawal for this arm.

        :param arm_index: Zero-based reporting arm identity.
        :return: True until all that arm's high-priority reports clear.
        """
        return any(
            index == arm_index and report.signal.priority == 100
            for (index, _), report in self.known.items()
        )

    def _record(self, message: PriorityMessage) -> None:
        """Retain one exchange and expose it to configured application log handlers.

        :param message: Immutable timestamped communication record.
        """
        # Keep UI history and configured logs on the same path. Do not configure
        # handlers here: the embedding application chooses destinations and level.
        self.messages.append(message)
        logging.getLogger(__name__).info("%.3fs %s", message.simulation_seconds, message.text)

    def receive(
        self, arm_index: int, signals: tuple[HazardSignal, ...], seconds: float, holding: bool
    ) -> bool:
        """Accept local messages and return a broad command, never a movement plan.

        Messages for an arm are processed as a batch so clearing one hazard does
        not briefly resume reaching while another hazard still requests withdrawal.
        History is bounded; cumulative counters survive eviction. The caller
        supplies trusted signaller output once per tick and an already validated
        simulation time. Inputs are not modified; receiver knowledge, histories
        and counters are updated. The return value is acted on by the arm layer.

        :param arm_index: Zero-based source arm identity.
        :param signals: New local observations, upgrades and clear messages.
        :param seconds: Current central simulation time for logging.
        :param holding: Whether this arm holds food; the separate arbiter resolves its grip.
        :return: Whether this arm should suspend its ordinary assignment.
        """
        before = self.withdrawing(arm_index)
        for signal in signals:
            report = HazardReport(arm_index, seconds, signal)
            self.reports.append(report)

            # Clears stay in history but remove active knowledge. Only non-clear
            # reports count towards report totals; upgrades also count as reports.
            key = (arm_index, signal.object_id)
            if signal.priority == 0:
                self.known.pop(key, None)
            else:
                self.known[key] = report
                self.report_count += 1
                self.escalation_count += signal.priority == 100
            label = (
                "CLEAR"
                if signal.priority == 0
                else f"P{signal.priority} severity {signal.severity:.2f} {signal.reason}"
            )
            self._record(
                PriorityMessage(
                    seconds, f"Arm {arm_index + 1} -> Central: H{signal.object_id} {label}"
                )
            )
            if signal.priority == 50:
                self._record(
                    PriorityMessage(seconds, f"Central -> Arm {arm_index + 1}: continue locally")
                )

        # Decide after the entire batch: one arm with two high-priority hazards
        # needs only one intervention, and clearing just one cannot resume work.
        after = self.withdrawing(arm_index)
        if after != before:
            if after:
                self.intervention_count += 1

            # Holding changes the explanatory command, not the priority. This
            # receiver never releases a grip; the conflict resolver applies that
            # decision after all arms have sent their reports.
            action = (
                ("resolve grip versus withdrawal" if holding else "withdraw locally")
                if after
                else "resume assignment"
            )
            self._record(PriorityMessage(seconds, f"Central -> Arm {arm_index + 1}: {action}"))
        return after
