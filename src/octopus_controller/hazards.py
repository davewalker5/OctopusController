"""Segment-local aversion. Hazards never participate in collision rejection.

Severity is an experimental value in (0, 1]. Encounters at 0.4 or above
interrupt attraction for a bounded recoil, followed by local boundary steering.
Only sensed circles are remembered; the central assignment is never modified.
Positions and distances use world pixels, with x rightwards and y downwards.
Timers use simulation seconds, so callers must not advance them while paused.
"""

from dataclasses import replace
from math import hypot

from octopus_controller.avoidance import steer_around_obstacles
from octopus_controller.geometry import Point, closest_point
from octopus_controller.sensing import SENSOR_RADIUS, ObjectKind, WorldObject


class HazardResponse:
    """Keep one arm's aversive episode separate from its central assignment.

    State and severity describe the current response. Encounter and exposure
    totals persist across episodes until the controller is rebuilt. Observations
    belong to individual segments; an arm-wide state does not give an unsensing
    segment knowledge of a hazard.
    """

    def __init__(self) -> None:
        """Start with no observations, active recoil or accumulated metrics."""
        self.state = "Reaching"
        self.severity = 0.0
        self.encounters = 0
        self.exposure_seconds = 0.0

        # Remember identities across ticks so continued detection does not
        # restart recoil indefinitely. Local observations retain the geometry
        # last sensed by each segment, rather than tracking unseen movement.
        self._seen: set[int] = set()
        self._remaining = 0.0
        self._quiet = 0.0
        self._local: dict[int, list[WorldObject]] = {}

    @staticmethod
    def nearby(points: list[Point], objects: tuple[WorldObject, ...]) -> bool:
        """Check local hazard proximity without changing episode state or timers.

        This read-only query lets grasping check for aversion before starting a
        food-contact dwell. Inputs are assumed to contain validated geometry.

        :param points: Ordered world-pixel joints, including base and tip.
        :param objects: Immutable snapshot of validated environment objects.
        :return: True if any finite segment is within a hazard's sensing range.
        """
        # Project onto the full finite link: endpoint-only sensing would miss a
        # hazard beside its middle. Include the sensing strip as well as the
        # hazard's detection range; equality counts as detection.
        return any(
            hypot(site[0] - obj.centre[0], site[1] - obj.centre[1])
            <= obj.radius + obj.detection_range + SENSOR_RADIUS
            for start, end in zip(points, points[1:])
            for obj in objects
            if obj.kind is ObjectKind.HAZARD
            for site in [closest_point(start, end, obj.centre)]
        )

    def sense(self, points: list[Point], objects: tuple[WorldObject, ...], seconds: float) -> None:
        """Advance local observations, episode metrics and the aversive state.

        Call once per active movement tick with validated geometry and a positive
        duration. The reach controller validates time and skips zero-time ticks.
        Only this response object is mutated; input poses and objects are read-only.

        :param points: Ordered arm joints in world pixels, including the tip.
        :param objects: Immutable snapshot of validated environment objects.
        :param seconds: Positive elapsed simulation time in seconds.
        :return: None; updates state, severity, metrics and segment memories.
        """
        detected = set()
        severity = 0.0
        exposed = False
        fresh = {}

        # Keep observations per segment, but aggregate identities and severity
        # for the arm's episode. Exposure means centreline penetration of the
        # actual hazard, not merely entry into its larger sensing envelope.
        for index, (start, end) in enumerate(zip(points, points[1:])):
            for obj in objects:
                if obj.kind is not ObjectKind.HAZARD:
                    continue
                site = closest_point(start, end, obj.centre)
                separation = hypot(site[0] - obj.centre[0], site[1] - obj.centre[1])
                if separation <= obj.radius + obj.detection_range + SENSOR_RADIUS:
                    fresh.setdefault(index, []).append(obj)
                    detected.add(obj.identifier)
                    severity = max(severity, obj.severity)
                    exposed |= separation < obj.radius

        # One hazard sensed by several segments is still one encounter. Count
        # exposure once per tick, not once per penetrating segment or hazard.
        new = detected - self._seen
        self.encounters += len(new)
        if exposed:
            self.exposure_seconds += seconds
        if new:
            new_severity = max(obj.severity for obj in objects if obj.identifier in new)

            # These are experimental thresholds: below 0.4, steer without a
            # discrete recoil. Otherwise inhibit attraction for 0.25 + 0.9*s
            # seconds. A new encounter may extend, but never shorten, a recoil.
            if new_severity >= 0.4:
                self._remaining = max(self._remaining, 0.25 + new_severity * 0.9)

        self._seen |= detected

        # Deleted hazards cannot keep steering an arm. For hazards that still
        # exist, retain old observations rather than reading unseen new positions.
        # A fresh reading replaces that segment's entire remembered list; a
        # segment with no fresh reading keeps its memory until the episode ends.
        existing = {obj.identifier: obj for obj in objects if obj.kind is ObjectKind.HAZARD}
        self._local = {
            i: [o for o in observations if o.identifier in existing]
            for i, observations in self._local.items()
        }
        self._local.update(fresh)

        # Preserve the last sensed severity through recoil/reassessment so the
        # inspector explains the response even after the arm leaves sensing range.
        if detected:
            self._quiet = 0.0
            self.severity = severity
        else:
            self._quiet += seconds
        if not existing:
            self._remaining = 0.0

        # Finish a timed recoil even if withdrawal already cleared the hazard.
        # Resetting quiet time here gives a subsequent clear interval of 0.6 s
        # for reassessment, avoiding immediate retriggers at the sensing boundary.
        if self._remaining > 0:
            self._quiet = 0.0
            self.state = "Recoiling"
            self._remaining = max(0.0, self._remaining - seconds)
        elif detected:
            self.state = "Avoiding hazard"
        elif self._seen and self._quiet < 0.6:
            self.state = "Reassessing"
        else:
            self.state = "Reaching"
            self.severity = 0.0

            # Clearing episode memory rearms encounters, while lifetime metrics
            # remain available for comparing repeated runs or prolonged exposure.
            self._seen.clear()
            self._local.clear()

    def steer(self, index: int, start: Point, end: Point, goal: Point, length: float) -> Point:
        """Return a local endpoint request without changing the assigned target.

        Read observations from the latest sense call. The result is a steering
        request, not a new joint position: the reach controller still enforces
        segment lengths, turn limits and collision checks against solid obstacles.
        This method does not mutate the response or its inputs.

        :param index: Zero-based index of the segment requesting a turn.
        :param start: Segment attachment in world pixels.
        :param end: Current segment endpoint in world pixels.
        :param goal: Endpoint requested by its neighbour, or the tip's target.
        :param length: Positive segment length in pixels, scaling the response.
        :return: Original goal or an aversive endpoint request in world pixels.
        """
        local = self._local.get(index, ())
        if not local or self.state == "Reaching":
            return goal
        if self.state == "Recoiling":
            # Let the most severe local stimulus dominate instead of allowing
            # opposing repulsion vectors to cancel. Equal severities retain the
            # first observation, keeping the choice deterministic.
            hazard = max(local, key=lambda obj: obj.severity)
            site = closest_point(start, end, hazard.centre)
            dx, dy = site[0] - hazard.centre[0], site[1] - hazard.centre[1]
            distance = hypot(dx, dy)

            # At the centre, a radial direction is undefined. Below the 1e-8
            # pixel tolerance, use a consistent perpendicular to the link. The
            # denominator floor also avoids division by zero for a degenerate link.
            if distance < 1e-8:
                dx, dy = end[1] - start[1], start[0] - end[0]
                distance = max(hypot(dx, dy), 1e-8)

            # Offset the current endpoint away from the stimulus, replacing
            # attraction for this segment. Severity scales the request from just
            # over one to four link lengths; it does not bypass turn-speed limits.
            strength = length * (1 + 3 * hazard.severity)
            return end[0] + dx / distance * strength, end[1] + dy / distance * strength

        # After recoil (or at low severity), reuse boundary steering with an
        # experimental buffer of 35% of detection range. Copies preserve the
        # real hazard geometry; these circles never enter the solid collision
        # guard, so avoidance remains a preference rather than an impassable wall.
        circles = tuple(
            replace(obj, radius=obj.radius + obj.detection_range * 0.35) for obj in local
        )
        return steer_around_obstacles(start, goal, length, circles)[0]
