"""Arm geometry independent of Pygame; distances are pixels and angles radians.

Coordinates follow the screen: x increases rightwards and y downwards. Positive
angles therefore turn clockwise. Each joint angle is relative to its parent;
the base angle is relative to the positive x axis and can rotate freely.
"""

from dataclasses import dataclass
from math import cos, hypot, isfinite, pi, sin

from octopus_controller.avoidance import (
    ARM_CLEARANCE,
    safe_motion,
    steer_around_obstacles,
)
from octopus_controller.geometry import Point, pose_points, validate_point
from octopus_controller.segment_control import SegmentRequest, request_start, turn_segment
from octopus_controller.sensing import ObjectKind, WorldObject, detect_contacts

REACH_TOLERANCE = 2.0


@dataclass(frozen=True)
class ArmParameters:
    """Geometry and motion limits for one arm, using pixels and radians."""

    segment_count: int = 20
    segment_length: float = 18.0
    maximum_bend: float = pi / 3
    turning_speed: float = 1.5  # Maximum change per joint per simulation second.

    def __post_init__(self) -> None:
        """Reject parameters that cannot describe a finite articulated arm.

        :raises ValueError: If counts, lengths, angular limits or speeds are invalid.
        """
        if type(self.segment_count) is not int or not 2 <= self.segment_count <= 100:
            raise ValueError("Segment count must be an integer between 2 and 100")
        for value in (self.segment_length, self.maximum_bend, self.turning_speed):
            if not isfinite(value) or value <= 0:
                raise ValueError("Length, bend and speed must be finite and positive")
        if self.maximum_bend > pi:
            raise ValueError("Maximum bend must not exceed pi radians")


class Arm:
    """A fixed base and a chain of rigid segments joined by rotating joints."""

    def __init__(
        self,
        base: Point,
        parameters: ArmParameters = ArmParameters(),
        initial_heading: float = 0.0,
    ) -> None:
        """Create a gently curved starting pose so inward reaches can begin bending.

        :param base: Fixed attachment point in world pixels.
        :param parameters: Segment geometry and joint limits.
        :param initial_heading: Starting base heading in clockwise radians.
        :raises ValueError: If the base or heading is not finite.
        """
        validate_point(base)
        if not isfinite(initial_heading):
            raise ValueError("Initial heading must be finite")
        self.base = tuple(base)
        self.parameters = parameters

        # A perfectly straight arm has no preferred direction to fold when a
        # target lies closer to the base on the same line. A gentle initial curve
        # gives the local rules a consistent bend direction without randomness.
        self.angles = [(initial_heading + pi) % (2 * pi) - pi] + [
            min(0.06, parameters.maximum_bend)
        ] * (parameters.segment_count - 1)

    @property
    def points(self) -> list[Point]:
        """Return fresh joint positions, including base and tip, in world pixels."""
        return pose_points(self.base, self.angles, self.parameters.segment_length)


class ReachController:
    """Local reaching with obstacle steering and swept movement checks.

    Only the final segment receives the target. Each other segment sees its own
    pose and a request from its immediate tip-side neighbour. It asks its own
    base-side neighbour for a useful attachment position, respecting the bend
    allowed between them. Requests travel one segment per update, so a changed
    target influences the arm progressively rather than setting a complete pose.

    The controller schedules these local rules and reports overall progress.
    Earlier stages of development used CCD (cyclic coordinate descent) but,
    unlike that earlier implementation, it does not use the distant tip position
    to calculate every joint's movement. The base stays fixed, and each segment
    turns gradually using its neighbours and its own motion limits.

    Nearby obstacles can redirect a segment's request around a boundary. The
    controller checks that the resulting movement is safe before committing it;
    this guard rejects crossings but does not plan a route through the scene.
    """

    def __init__(self, arm: Arm, target: Point) -> None:
        """Assign a single target to an arm.

        :param arm: Arm whose joint angles this controller will modify.
        :param target: Desired tip position in world pixels.
        """
        self.avoiding = False
        self.blocked = False
        self.arm = arm
        positions = arm.points
        heading = 0.0
        self._requests: list[SegmentRequest] = []
        for index, angle in enumerate(arm.angles):
            heading += angle
            self._requests.append(SegmentRequest(positions[index], heading))
        self.set_target(target)

    def set_target(self, target: Point) -> None:
        """Move the objective without changing the pose or clearing local messages.

        Existing requests remain in flight, so the new target's influence must
        travel back from the tip through the same neighbour links as before.

        :param target: Finite target coordinates in world pixels.
        """
        validate_point(target)
        self.target = tuple(target)
        self.blocked = False
        self.avoiding = False

    @property
    def distance(self) -> float:
        """Return the tip-to-target distance in pixels."""
        tip = self.arm.points[-1]
        return hypot(tip[0] - self.target[0], tip[1] - self.target[1])

    @property
    def status(self) -> str:
        """Describe progress without claiming all in-range targets are reachable."""
        if self.blocked:
            return "Blocked"
        if self.avoiding:
            return "Avoiding"
        if self.distance <= REACH_TOLERANCE:
            return "Reached"

        base = self.arm.base
        reach = self.arm.parameters.segment_count * self.arm.parameters.segment_length

        if hypot(self.target[0] - base[0], self.target[1] - base[1]) > reach:
            return "Beyond reach"

        return "Reaching"

    def update(self, elapsed_seconds: float, objects: tuple[WorldObject, ...] = ()) -> None:
        """Exchange local requests, then apply one bounded turn per segment.

        :param elapsed_seconds: Finite non-negative simulation duration in seconds.
        :param objects: Local environment circles; only obstacles constrain movement.
        :raises ValueError: If the duration is negative or non-finite.
        """
        if not isfinite(elapsed_seconds) or elapsed_seconds < 0:
            raise ValueError("Elapsed seconds must be finite and non-negative")
        if elapsed_seconds == 0:
            return
        obstacles = tuple(obj for obj in objects if obj.kind is ObjectKind.OBSTACLE)
        self.blocked = False
        self.avoiding = False

        # No route can put the tip inside a solid obstacle. Keep the assignment
        # intact so moving the obstacle or target lets the arm resume naturally.
        if any(
            hypot(self.target[0] - obj.centre[0], self.target[1] - obj.centre[1])
            < obj.radius + ARM_CLEARANCE
            for obj in obstacles
        ):
            self.blocked = True
            return
        if self.distance <= REACH_TOLERANCE and not any(
            contact.kind is ObjectKind.OBSTACLE
            for contact in detect_contacts(self.arm.points, obstacles)
        ):
            return

        parameters = self.arm.parameters
        positions = self.arm.points

        # Read the previous messages before writing any new ones. Otherwise a
        # single loop could pass the target through every segment in one update,
        # hiding the gradual neighbour-to-neighbour communication we want to study.
        end_goals = [request.start for request in self._requests[1:]] + [self.target]

        # Redirect only requests whose next few segment lengths meet an obstacle.
        # The target remains unchanged; the altered requests travel through the
        # same neighbour links as ordinary reaching requests.
        for index, goal in enumerate(end_goals):
            end_goals[index], changed = steer_around_obstacles(
                positions[index], goal, parameters.segment_length, obstacles
            )
            self.avoiding |= changed

        next_requests = []
        heading = 0.0
        for index, angle in enumerate(self.arm.angles):
            heading += angle
            neighbour = self._requests[index + 1] if index + 1 < len(self._requests) else None
            next_requests.append(
                request_start(
                    positions[index],
                    end_goals[index],
                    heading,
                    neighbour.heading if neighbour is not None else None,
                    parameters.segment_length,
                    parameters.maximum_bend,
                )
            )

        self._requests = next_requests

        # Pass the actual attachment forward from the fixed base. This is the
        # chain's geometric connection, not a centrally chosen destination: each
        # segment decides its own turn using that attachment and the old request
        # from its next neighbour. No segment reads the distant tip's position.
        previous_angles = self.arm.angles.copy()
        proposed_angles = []
        start = self.arm.base
        parent_heading = 0.0
        for index, end_goal in enumerate(end_goals):
            # The previous segment may have moved our attachment. Recheck this
            # local approach from its new position before deciding our own turn.
            end_goal, changed = steer_around_obstacles(
                start, end_goal, parameters.segment_length, obstacles
            )

            self.avoiding |= changed
            angle = turn_segment(
                start,
                end_goal,
                parent_heading,
                self.arm.angles[index],
                parameters.maximum_bend if index else None,
                parameters.turning_speed,
                elapsed_seconds,
            )
            proposed_angles.append(angle)

            # Build the next attachment from a fixed length so joints cannot
            # separate or segments stretch, even when a request is impossible.
            parent_heading += angle
            start = (
                start[0] + parameters.segment_length * cos(parent_heading),
                start[1] + parameters.segment_length * sin(parent_heading),
            )

        # This geometric guard checks the full swept movement, but never chooses
        # a route. If any segment would cross an obstacle, hold the visible pose.
        # Keep the new requests: neighbours can continue changing their requests
        # on later ticks and may find a turn that the constraints will allow.
        if safe_motion(
            self.arm.base, previous_angles, proposed_angles, parameters.segment_length, obstacles
        ):
            self.arm.angles = proposed_angles
        else:
            self.blocked = True
