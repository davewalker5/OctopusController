"""High-level assignments for eight arms, independent of the graphical interface.

The central controller chooses an arm and an objective. Each arm's existing
local controller still handles every segment adjustment. No pose is calculated
here in response to a target; only the initial radial layout belongs here.
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import cos, hypot, isfinite, pi, sin

from octopus_controller.grasping import (
    CARRY_TOLERANCE,
    GRASP_CONTACT_SECONDS,
    Grip,
    grasp_candidate,
)
from octopus_controller.model import Arm, ArmParameters, Point, ReachController, validate_point
from octopus_controller.sensing import Contact, Environment, WorldObject, detect_contacts

ARM_COUNT = 8
BODY_RADIUS = 32.0
DEFAULT_PARAMETERS = ArmParameters(segment_length=12.0)
DEFAULT_TARGET_DISTANCE = 180.0
RETRACT_DISTANCE = 40.0  # Pixels outward from the fixed arm attachment.


class ArmState(Enum):
    """Observable local movement, contact and grasp states."""

    GRASPING = "Grasping"
    HOLDING = "Holding"
    CARRYING = "Carrying"
    RETRACTING = "Retracting"
    AVOIDING = "Avoiding"
    BLOCKED = "Blocked"
    CONTACT = "Contact"
    IDLE = "Idle"
    REACHING = "Reaching"
    REACHED = "Reached"


class ControlledArm:
    """One arm's independent local controller, starting direction and assignment."""

    def __init__(self, base: Point, heading: float, target: Point) -> None:
        """Create a radial arm with an initial reach assignment.

        :param base: Fixed attachment on the body's edge in pixels.
        :param heading: Outward direction in clockwise radians.
        :param target: Initial target in world pixels.
        """
        self.initial_heading = heading
        self.controller = ReachController(Arm(base, DEFAULT_PARAMETERS, heading), target)
        self.active = True
        self.contacts: tuple[Contact, ...] = ()
        self.grip: Grip | None = None
        self.carry_goal: Point | None = None
        self.retracting = False
        self._candidate: tuple[int, int] | None = None
        self._contact_seconds = 0.0

    @property
    def state(self) -> ArmState:
        """Report movement constraints first, then contact and assignment progress."""
        # Blocking does not cancel an assignment. Keep sensing available in the
        # contacts list even when the status explains why movement has stopped.
        if self.active and self.controller.blocked:
            return ArmState.BLOCKED
        if self.active and self.controller.avoiding:
            return ArmState.AVOIDING
        if self.grip is not None:
            if not self.active:
                return ArmState.HOLDING
            return ArmState.RETRACTING if self.retracting else ArmState.CARRYING
        if self._candidate is not None:
            return ArmState.GRASPING
        if self.contacts:
            return ArmState.CONTACT
        if not self.active:
            return ArmState.IDLE
        if self.controller.status == "Reached":
            return ArmState.REACHED
        return ArmState.REACHING

    def assign_reach(self, target: Point) -> None:
        """Assign a tip objective, or an object destination while carrying.

        :param target: Finite desired tip or held-object position in world pixels.
        :raises ValueError: If target coordinates are invalid.
        """
        validate_point(target)
        if self.grip is not None:
            self.carry_goal = tuple(target)
            self.retracting = False
        else:
            self.controller.set_target(target)
        self.controller.blocked = False
        self.controller.avoiding = False
        self._candidate = None
        self._contact_seconds = 0.0
        self.active = True

    def idle(self) -> None:
        """Hold the current pose and any existing grip until another task is assigned."""
        # Keep the old controller and messages, so resuming is a continuation,
        # not a sudden reset. The previous target is hidden while there is no task.
        self.active = False
        self._candidate = None
        self._contact_seconds = 0.0

    def reset_pose(self, parameters: ArmParameters | None = None) -> None:
        """Release held food and rebuild this arm, preserving target and active/idle setting.

        :param parameters: Replacement geometry, or None to keep current settings.
        """
        was_active = self.active
        self.release()
        self.active = was_active
        previous = self.controller
        arm = Arm(
            previous.arm.base,
            parameters if parameters is not None else previous.arm.parameters,
            self.initial_heading,
        )

        # Geometry changes invalidate old neighbour requests. Constructing a fresh
        # controller seeds messages from the new pose, leaving other arms alone.
        self.controller = ReachController(arm, previous.target)
        self.contacts = ()

    def sense(self, objects: tuple[WorldObject, ...]) -> None:
        """Refresh this arm's local observations, including while the arm is idle.

        :param objects: Current environment snapshot in world pixels.
        """
        # Replace readings rather than accumulating them: moving an object away
        # clears contact on the next refresh. Several segments may feel the same
        # object, and one segment may feel several overlapping objects at once.
        if self.grip is not None:
            obj = next((obj for obj in objects if obj.identifier == self.grip.object_id), None)
            expected = self.grip.centre(self.controller.arm.points)
            if (
                obj is None
                or hypot(obj.centre[0] - expected[0], obj.centre[1] - expected[1]) > 1e-6
            ):
                # A scene edit may delete or drag the object while paused. Drop
                # that attachment instead of resurrecting or snapping it back.
                self.release()

        self.contacts = detect_contacts(self.controller.arm.points, objects)
        if self._candidate is not None and grasp_candidate(self.contacts, set()) != self._candidate:
            # A paused scene edit must also cancel an interrupted grip attempt;
            # elapsed contact from an earlier location cannot be saved for later.
            self._candidate = None
            self._contact_seconds = 0.0

    def update(self, elapsed_seconds: float, objects: tuple[WorldObject, ...] = ()) -> None:
        """Advance this arm's local rules only while it has a reach assignment.

        :param elapsed_seconds: Finite non-negative simulation duration in seconds.
        :param objects: Current sensing objects; an empty tuple clears old readings.
        :raises ValueError: If the duration is negative or non-finite.
        """
        if not isfinite(elapsed_seconds) or elapsed_seconds < 0:
            raise ValueError("Elapsed seconds must be finite and non-negative")
        if self.active:
            self.controller.update(elapsed_seconds, objects)

        # Sample the resulting pose so the displayed contacts match the visible arm.
        self.sense(objects)

    def release(self) -> None:
        """Drop any held object in place and idle, preventing immediate recapture."""
        self.grip = None
        self.carry_goal = None
        self.retracting = False
        self._candidate = None
        self._contact_seconds = 0.0

        # A new click explicitly resumes reaching after a deliberate release.
        self.active = False

    def retract(self) -> None:
        """Carry a held object towards a point just outside this arm's attachment."""
        if self.grip is None:
            return
        base = self.controller.arm.base
        self.assign_reach(
            (
                base[0] + RETRACT_DISTANCE * cos(self.initial_heading),
                base[1] + RETRACT_DISTANCE * sin(self.initial_heading),
            )
        )
        self.retracting = True

    def update_grasp(
        self, elapsed_seconds: float, environment: Environment, unavailable: set[int]
    ) -> int | None:
        """Advance local capture or carrying and return a newly captured object's ID.

        :param elapsed_seconds: Finite non-negative simulation duration in seconds.
        :param environment: Mutable scene receiving held-object positions.
        :param unavailable: Object IDs already held by other arms.
        :return: New capture ID once, or None when no new capture occurred.
        :raises ValueError: If the duration is negative or non-finite.
        """
        if not isfinite(elapsed_seconds) or elapsed_seconds < 0:
            raise ValueError("Elapsed seconds must be finite and non-negative")

        if elapsed_seconds == 0:
            return None

        objects = environment.objects
        self.sense(objects)
        if self.grip is not None:
            self._carry(elapsed_seconds, environment)
            return None

        candidate = grasp_candidate(self.contacts, unavailable) if self.active else None
        if candidate is None:
            self._candidate = None
            self._contact_seconds = 0.0
            self.update(elapsed_seconds, objects)
            return None

        # Hold the pose while neighbouring suckers establish a grip. Contact must
        # remain with the same adjacent pair and object for the full dwell time.
        if candidate != self._candidate:
            self._candidate = candidate
            self._contact_seconds = 0.0

        self._contact_seconds += elapsed_seconds
        self.controller.blocked = self.controller.avoiding = False
        if self._contact_seconds + 1e-12 < GRASP_CONTACT_SECONDS:
            return None

        identifier, segment = candidate
        obj = next(obj for obj in objects if obj.identifier == identifier)
        self.grip = Grip.attach(obj, segment, self.controller.arm.points)
        self._candidate = None
        self.active = False
        return identifier

    def _carry(self, elapsed_seconds: float, environment: Environment) -> None:
        """Move a latched object with its segment, committing only a safe payload path.

        :param elapsed_seconds: Positive update duration in seconds.
        :param environment: Mutable scene containing the carried object.
        """
        grip = self.grip
        if grip is None:
            return

        obj = next((obj for obj in environment.objects if obj.identifier == grip.object_id), None)
        if obj is None:
            self.release()
            return

        if not self.active or self.carry_goal is None:
            return

        points = self.controller.arm.points
        centre = grip.centre(points)
        if hypot(centre[0] - self.carry_goal[0], centre[1] - self.carry_goal[1]) <= CARRY_TOLERANCE:
            self.active = False
            self.controller.blocked = self.controller.avoiding = False
            return

        # The local solver steers the tip. Shift its objective by the current
        # tip-to-payload offset so repeated corrections bring the payload itself
        # to the requested destination. No joint angles are centrally prescribed.
        tip = points[-1]
        self.controller.set_target(
            (self.carry_goal[0] + tip[0] - centre[0], self.carry_goal[1] + tip[1] - centre[1])
        )
        arm = self.controller.arm
        before = arm.angles.copy()
        self.controller.update(elapsed_seconds, environment.objects)
        if grip.safe_motion(
            arm.base,
            before,
            arm.angles,
            arm.parameters.segment_length,
            obj.radius,
            environment.objects,
        ):
            environment.move(grip.object_id, grip.centre(arm.points))
        else:
            # Keep the object's attachment and arm consistent when only the wider
            # payload hits an obstacle. Neighbour requests can still evolve.
            arm.angles = before
            self.controller.blocked = True
        self.sense(environment.objects)


@dataclass(frozen=True)
class CaptureReport:
    """One local capture reported centrally, with zero-based arm identity."""

    arm_index: int
    object_id: int
    simulation_seconds: float


class CentralController:
    """Assign independent reach or idle tasks to eight numbered arms.

    Arm indices are zero-based inside the model; the interface displays 1–8.
    All arms receive the same elapsed time, but none reads another arm's state.
    """

    def __init__(self, centre: Point) -> None:
        """Arrange eight arms around a fixed body with separate starting targets.

        :param centre: Body centre in world pixels.
        :raises ValueError: If the centre coordinates are invalid.
        """
        validate_point(centre)
        self.centre = tuple(centre)
        self.simulation_seconds = 0.0
        self.capture_count = 0
        self.capture_reports: deque[CaptureReport] = deque(maxlen=32)
        arms = []
        for index in range(ARM_COUNT):
            # Start numbering at the top and proceed clockwise in equal steps.
            # Shorter segments keep all eight default arms inside the same world
            # used for earlier stages; their local movement rules are unchanged.
            heading = -pi / 2 + index * 2 * pi / ARM_COUNT
            base = (
                centre[0] + BODY_RADIUS * cos(heading),
                centre[1] + BODY_RADIUS * sin(heading),
            )
            target = (
                base[0] + DEFAULT_TARGET_DISTANCE * cos(heading - 0.3),
                base[1] + DEFAULT_TARGET_DISTANCE * sin(heading - 0.3),
            )
            arms.append(ControlledArm(base, heading, target))
        self.arms = tuple(arms)

    def arm(self, index: int) -> ControlledArm:
        """Return one numbered arm, rejecting invalid or negative indices.

        :param index: Zero-based arm number in [0, 7].
        :return: The independently controlled arm.
        :raises ValueError: If the index is not an integer in the allowed range.
        """
        if type(index) is not int or not 0 <= index < ARM_COUNT:
            raise ValueError("Arm index must be an integer between 0 and 7")
        return self.arms[index]

    def assign_reach(self, index: int, target: Point) -> None:
        """Send a high-level reach objective without specifying any segment angles.

        :param index: Zero-based arm number.
        :param target: Desired tip position in world pixels.
        :raises ValueError: If the index or target is invalid.
        """
        self.arm(index).assign_reach(target)

    def assign_idle(self, index: int) -> None:
        """Cancel one arm's active task without interrupting the other seven.

        :param index: Zero-based arm number.
        :raises ValueError: If the index is invalid.
        """
        self.arm(index).idle()

    def update(self, elapsed_seconds: float, objects: tuple[WorldObject, ...] = ()) -> None:
        """Run reach/sensing against a read-only snapshot, without manipulating objects.

        Use step with an Environment for the full grasp/carry lifecycle. A snapshot
        cannot receive object movements or exclusive capture claims.

        :param elapsed_seconds: Finite non-negative duration in seconds.
        :param objects: Shared read-only scene; each arm interprets it locally.
        :raises ValueError: If the duration is invalid; no arm is changed in that case.
        """
        if not isfinite(elapsed_seconds) or elapsed_seconds < 0:
            raise ValueError("Elapsed seconds must be finite and non-negative")
        for arm in self.arms:
            arm.update(elapsed_seconds, objects)

    def refresh_sensing(self, objects: tuple[WorldObject, ...]) -> None:
        """Refresh observations after scene edits without advancing movement or time.

        :param objects: Current environment snapshot, shared read-only with each arm.
        """
        # This also supports inspecting contacts while paused. No assignments are
        # issued in response; the central layer only schedules each local sensor.
        for arm in self.arms:
            arm.sense(objects)

    def step(self, elapsed_seconds: float, environment: Environment) -> None:
        """Advance arms and held scene objects together, collecting local capture reports.

        :param elapsed_seconds: Finite non-negative simulation duration in seconds.
        :param environment: Scene whose food objects can be captured and carried.
        :raises ValueError: If elapsed time is invalid, before any state changes.
        """
        if not isfinite(elapsed_seconds) or elapsed_seconds < 0:
            raise ValueError("Elapsed seconds must be finite and non-negative")
        if elapsed_seconds == 0:
            return
        self.simulation_seconds += elapsed_seconds
        owners = {arm.grip.object_id for arm in self.arms if arm.grip is not None}
        for index, arm in enumerate(self.arms):
            captured = arm.update_grasp(elapsed_seconds, environment, owners)
            if captured is not None:
                # First completed claim wins in stable arm order. This prevents
                # two attachments moving one object
                owners.add(captured)
                self.capture_count += 1
                self.capture_reports.append(CaptureReport(index, captured, self.simulation_seconds))
        self.refresh_sensing(environment.objects)

    def release_object(self, identifier: int) -> None:
        """Detach an object before a direct scene edit moves or deletes it.

        :param identifier: Object ID being manipulated by the user.
        """
        for arm in self.arms:
            if arm.grip is not None and arm.grip.object_id == identifier:
                arm.release()
