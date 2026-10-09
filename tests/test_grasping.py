"""Capture thresholds, exclusive ownership, carrying and obstacle-safe payload motion."""

from math import hypot, pi, sqrt

import pytest

from octopus_controller.app import Application
from octopus_controller.avoidance import safe_motion
from octopus_controller.grasping import GRASP_CONTACT_SECONDS, Grip, grasp_candidate
from octopus_controller.organism import ArmState, CentralController, ControlledArm
from octopus_controller.sensing import Contact, Environment, ObjectKind, WorldObject


def capture_scene() -> tuple[CentralController, Environment]:
    """Return a default radial arm with food across two adjacent sensing segments."""
    central = CentralController((0, 0))
    scene = Environment()
    scene.add(central.arm(0).controller.arm.points[10], ObjectKind.FOOD, 5)
    return central, scene


def test_grasp_requires_adjacent_contacts_on_same_food() -> None:
    """Scattered contacts, different objects and obstacles cannot establish a grip."""
    assert (
        grasp_candidate(
            (Contact(0, 1, ObjectKind.FOOD, (0, 0)), Contact(2, 1, ObjectKind.FOOD, (0, 0))), set()
        )
        is None
    )
    assert (
        grasp_candidate(
            (Contact(0, 1, ObjectKind.FOOD, (0, 0)), Contact(1, 2, ObjectKind.FOOD, (0, 0))), set()
        )
        is None
    )
    obstacle_contacts = tuple(Contact(i, 1, ObjectKind.OBSTACLE, (0, 0)) for i in (0, 1))
    assert grasp_candidate(obstacle_contacts, set()) is None
    food_contacts = tuple(Contact(i, 1, ObjectKind.FOOD, (0, 0)) for i in (0, 1))
    assert grasp_candidate(food_contacts, set()) == (1, 1)
    assert grasp_candidate(food_contacts, {1}) is None


def test_continuous_dwell_capture_and_single_report() -> None:
    """A grip matures in simulation time and capture is reported exactly once."""
    central, scene = capture_scene()
    arm = central.arm(0)
    before = scene.objects[0].centre
    central.step(GRASP_CONTACT_SECONDS / 2, scene)
    assert arm.state is ArmState.GRASPING
    assert central.capture_count == 0
    central.step(0, scene)
    assert arm.grip is None
    central.step(GRASP_CONTACT_SECONDS / 2, scene)
    assert arm.state is ArmState.HOLDING
    assert scene.objects[0].centre == before
    assert arm.grip.centre(arm.controller.arm.points) == pytest.approx(before)
    for _ in range(30):
        central.step(1 / 120, scene)
    assert central.capture_count == 1
    assert central.capture_reports[0].object_id == scene.objects[0].identifier
    assert central.capture_reports[0].arm_index == 0
    assert central.capture_reports[0].simulation_seconds == pytest.approx(GRASP_CONTACT_SECONDS)


def test_lost_contact_restarts_dwell_and_idle_does_not_capture() -> None:
    """Separate short contacts do not add up to a grip, and idle is respected."""
    central, scene = capture_scene()
    original = scene.objects[0].centre
    central.step(0.2, scene)
    scene.move(1, (1000, 1000))
    central.step(1 / 120, scene)
    scene.move(1, original)
    central.step(0.1, scene)
    assert central.arm(0).grip is None
    central.assign_idle(0)
    central.step(1, scene)
    assert central.arm(0).grip is None


def test_two_arms_cannot_claim_one_object() -> None:
    """A stable tie-break grants the first completed claim without duplicate ownership."""
    central, scene = capture_scene()
    first = central.arm(0)
    second = ControlledArm(
        first.controller.arm.base, first.initial_heading, first.controller.target
    )
    central.arms = (first, second) + central.arms[2:]
    central.step(GRASP_CONTACT_SECONDS, scene)
    assert first.grip is not None
    assert second.grip is None
    assert central.capture_count == 1


def test_carry_and_retract_keep_attachment_and_report_count() -> None:
    """The opening food can be moved and brought home while staying attached."""
    application = Application()
    central, scene = application.central, application.environment
    arm = central.arm(0)
    for _ in range(120):
        central.step(1 / 60, scene)
    assert arm.state is ArmState.HOLDING
    centre = scene.objects[0].centre
    goal = (centre[0] + 30, centre[1] + 20)
    central.assign_reach(0, goal)
    for _ in range(1200):
        central.step(1 / 120, scene)
        assert scene.objects[0].centre == pytest.approx(arm.grip.centre(arm.controller.arm.points))
        if arm.state is ArmState.HOLDING:
            break
    assert arm.state is ArmState.HOLDING
    assert hypot(scene.objects[0].centre[0] - goal[0], scene.objects[0].centre[1] - goal[1]) <= 2
    arm.retract()
    assert arm.state is ArmState.RETRACTING
    for _ in range(2400):
        central.step(1 / 120, scene)
        if arm.state is ArmState.HOLDING:
            break
    assert arm.state is ArmState.HOLDING
    assert (
        hypot(
            scene.objects[0].centre[0] - arm.carry_goal[0],
            scene.objects[0].centre[1] - arm.carry_goal[1],
        )
        <= 2
    )
    assert central.capture_count == 1


def test_release_and_scene_edits_cannot_snap_object_back() -> None:
    """Release, external dragging and removal discard the attachment immediately."""
    central, scene = capture_scene()
    central.step(GRASP_CONTACT_SECONDS, scene)
    arm = central.arm(0)
    original = scene.objects[0].centre
    arm.release()
    for _ in range(60):
        central.step(1 / 120, scene)
    assert arm.grip is None
    assert scene.objects[0].centre == original
    central.assign_reach(0, original)
    central.step(GRASP_CONTACT_SECONDS, scene)
    assert arm.grip is not None
    scene.move(1, (1000, 1000))
    central.refresh_sensing(scene.objects)
    assert arm.grip is None
    assert scene.objects[0].centre == (1000, 1000)
    scene.move(1, original)
    central.assign_reach(0, original)
    central.step(GRASP_CONTACT_SECONDS, scene)
    scene.remove(1)
    central.refresh_sensing(scene.objects)
    assert arm.grip is None


def test_payload_sweep_blocks_when_arm_itself_is_clear() -> None:
    """The wider carried object cannot swing through a circle missed by the arm."""
    grip = Grip(1, 0, (0, 20))
    obstacle = WorldObject(2, (0, 20 * sqrt(2)), ObjectKind.OBSTACLE, 1)
    assert safe_motion((0, 0), [0], [pi / 2], 20, (obstacle,))
    assert not grip.safe_motion((0, 0), [0], [pi / 2], 20, 2, (obstacle,))
    assert grip.safe_motion((0, 0), [0], [pi / 2], 20, 2, ())


def test_reset_drops_object_without_moving_it() -> None:
    """Geometry edits must never retain an attachment to an invalid segment index."""
    central, scene = capture_scene()
    central.step(GRASP_CONTACT_SECONDS, scene)
    arm = central.arm(0)
    original = scene.objects[0].centre
    arm.reset_pose()
    assert arm.grip is None
    assert scene.objects[0].centre == original


@pytest.mark.parametrize("duration", [-1, float("nan"), float("inf")])
def test_invalid_step_does_not_change_time_or_scene(duration: float) -> None:
    """Validate a timestep before advancing capture dwell or moving an object.

    :param duration: Invalid simulation duration in seconds.
    """
    central, scene = capture_scene()
    before = scene.objects
    with pytest.raises(ValueError):
        central.step(duration, scene)
    assert central.simulation_seconds == 0
    assert scene.objects == before


def test_blocked_carry_keeps_object_and_grip_consistent() -> None:
    """A destination inside an obstacle cannot detach or relocate the held food."""
    application = Application()
    for _ in range(120):
        application.advance(1 / 60)
    arm = application.central.arm(0)
    original = application.environment.objects[0].centre
    goal = (original[0] + 50, original[1])
    application.environment.add(goal, ObjectKind.OBSTACLE, 20)
    application.central.assign_reach(0, goal)
    for _ in range(60):
        application.advance(1 / 60)
    assert arm.state is ArmState.BLOCKED
    assert arm.grip is not None
    assert application.environment.objects[0].centre == pytest.approx(original)
    assert arm.grip.centre(arm.controller.arm.points) == pytest.approx(original)
