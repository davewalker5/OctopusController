"""Obstacle detours, swept movement constraints and recovery without central planning."""

from math import hypot, pi

import pytest

from octopus_controller.avoidance import ARM_CLEARANCE, safe_motion, steer_around_obstacles
from octopus_controller.geometry import closest_point
from octopus_controller.model import Arm, ReachController
from octopus_controller.organism import ArmState, CentralController
from octopus_controller.sensing import ObjectKind, WorldObject


def clearance(arm: Arm, obstacle: WorldObject) -> float:
    """Measure the smallest centreline-to-obstacle surface gap across all segments.

    :param arm: Pose to inspect.
    :param obstacle: Circular obstacle in pixels.
    :return: Smallest gap, negative for penetration.
    """
    distances = []
    points = arm.points
    for start, end in zip(points, points[1:]):
        point = closest_point(start, end, obstacle.centre)
        distances.append(hypot(point[0] - obstacle.centre[0], point[1] - obstacle.centre[1]))
    return min(distances) - obstacle.radius


@pytest.mark.parametrize("position,target", [((90, 60), (150, 0)), ((90, 100), (150, 0))])
def test_local_detour_reaches_without_crossing_obstacle(
    position: tuple[float, float], target: tuple[float, float]
) -> None:
    """A local detour reaches a target while retaining clearance and joint limits.

    :param position: Obstacle centre blocking part of the normal approach.
    :param target: Reachable objective beyond the obstruction.
    """
    controller = ReachController(Arm((0, 0)), target)
    obstacle = WorldObject(1, position, ObjectKind.OBSTACLE, 14)
    assert clearance(controller.arm, obstacle) >= ARM_CLEARANCE
    saw_avoidance = False
    for _ in range(1200):
        before = controller.arm.angles.copy()
        controller.update(1 / 120, (obstacle,))
        saw_avoidance |= controller.avoiding
        assert clearance(controller.arm, obstacle) >= ARM_CLEARANCE - 1e-8
        points = controller.arm.points
        assert points[0] == (0, 0)
        for start, end in zip(points, points[1:]):
            assert hypot(end[0] - start[0], end[1] - start[1]) == pytest.approx(18)
        assert all(abs(angle) <= pi / 3 for angle in controller.arm.angles[1:])
        for old, new in zip(before, controller.arm.angles):
            assert abs((new - old + pi) % (2 * pi) - pi) <= 1.5 / 120 + 1e-10
        if controller.status == "Reached":
            break
    assert saw_avoidance
    assert controller.status == "Reached"
    assert controller.target == target


def test_obstacle_changes_a_previously_colliding_approach() -> None:
    """The obstacle rule makes a real difference, not merely a label on an existing path."""
    controller = ReachController(Arm((0, 0)), (150, 0))
    obstacle = WorldObject(1, (90, 60), ObjectKind.OBSTACLE, 14)
    smallest_gap = float("inf")
    for _ in range(120):
        controller.update(1 / 120)
        smallest_gap = min(smallest_gap, clearance(controller.arm, obstacle))
    assert smallest_gap < 0


def test_swept_guard_catches_crossing_between_clear_endpoints() -> None:
    """A long rotating segment cannot jump through a thin circle between samples."""
    obstacle = WorldObject(1, (-10, 0), ObjectKind.OBSTACLE, 0.1)
    assert not safe_motion((0, 0), [-pi / 2], [pi / 2], 20, (obstacle,))
    assert safe_motion((0, 0), [-pi / 2], [pi / 2], 20, ())


def test_enclosed_target_blocks_and_removal_resumes() -> None:
    """An impossible assignment stays intact and recovers when the obstruction is gone."""
    controller = ReachController(Arm((0, 0)), (250, 0))
    obstacle = WorldObject(1, (250, 0), ObjectKind.OBSTACLE, 20)
    before = controller.arm.points
    controller.update(1 / 120, (obstacle,))
    assert controller.status == "Blocked"
    assert controller.arm.points == before
    for _ in range(1200):
        controller.update(1 / 120)
    assert controller.status == "Reached"


def test_dragged_overlap_never_teleports_or_worsens() -> None:
    """Placing an obstacle over an existing segment holds or safely retreats the arm."""
    controller = ReachController(Arm((0, 0)), (250, -30))
    obstacle = WorldObject(1, controller.arm.points[8], ObjectKind.OBSTACLE, 10)
    original = clearance(controller.arm, obstacle)
    for _ in range(40):
        controller.update(1 / 120, (obstacle,))
        assert clearance(controller.arm, obstacle) >= original - 1e-8
        original = clearance(controller.arm, obstacle)
    assert controller.target == (250, -30)


def test_all_obstacles_are_checked_even_when_one_steering_boundary_is_selected() -> None:
    """A safe turn with respect to one circle may still be blocked by another."""
    clear = WorldObject(1, (100, 100), ObjectKind.OBSTACLE, 1)
    crossing = WorldObject(2, (-10, 0), ObjectKind.OBSTACLE, 1)
    assert safe_motion((0, 0), [-pi / 2], [pi / 2], 20, (clear,))
    assert not safe_motion((0, 0), [-pi / 2], [pi / 2], 20, (clear, crossing))


def test_distant_obstacle_does_not_change_local_request() -> None:
    """The steering rule has a finite neighbourhood rather than a global path search."""
    obstacle = WorldObject(1, (500, 0), ObjectKind.OBSTACLE, 20)
    assert steer_around_obstacles((0, 0), (1000, 0), 18, (obstacle,)) == ((1000, 0), False)


def test_blocked_arm_does_not_interrupt_other_arms() -> None:
    """A local obstruction leaves unrelated arms and central assignments unchanged."""
    central = CentralController((0, 0))
    reference = CentralController((0, 0))
    target = central.arm(0).controller.target
    obstacle = WorldObject(1, target, ObjectKind.OBSTACLE, 10)
    for _ in range(30):
        central.update(1 / 120, (obstacle,))
        reference.update(1 / 120)
    assert central.arm(0).state is ArmState.BLOCKED
    assert central.arm(0).controller.target == target
    for index in range(1, 8):
        assert (
            central.arm(index).controller.arm.points == reference.arm(index).controller.arm.points
        )
