"""Deterministic checks of reaching and geometric invariants without a display."""

from math import hypot, pi

import pytest

from octopus_controller.model import REACH_TOLERANCE, Arm, ArmParameters, ReachController


@pytest.mark.parametrize("target", [(250, 100), (100, -150), (-200, 0), (0, 0), (150, 0), (360, 0)])
def test_reaches_targets(target: tuple[float, float]) -> None:
    """Reach representative targets, including inward, behind-base and full extension."""
    controller = ReachController(Arm((0, 0)), target)
    for _ in range(2400):
        controller.update(1 / 120)
    assert controller.distance <= REACH_TOLERANCE
    assert controller.status == "Reached"
    angles = controller.arm.angles.copy()
    controller.update(1 / 120)
    assert controller.arm.angles == angles


@pytest.mark.parametrize("maximum_bend", [pi / 36, pi / 6, pi])
def test_motion_preserves_geometry_and_limits(maximum_bend: float) -> None:
    """Check every intermediate pose, not just the final reachable configuration.

    :param maximum_bend: Joint limit covering tight, moderate and unrestricted bends.
    """
    parameters = ArmParameters(maximum_bend=maximum_bend)
    arm = Arm((17, 32), parameters)
    controller = ReachController(arm, (-180, -120))
    for _ in range(600):
        before = arm.angles.copy()
        controller.update(1 / 120)
        assert arm.points[0] == (17, 32)
        for start, end in zip(arm.points, arm.points[1:]):
            assert hypot(end[0] - start[0], end[1] - start[1]) == pytest.approx(18)
        assert all(abs(angle) <= parameters.maximum_bend for angle in arm.angles[1:])
        for old, new in zip(before, arm.angles):
            change = (new - old + pi) % (2 * pi) - pi
            assert abs(change) <= parameters.turning_speed / 120 + 1e-12


def test_unreachable_target_extends_without_stretching() -> None:
    """An outside target leaves the tip at the maximum radial reach."""
    controller = ReachController(Arm((0, 0)), (500, 0))
    for _ in range(2400):
        controller.update(1 / 120)
    assert controller.distance == pytest.approx(140, abs=0.1)
    assert controller.status == "Beyond reach"


def test_target_change_preserves_pose_and_reaches_again() -> None:
    """Retargeting is an intention change, not an instantaneous pose change."""
    controller = ReachController(Arm((0, 0)), (250, 100))
    for _ in range(600):
        controller.update(1 / 120)
    before = controller.arm.points
    controller.set_target((-100, -200))
    assert controller.arm.points == before
    for _ in range(2400):
        controller.update(1 / 120)
    assert controller.distance <= REACH_TOLERANCE


@pytest.mark.parametrize("value", [-1, float("inf"), float("nan")])
def test_invalid_time_is_rejected(value: float) -> None:
    """Reject time values that would poison all subsequent geometry."""
    controller = ReachController(Arm((0, 0)), (100, 100))
    with pytest.raises(ValueError):
        controller.update(value)


@pytest.mark.parametrize(
    "field,value",
    [
        ("segment_count", 0),
        ("segment_count", 101),
        ("segment_count", 2.5),
        ("segment_count", True),
        ("segment_length", 0),
        ("segment_length", float("inf")),
        ("maximum_bend", 0),
        ("maximum_bend", pi + 0.01),
        ("turning_speed", -1),
        ("turning_speed", float("nan")),
    ],
)
def test_invalid_parameters_are_rejected(field: str, value: float) -> None:
    """Configuration errors fail at the model boundary."""
    with pytest.raises(ValueError):
        ArmParameters(**{field: value})


@pytest.mark.parametrize("point", [(float("nan"), 0), (0, float("inf")), (1,)])
def test_invalid_points_are_rejected(point: tuple[float, ...]) -> None:
    """Neither attachment nor target coordinates may contain invalid geometry."""
    with pytest.raises(ValueError):
        Arm(point)
    with pytest.raises(ValueError):
        ReachController(Arm((0, 0)), point)


def test_zero_time_and_returned_positions_do_not_mutate_arm() -> None:
    """Queries and a zero-duration tick preserve the live simulation."""
    arm = Arm((0, 0))
    controller = ReachController(arm, (100, 100))
    before = arm.points
    controller.update(0)
    positions = arm.points
    positions[0] = (100, 100)
    assert arm.points == before
