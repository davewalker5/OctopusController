"""Verify independent assignments, local state and simultaneous progress."""

from math import hypot, pi

import pytest

from octopus_controller.model import Arm, ArmParameters
from octopus_controller.organism import ARM_COUNT, BODY_RADIUS, ArmState, CentralController


def test_eight_separate_radial_arms_reach_together() -> None:
    """Every arm owns its geometry and messages and reaches its own starting target."""
    central = CentralController((400, 410))
    assert len(central.arms) == ARM_COUNT
    assert len({id(arm.controller) for arm in central.arms}) == ARM_COUNT
    assert len({id(arm.controller.arm.angles) for arm in central.arms}) == ARM_COUNT
    assert len({id(arm.controller._requests) for arm in central.arms}) == ARM_COUNT
    for arm in central.arms:
        x, y = arm.controller.arm.base
        assert hypot(x - 400, y - 410) == pytest.approx(BODY_RADIUS)
    for _ in range(2400):
        central.update(1 / 120)
    assert all(arm.state is ArmState.REACHED for arm in central.arms)


def test_other_arms_follow_identical_paths_when_one_assignment_changes() -> None:
    """Retargeting one arm must not influence any other arm's intermediate motion."""
    changed = CentralController((400, 410))
    reference = CentralController((400, 410))
    changed.assign_reach(3, (650, 500))
    for _ in range(120):
        changed.update(1 / 120)
        reference.update(1 / 120)
        for index in range(ARM_COUNT):
            if index != 3:
                assert (
                    changed.arm(index).controller.arm.points
                    == reference.arm(index).controller.arm.points
                )
    assert changed.arm(3).controller.arm.points != reference.arm(3).controller.arm.points


def test_idle_holds_pose_and_resumes_without_reset() -> None:
    """Idle freezes one arm and its messages while a different arm continues."""
    central = CentralController((400, 410))
    for _ in range(40):
        central.update(1 / 120)
    central.assign_idle(0)
    held = central.arm(0)
    before = held.controller.arm.points
    messages = held.controller._requests.copy()
    central.assign_reach(1, (600, 450))
    other_before = central.arm(1).controller.arm.points
    for _ in range(20):
        central.update(1 / 120)
    assert held.state is ArmState.IDLE
    assert held.controller.arm.points == before
    assert held.controller._requests == messages
    assert central.arm(1).controller.arm.points != other_before
    central.assign_reach(0, (550, 200))
    assert held.state is ArmState.REACHING
    assert held.controller.arm.points == before
    for _ in range(30):
        central.update(1 / 120)
    assert held.controller.arm.points != before


def test_reset_geometry_is_local_and_keeps_assignment() -> None:
    """Changing an idle arm's geometry clears only its now-invalid messages."""
    central = CentralController((400, 410))
    central.assign_idle(2)
    other = central.arm(1).controller
    arm = central.arm(2)
    target = arm.controller.target
    arm.reset_pose(ArmParameters(segment_count=15, segment_length=10))
    assert arm.state is ArmState.IDLE
    assert arm.controller.target == target
    assert len(arm.controller._requests) == 15
    assert arm.controller.arm.angles[0] == pytest.approx(0)
    assert central.arm(1).controller is other


@pytest.mark.parametrize("index", [-1, 8, True, 1.5])
def test_invalid_assignments_do_not_change_arms(index: int) -> None:
    """Bad arm IDs cannot accidentally select a different arm through list indexing.

    :param index: Invalid zero-based identity to reject.
    """
    central = CentralController((0, 0))
    targets = [arm.controller.target for arm in central.arms]
    with pytest.raises(ValueError):
        central.assign_reach(index, (0, 0))
    with pytest.raises(ValueError):
        central.assign_idle(index)
    assert [arm.controller.target for arm in central.arms] == targets


def test_invalid_target_and_time_preserve_existing_state() -> None:
    """Validate before activating an idle arm or advancing any controller."""
    central = CentralController((0, 0))
    central.assign_idle(0)
    before = [arm.controller.arm.points for arm in central.arms]
    with pytest.raises(ValueError):
        central.assign_reach(0, (float("nan"), 0))
    assert central.arm(0).state is ArmState.IDLE
    for duration in [-1, float("nan"), float("inf")]:
        with pytest.raises(ValueError):
            central.update(duration)
    assert [arm.controller.arm.points for arm in central.arms] == before


def test_heading_rotates_geometry_without_changing_lengths() -> None:
    """Radial placement rotates the whole initial pose and rejects invalid headings."""
    original = Arm((0, 0))
    rotated = Arm((0, 0), initial_heading=pi / 2)
    for point, turned in zip(original.points, rotated.points):
        assert turned == pytest.approx((-point[1], point[0]))
    with pytest.raises(ValueError):
        Arm((0, 0), initial_heading=float("inf"))
