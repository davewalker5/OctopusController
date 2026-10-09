"""Check local rules and show that target changes travel through neighbour links."""

from math import hypot, pi

import pytest

from octopus_controller.model import Arm, ArmParameters, ReachController
from octopus_controller.segment_control import request_start, turn_segment, wrap_angle


def test_request_preserves_length_and_neighbour_bend() -> None:
    """A neighbour's proposed heading constrains the requested attachment."""
    request = request_start((0, 0), (0, 100), 0, 0, 18, pi / 6)
    assert request.heading == pytest.approx(pi / 6)
    assert hypot(request.start[0], request.start[1] - 100) == pytest.approx(18)


def test_coincident_goal_keeps_direction() -> None:
    """An endpoint goal at the attachment must not invent a rightward direction."""
    request = request_start((10, 20), (10, 20), pi / 2, None, 18, pi / 3)
    assert request.start == pytest.approx((10, 2))
    assert turn_segment((10, 20), (10, 20), 0, pi / 2, None, 1.5, 0.01) == pi / 2


def test_turn_crosses_angle_seam_by_short_route() -> None:
    """A target just below the negative x axis needs a small positive turn."""
    angle = turn_segment((0, 0), (-100, -1), 0, pi - 0.01, None, 1.5, 0.1)
    assert 0 < wrap_angle(angle - (pi - 0.01)) < 0.03


def test_turn_honours_speed_and_bend_limits() -> None:
    """Even a large request cannot bypass either local movement constraint."""
    assert turn_segment((0, 0), (0, 100), 0, 0, pi / 3, 1.5, 0.01) == pytest.approx(0.015)
    assert turn_segment((0, 0), (0, 100), 0, 0, pi / 6, 1.5, 1) == pytest.approx(pi / 6)


def test_target_change_advances_one_neighbour_per_tick() -> None:
    """Different targets initially affect only the tip, then its immediate neighbour.

    Comparing two identical starting arms isolates the effect of the target from
    any movement caused by requests that were already present in the initial pose.
    """
    first = ReachController(Arm((0, 0)), (200, -100))
    second = ReachController(Arm((0, 0)), (200, 200))
    for ticks in range(1, 6):
        first.update(1 / 120)
        second.update(1 / 120)
        unaffected_count = 20 - ticks
        assert first.arm.angles[:unaffected_count] == second.arm.angles[:unaffected_count]
        assert first._requests[:unaffected_count] == second._requests[:unaffected_count]
        assert first._requests[unaffected_count] != second._requests[unaffected_count]
    assert first.arm.angles != second.arm.angles


def test_zero_time_does_not_transmit_requests() -> None:
    """A paused or zero-duration update cannot advance hidden communication."""
    controller = ReachController(Arm((0, 0)), (100, -200))
    before = controller._requests.copy()
    controller.set_target((-100, 100))
    controller.update(0)
    assert controller._requests == before


@pytest.mark.parametrize("count", [15, 20, 25, 40])
def test_reaching_with_different_segment_counts(count: int) -> None:
    """Neighbour propagation supports both the brief's arm sizes and longer arms.

    :param count: Number of segments in the experimental arm.
    """
    arm = Arm((0, 0), ArmParameters(segment_count=count))
    reach = count * arm.parameters.segment_length
    controller = ReachController(arm, (reach * 0.7, -reach * 0.25))
    for _ in range(3600):
        controller.update(1 / 120)
    assert controller.status == "Reached"


def test_in_range_but_bend_limited_target_stays_connected() -> None:
    """Total length alone cannot determine whether a tightly constrained arm can fold."""
    arm = Arm((0, 0), ArmParameters(segment_count=2, maximum_bend=pi / 36))
    controller = ReachController(arm, (5, 0))
    for _ in range(1200):
        controller.update(1 / 120)
    assert controller.status == "Reaching"
    assert controller.distance > 2
    assert abs(arm.angles[1]) <= pi / 36
    for start, end in zip(arm.points, arm.points[1:]):
        assert hypot(end[0] - start[0], end[1] - start[1]) == pytest.approx(18)
