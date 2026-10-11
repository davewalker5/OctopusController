"""Phase 1: local detection, actual recoil, recovery and permeable hazards."""

from dataclasses import replace
from math import dist

import pytest

from octopus_controller.geometry import closest_point
from octopus_controller.hazards import HazardResponse
from octopus_controller.model import Arm, ReachController
from octopus_controller.scenario import load_scenario, parse_scenario, save_scenario
from octopus_controller.sensing import ObjectKind, WorldObject


def gap(arm, hazard):
    points = arm.points
    return (
        min(
            dist(closest_point(a, b, hazard.centre), hazard.centre)
            for a, b in zip(points, points[1:])
        )
        - hazard.radius
    )


def test_demo_recoils_resumes_and_keeps_goal():
    central, environment = load_scenario("scenarios/local-hazard.json").build()
    arm = central.arm(2)
    target = arm.controller.target
    hazard = environment.objects[-1]
    states = []
    recoil_gaps = []
    initial_other_poses = [a.controller.arm.points for a in central.arms if a is not arm]
    for _ in range(1200):
        central.step(1 / 120, environment)
        states.append(arm.controller.status)
        assert gap(arm.controller.arm, hazard) > 0
        if arm.controller.status == "Recoiling":
            recoil_gaps.append(gap(arm.controller.arm, hazard))
    assert states[0] == "Reaching"
    assert recoil_gaps[-1] > recoil_gaps[0] + 20
    assert "Reassessing" in states
    assert arm.controller.distance <= 2
    assert arm.controller.target == target
    assert arm.controller.hazard.encounters == 1
    assert arm.controller.hazard.exposure_seconds == 0
    assert initial_other_poses == [a.controller.arm.points for a in central.arms if a is not arm]


def test_obstacle_comparison_does_not_recoil():
    central, environment = load_scenario("scenarios/local-obstacle.json").build()
    arm = central.arm(2)
    avoided = False
    for _ in range(1200):
        central.step(1 / 120, environment)
        avoided |= arm.controller.avoiding
        assert arm.controller.hazard.state == "Reaching"
        assert gap(arm.controller.arm, environment.objects[-1]) >= 3 - 1e-8
    assert avoided
    assert arm.controller.distance <= 2


def test_detection_uses_full_segment_and_has_a_finite_range():
    h = WorldObject(1, (50, 20), ObjectKind.HAZARD, 2, 0.5, 15)
    response = HazardResponse()
    response.sense([(0, 0), (100, 0)], (h,), 0.01)
    assert response.state == "Recoiling"  # midpoint; endpoints are out of range
    response = HazardResponse()
    response.sense([(0, 0), (100, 0)], (replace(h, centre=(50, 20.01)),), 0.01)
    assert response.state == "Reaching"
    assert response.encounters == 0


def test_severity_scales_recoil_and_low_severity_only_steers():
    durations = []
    for severity in (0.2, 0.5, 1):
        response = HazardResponse()
        h = WorldObject(1, (10, 5), ObjectKind.HAZARD, 2, severity, 20)
        ticks = 0
        for _ in range(150):
            response.sense([(0, 0), (20, 0)], (h,), 0.01)
            ticks += response.state == "Recoiling"
        durations.append(ticks)
        assert response.state == "Avoiding hazard"
        assert response.encounters == 1
    assert durations[0] == 0 < durations[1] < durations[2]


def test_hazard_is_permeable_and_removal_recovers():
    c = ReachController(Arm((0, 0)), (150, 0))
    # Surround the whole arm and target: this is aversion, not an impassable wall.
    h = WorldObject(1, (150, 0), ObjectKind.HAZARD, 500)
    before = c.arm.angles.copy()
    c.update(1 / 120, (h,))
    assert c.arm.angles != before
    assert not c.blocked
    assert c.hazard.exposure_seconds > 0
    for _ in range(1200):
        c.update(1 / 120)
    assert c.status == "Reached"
    assert c.target == (150, 0)


def test_zero_time_and_restart_do_not_advance_or_retain_episodes(tmp_path):
    scenario = load_scenario("scenarios/local-hazard.json")
    c, e = scenario.build()
    for _ in range(80):
        c.step(1 / 120, e)
    h = c.arm(2).controller.hazard
    before = (h.state, h.encounters, h._remaining)
    c.step(0, e)
    assert before == (h.state, h.encounters, h._remaining)
    saved = tmp_path / "hazard.json"
    save_scenario(saved, c, e)
    assert load_scenario(saved).objects[-1] == scenario.objects[-1]
    fresh, _ = scenario.build()
    assert fresh.arm(2).controller.hazard.encounters == 0


@pytest.mark.parametrize(
    "field,value",
    [
        ("severity", 0),
        ("severity", 1.1),
        ("severity", True),
        ("severity", float("nan")),
        ("detection_range", -1),
        ("detection_range", float("inf")),
        ("detection_range", None),
    ],
)
def test_invalid_hazard_settings(field, value):
    with pytest.raises(ValueError):
        parse_scenario({"version": 1, "hazards": [{"position": [0, 0], field: value}]})


def test_new_hazard_interrupts_arrival_and_food_dwell():
    from octopus_controller.organism import CentralController
    from octopus_controller.sensing import Environment

    central = CentralController((400, 460))
    arm = central.arm(2)
    tip = arm.controller.arm.points[-1]
    arm.assign_reach(tip)
    environment = Environment()
    environment.add(arm.controller.arm.points[-2], ObjectKind.FOOD, 15)
    central.step(1 / 120, environment)
    assert arm.state.value == "Grasping"
    environment.add(tip, ObjectKind.HAZARD)
    central.step(1 / 120, environment)
    assert arm.state.value == "Recoiling"
    assert arm.grip is None
    assert arm.controller.target == tip


def test_distant_hazard_does_not_change_local_motion():
    normal = ReachController(Arm((0, 0)), (150, 0))
    distant = ReachController(Arm((0, 0)), (150, 0))
    hazard = WorldObject(1, (1000, 1000), ObjectKind.HAZARD)
    for _ in range(100):
        normal.update(1 / 120)
        distant.update(1 / 120, (hazard,))
    assert normal.arm.angles == distant.arm.angles
    assert distant.hazard.encounters == 0


def test_unreachable_solid_target_does_not_suppress_local_recoil():
    controller = ReachController(Arm((0, 0)), (150, -100))
    hazard = WorldObject(1, (90, 20), ObjectKind.HAZARD, 2)
    obstacle = WorldObject(2, controller.target, ObjectKind.OBSTACLE, 10)
    before = controller.arm.angles.copy()
    controller.update(1 / 120, (hazard, obstacle))
    assert controller.hazard.state == "Recoiling"
    assert controller.arm.angles != before
    assert controller.target == (150, -100)
