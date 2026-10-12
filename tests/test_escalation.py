"""Phase 2 reports must originate locally and change broad intention, not poses."""

from dataclasses import replace

import pytest

from octopus_controller.escalation import EscalationPolicy, HazardSignaller, PriorityReceiver
from octopus_controller.grasping import Grip
from octopus_controller.organism import CentralController
from octopus_controller.scenario import load_scenario, parse_scenario, save_scenario
from octopus_controller.sensing import Environment, ObjectKind, WorldObject

POINTS = [(0, 0), (20, 0), (40, 0)]


def hazard(severity=0.5, centre=(20, 5), identifier=1):
    """Make a finite-range stimulus beside the middle of a short test arm."""
    return WorldObject(identifier, centre, ObjectKind.HAZARD, 2, severity, 10)


def test_low_stays_local_and_distant_is_unknown():
    signaller = HazardSignaller(EscalationPolicy())
    receiver = PriorityReceiver()
    for _ in range(20):
        signals = signaller.update(POINTS, (hazard(0.2), hazard(1, (1000, 0), 2)), 0.1)
        assert not receiver.receive(0, signals, 0.1, False)
    assert signaller.notification == "Local only"
    assert signaller.detected_count == 1
    assert not receiver.known
    assert not receiver.messages


def test_medium_reports_once_and_high_upgrade_latches_until_clear():
    signaller = HazardSignaller(EscalationPolicy())
    receiver = PriorityReceiver()
    medium = hazard()
    signals = signaller.update(POINTS, (medium,), 0.1)
    assert signals[0].priority == 50
    assert signals[0].affected_segments == 2
    assert not receiver.receive(2, signals, 0.1, False)
    assert receiver.goal == "Continue assignments"
    # No live map: a moved but still sensed object does not silently change the report.
    moved = replace(medium, centre=(21, 6))
    assert signaller.update(POINTS, (moved,), 0.1) == ()
    assert receiver.known[2, 1].signal.position == medium.centre
    signals = signaller.update(POINTS, (replace(moved, severity=0.9),), 0.1)
    assert receiver.receive(2, signals, 0.3, False)
    assert receiver.goal == "Withdraw affected arms"
    assert receiver.known[2, 1].signal.position == moved.centre
    assert signaller.update(POINTS, (medium,), 0.1) == ()
    assert signaller.notification == "Withdrawal requested"
    assert signaller.update(POINTS, (), 0.59) == ()
    clear = signaller.update(POINTS, (), 0.01)
    assert len(clear) == 1 and clear[0].priority == 0
    assert not receiver.receive(2, clear, 1.0, False)
    assert not receiver.known
    assert receiver.report_count == 2
    assert receiver.escalation_count == receiver.intervention_count == 1
    assert signaller.update(POINTS, (medium,), 0.1)[0].priority == 50


def test_exposure_promotes_medium_but_proximity_and_low_severity_do_not():
    policy = EscalationPolicy(exposure_seconds=0.3)
    signaller = HazardSignaller(policy)
    exposed = hazard(0.5, (20, 0))
    assert signaller.update(POINTS, (exposed,), 0.1)[0].priority == 50
    assert signaller.update(POINTS, (exposed,), 0.1) == ()
    signal = signaller.update(POINTS, (exposed,), 0.1)[0]
    assert signal.priority == 100 and signal.reason == "Sustained exposure"
    for obj in (hazard(), hazard(0.2, (20, 0))):
        local = HazardSignaller(policy)
        messages = [s for _ in range(10) for s in local.update(POINTS, (obj,), 0.1)]
        assert all(s.priority != 100 for s in messages)


def test_exposure_must_be_continuous():
    signaller = HazardSignaller(EscalationPolicy(exposure_seconds=0.3))
    exposed = hazard(0.5, (20, 0))
    signaller.update(POINTS, (exposed,), 0.2)
    signaller.update(POINTS, (hazard(),), 0.1)
    assert not signaller.update(POINTS, (exposed,), 0.2)


def test_multiple_reports_clear_independently_without_premature_resume():
    signaller = HazardSignaller(EscalationPolicy())
    receiver = PriorityReceiver()
    first, second = hazard(0.9), hazard(0.9, identifier=2)
    receiver.receive(0, signaller.update(POINTS, (first, second), 0.1), 0.1, False)
    assert receiver.intervention_count == 1
    assert receiver.receive(0, signaller.update(POINTS, (second,), 0.6), 0.7, False)
    assert not receiver.receive(0, signaller.update(POINTS, (), 0.6), 1.3, False)
    assert receiver.report_count == receiver.escalation_count == 2
    assert receiver.intervention_count == 1


def test_pause_refresh_reset_and_policy_round_trip(tmp_path):
    scenario = load_scenario("scenarios/hazard-escalation.json")
    central, environment = scenario.build()
    assert not central.priority.known
    central.refresh_sensing(environment.objects)
    central.step(0, environment)
    assert not central.priority.messages
    for _ in range(70):
        central.step(1 / 120, environment)
    assert central.priority.escalation_count > 0
    arm = central.arm(2)
    target = arm.controller.target
    arm.reset_pose()
    assert arm.controller.target == target
    assert arm.controller.withdrawal_goal is not None
    path = tmp_path / "saved.json"
    save_scenario(path, central, environment)
    saved = load_scenario(path)
    assert saved.escalation == scenario.escalation
    assert saved.arms[2].target == target
    fresh, _ = scenario.build()
    assert fresh.priority.report_count == 0
    assert fresh.arm(2).signaller.notification == "None"


def test_high_withdraws_locally_then_resumes_original_assignment():
    central, environment = load_scenario("scenarios/hazard-escalation.json").build()
    arm = central.arm(2)
    target = arm.controller.target
    states = set()
    for _ in range(120):
        central.step(1 / 120, environment)
        states.add(arm.state.value)
    assert "Withdrawing" in states
    assert arm.controller.target == target
    assert central.priority.intervention_count > 0
    environment.remove(environment.objects[-1].identifier)
    for _ in range(1500):
        central.step(1 / 120, environment)
    assert not central.priority.known
    assert central.priority.goal == "Continue assignments"
    assert arm.controller.target == target
    assert arm.controller.withdrawal_goal is None
    assert arm.controller.distance <= 2


def test_high_priority_withdrawal_releases_a_held_grip():
    central = CentralController((400, 460))
    for arm in central.arms:
        arm.idle()
    arm = central.arm(2)
    environment = Environment()
    food = environment.add(arm.controller.arm.points[3], ObjectKind.FOOD)
    arm.grip = Grip.attach(environment.objects[0], 3, arm.controller.arm.points)
    environment.move(food, arm.grip.centre(arm.controller.arm.points))
    environment.add(arm.controller.arm.points[3], ObjectKind.HAZARD, severity=0.9)
    before = arm.controller.arm.points
    central.step(0.01, environment)
    assert arm.state.value == "Withdrawing"
    assert arm.grip is None
    assert arm.controller.arm.points != before
    assert central.conflicts.release_count == 1
    assert any("resolve grip versus withdrawal" in m.text for m in central.priority.messages)


def test_idle_arm_withdraws_without_turning_idle_into_a_reach_assignment():
    central = CentralController((400, 460))
    for arm in central.arms:
        arm.idle()
    arm = central.arm(2)
    h = WorldObject(1, arm.controller.arm.points[5], ObjectKind.HAZARD, severity=0.9)
    before = arm.controller.arm.points
    central.update(0.01, (h,))
    assert arm.state.value == "Withdrawing"
    assert arm.controller.arm.points != before
    assert not arm.active
    central.update(0.6)
    assert not arm.withdrawal_requested
    assert not arm.active


def test_receiver_history_is_bounded_but_counts_are_cumulative():
    signaller = HazardSignaller(EscalationPolicy())
    receiver = PriorityReceiver()
    for _ in range(100):
        receiver.receive(0, signaller.update(POINTS, (hazard(0.9),), 0.1), 0.1, False)
        receiver.receive(0, signaller.update(POINTS, (), 0.6), 0.7, False)
    assert len(receiver.reports) == len(receiver.messages) == 64
    assert receiver.report_count == receiver.intervention_count == 100


@pytest.mark.parametrize(
    "settings",
    [
        {"report_severity": 0},
        {"withdrawal_severity": 1.1},
        {"report_severity": 0.8},
        {"report_severity": True},
        {"clear_seconds": 0},
        {"exposure_seconds": -1},
        {"exposure_seconds": float("inf")},
        {"unknown": 1},
    ],
)
def test_invalid_policy_does_not_build_a_scene(settings):
    with pytest.raises(ValueError):
        parse_scenario({"version": 1, "escalation": settings})


@pytest.mark.parametrize("severity,priority", [(0.399, 0), (0.4, 50), (0.749, 50), (0.75, 100)])
def test_priority_threshold_boundaries(severity, priority):
    local = HazardSignaller(EscalationPolicy())
    messages = local.update(POINTS, (hazard(severity),), 0.1)
    assert (messages[0].priority if messages else 0) == priority


def test_unaffected_arms_continue_their_original_motion():
    central, reference = CentralController((0, 0)), CentralController((0, 0))
    affected = central.arm(2)
    stimulus = WorldObject(
        1,
        affected.controller.arm.points[10],
        ObjectKind.HAZARD,
        radius=1,
        severity=0.9,
        detection_range=0,
    )
    central.update(0.01, (stimulus,))
    reference.update(0.01)
    assert affected.withdrawal_requested
    for index in range(8):
        if index != 2:
            assert not central.arm(index).withdrawal_requested
            assert (
                central.arm(index).controller.arm.angles
                == reference.arm(index).controller.arm.angles
            )


def test_invalid_duration_does_not_change_reporting_state():
    local = HazardSignaller(EscalationPolicy())
    for seconds in (-1, float("inf"), float("nan")):
        with pytest.raises(ValueError):
            local.update(POINTS, (hazard(1),), seconds)
    assert local.notification == "None"
    assert local.update(POINTS, (hazard(1),), 0) == ()


def test_priority_messages_are_available_to_configured_logging(caplog):
    local, receiver = HazardSignaller(EscalationPolicy()), PriorityReceiver()
    with caplog.at_level("INFO", logger="octopus_controller.escalation"):
        receiver.receive(0, local.update(POINTS, (hazard(1),), 0.1), 0.1, False)
    assert "Arm 1 -> Central" in caplog.text
    assert "Central -> Arm 1: withdraw locally" in caplog.text
