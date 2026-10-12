"""Deterministic arbitration, task retention and safety-release regressions."""

from dataclasses import replace
from pathlib import Path

import pytest

from octopus_controller.conflicts import ArmIntent, ConflictPolicy, ConflictResolver
from octopus_controller.grasping import Grip
from octopus_controller.organism import CentralController
from octopus_controller.scenario import load_scenario, parse_scenario, save_scenario
from octopus_controller.sensing import Environment, ObjectKind


def test_priorities_and_stable_resource_claims() -> None:
    """Ownership outranks incumbency, which outranks stable arm numbering."""
    resolver = ConflictResolver()
    task = ArmIntent(True, (100, 100), goal_object=1)
    idle = ArmIntent(False, (0, 0))
    resolver.resolve((idle, task), set(), False, 0.1)
    decisions = resolver.resolve((task, task), set(), False, 0.2)
    assert [d.action for d in decisions] == ["Yield", "Reach"]
    assert decisions[0].winner == 1
    count = resolver.conflict_count
    resolver.resolve((task, task), set(), False, 0.3)
    assert resolver.conflict_count == count
    decisions = resolver.resolve((replace(task, held_object=1), task), set(), False, 0.4)
    assert [d.action for d in decisions] == ["Maintain grip", "Yield"]
    decisions = resolver.resolve((replace(task, avoiding=True), task), set(), False, 0.5)
    assert [d.action for d in decisions] == ["Avoid hazard", "Reach"]
    assert decisions[0].priority == 80
    decisions = resolver.resolve((task, task), {1}, False, 0.6)
    assert [d.action for d in decisions] == ["Reach", "Withdraw"]
    assert decisions[1].priority == 100


def test_target_changes_and_clearance_release_reservations() -> None:
    """An incumbent cannot transfer precedence to a different resource."""
    resolver = ConflictResolver()
    first = ArmIntent(True, (100, 100), goal_object=1)
    second = replace(first, goal_object=2)
    resolver.resolve((first, second), set(), False, 0.1)
    decisions = resolver.resolve((first, first), set(), False, 0.2)
    assert decisions[0].action == "Reach"
    assert decisions[1].action == "Yield"
    decisions = resolver.resolve((replace(first, active=False), first), set(), False, 0.3)
    assert decisions[1].action == "Reach"
    assert decisions[1].conflict == ""
    assert resolver.events[-1].decision.conflict == ""


def test_coordinate_tolerance_and_distinct_food_identity() -> None:
    """Nearby coordinate goals share a claim; distinct named objects do not."""
    resolver = ConflictResolver()
    first = ArmIntent(True, (0, 0))
    assert (
        resolver.resolve((first, replace(first, target=(2, 0))), set(), False, 0)[1].action
        == "Yield"
    )
    assert (
        resolver.resolve((first, replace(first, target=(2.001, 0))), set(), False, 1)[1].action
        == "Reach"
    )
    assert (
        resolver.resolve(
            (replace(first, goal_object=1), replace(first, goal_object=2)), set(), False, 2
        )[1].action
        == "Reach"
    )


def held_scene() -> tuple[CentralController, Environment]:
    """Build a remote food owner and a separate local hazard reporter."""
    central, environment = CentralController((400, 460)), Environment()
    for arm in central.arms:
        arm.idle()
    owner = central.arm(0)
    environment.add(owner.controller.arm.points[3], ObjectKind.FOOD)
    owner.grip = Grip.attach(environment.objects[0], 3, owner.controller.arm.points)
    return central, environment


def test_critical_report_upgrade_releases_remote_carry_once() -> None:
    """A same-priority severity upgrade must reach central arbitration."""
    central, environment = held_scene()
    owner, reporter = central.arm(0), central.arm(4)
    owner.assign_reach((300, 250))
    hazard = environment.add(reporter.controller.arm.points[3], ObjectKind.HAZARD, severity=0.9)
    central.step(1 / 120, environment)
    assert owner.grip is not None
    assert not owner.withdrawal_requested
    before = environment.objects[0].centre
    # Replace the immutable hazard record without changing its identity.
    objects = tuple(
        replace(o, severity=1) if o.identifier == hazard else o for o in environment.objects
    )
    central.update(1 / 120, objects)
    assert owner.grip is None
    assert not owner.active
    assert central.goal == "Withdraw all arms"
    assert all(a.withdrawal_requested for a in central.arms)
    assert central.conflicts.release_count == central.abandoned_goal_count == 1
    assert environment.objects[0].centre == before
    assert any(r.signal.reason == "Critical severity" for r in central.priority.reports)
    central.update(1 / 120, objects)
    assert central.conflicts.release_count == 1
    environment.remove(hazard)
    for _ in range(120):
        central.step(1 / 120, environment)
    assert central.goal == "Continue assignments"
    assert not owner.active and owner.grip is None


def test_unreported_remote_hazard_cannot_trigger_global_withdrawal() -> None:
    """Central decisions use messages, not omniscient environment inspection."""
    central, environment = held_scene()
    environment.add((10000, 10000), ObjectKind.HAZARD, severity=1)
    central.step(1 / 120, environment)
    assert not central.priority.known
    assert central.arm(0).grip is not None
    assert central.conflicts.release_count == 0


def test_named_food_beats_incidental_contact_without_interrupting_dwell() -> None:
    """The lower-ID incidental object must not repeatedly reset focused grasping."""
    central, environment = held_scene()
    arm = central.arm(0)
    arm.release()
    goal = environment.add(environment.objects[0].centre, ObjectKind.FOOD)
    arm.assign_reach(environment.objects[0].centre, goal)
    for _ in range(30):
        central.step(1 / 120, environment)
    assert arm.grip is not None and arm.grip.object_id == goal
    assert central.capture_count == 1
    assert any(e.decision.conflict == "Discovery versus task" for e in central.conflicts.events)


def test_competing_arms_wait_then_resume_their_original_goal() -> None:
    """Yielding neither moves the loser nor erases its active assignment."""
    central, environment = load_scenario(Path("scenarios/competing-arms.json")).build()
    loser = central.arm(3)
    points, target = loser.controller.arm.points, loser.controller.target
    for _ in range(30):
        central.step(1 / 120, environment)
    assert loser.yielding and loser.active and loser.grip is None
    assert loser.controller.arm.points == points
    assert loser.controller.target == target
    central.arm(2).idle()
    central.step(1 / 120, environment)
    assert not loser.yielding
    assert loser.controller.arm.points != points
    assert loser.controller.target == target


def test_pause_restart_and_bounded_history() -> None:
    """Inspection consumes no arbitration time; restart forgets all reservations."""
    scenario = load_scenario(Path("scenarios/competing-arms.json"))
    central, environment = scenario.build()
    central.refresh_sensing(environment.objects)
    central.step(0, environment)
    assert not central.conflicts.events
    central.step(1 / 120, environment)
    assert central.conflicts.conflict_count == 2
    assert scenario.build()[0].conflicts.conflict_count == 0
    resolver = ConflictResolver()
    task = ArmIntent(True, (0, 0))
    for tick in range(100):
        resolver.resolve((task, task), {0} if tick % 2 else set(), False, tick)
    assert len(resolver.events) == 64
    assert resolver.conflict_count > 64


@pytest.mark.parametrize(
    "settings",
    [
        {"critical_severity": 0},
        {"critical_severity": 1.1},
        {"critical_severity": True},
        {"target_tolerance": -1},
        {"target_tolerance": float("nan")},
        {"unknown": 1},
    ],
)
def test_invalid_conflict_settings(settings: dict) -> None:
    """Reject ambiguous priorities before changing a running scene."""
    with pytest.raises(ValueError):
        parse_scenario({"version": 1, "conflicts": settings})


def test_focus_save_preserves_resource_and_fixed_coordinate(tmp_path: Path) -> None:
    """Save/load keeps task identity without following a moved food destination."""
    central, environment = load_scenario(Path("scenarios/discovery-vs-task.json")).build()
    arm = central.arm(2)
    target = arm.controller.target
    environment.move(arm.goal_object, (500, 200))
    path = tmp_path / "saved.json"
    save_scenario(path, central, environment)
    rebuilt, _ = load_scenario(path).build()
    assert rebuilt.arm(2).controller.target == target
    assert rebuilt.arm(2).goal_object == arm.goal_object
    assert rebuilt.conflicts.policy == ConflictPolicy()


def test_local_avoidance_redirects_a_carry_without_transferring_ownership() -> None:
    """Mild local aversion changes motion; only withdrawal forces a release."""
    resolver = ConflictResolver()
    task = ArmIntent(True, (100, 100), goal_object=1)
    decisions = resolver.resolve(
        (task, replace(task, held_object=1, avoiding=True)), set(), False, 0.1
    )
    assert [d.action for d in decisions] == ["Yield", "Avoid hazard"]
    assert decisions[1].priority == 80
    assert decisions[0].winner == 1
    decisions = resolver.resolve(
        (task, replace(task, active=False, held_object=1, avoiding=True)), set(), False, 0.2
    )
    assert [d.action for d in decisions] == ["Yield", "Maintain grip"]
