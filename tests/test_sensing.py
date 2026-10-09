"""Deterministic contact geometry and local observations, independent of drawing."""

from math import hypot

import pytest

from octopus_controller.organism import ArmState, CentralController
from octopus_controller.sensing import Environment, ObjectKind, WorldObject, detect_contacts


@pytest.mark.parametrize(
    "centre,expected",
    [
        ((5, 0), True),
        ((5, 4), True),
        ((5, 4.001), False),
        ((-4, 0), True),
        ((14, 0), True),
        ((15, 0), False),
    ],
)
def test_strip_contact_boundaries(centre: tuple[float, float], expected: bool) -> None:
    """Sense the strip's middle and end caps, including exact tangent contact.

    :param centre: Object centre relative to the 10-pixel test segment.
    :param expected: Whether the one-pixel object touches the three-pixel sensor.
    """
    obj = WorldObject(1, centre, ObjectKind.FOOD, 1)
    assert bool(detect_contacts([(0, 0), (10, 0)], (obj,))) is expected


def test_contacts_identify_segments_objects_and_types() -> None:
    """A shared joint can sense two types without discarding either observation."""
    objects = (
        WorldObject(1, (10, 0), ObjectKind.FOOD, 1),
        WorldObject(2, (10, 1), ObjectKind.OBSTACLE, 1),
    )
    readings = detect_contacts([(0, 0), (10, 0), (20, 0)], objects)
    assert [(item.segment_index, item.object_id) for item in readings] == [
        (0, 1),
        (0, 2),
        (1, 1),
        (1, 2),
    ]
    assert {item.kind for item in readings} == {ObjectKind.FOOD, ObjectKind.OBSTACLE}
    assert all(item.position == (10, 0) for item in readings)


def test_zero_length_segment_and_empty_inputs() -> None:
    """Degenerate geometry is a sensing point, and absent segments produce no readings."""
    obj = WorldObject(1, (2, 3), ObjectKind.FOOD, 1)
    assert len(detect_contacts([(2, 3), (2, 3)], (obj,))) == 1
    assert detect_contacts([], (obj,)) == ()
    assert detect_contacts([(0, 0), (10, 0)], ()) == ()


def test_scene_move_remove_and_stable_identity() -> None:
    """Scene snapshots stay immutable and IDs are never reused after removal."""
    scene = Environment()
    first = scene.add((0, 0), ObjectKind.FOOD)
    second = scene.add((0, 0), ObjectKind.OBSTACLE)
    assert scene.pick((0, 0)) == second
    snapshot = scene.objects
    scene.move(second, (100, 100))
    assert snapshot[1].centre == (0, 0)
    assert scene.pick((0, 0)) == first
    scene.remove(first)
    assert scene.pick((0, 0)) is None
    assert scene.add((0, 0), ObjectKind.FOOD) > second


def test_idle_arm_senses_and_clears_contacts_without_motion() -> None:
    """Contact remains local and disappears when an object is moved or removed."""
    central = CentralController((0, 0))
    central.assign_idle(0)
    arm = central.arm(0)
    scene = Environment()
    identifier = scene.add(arm.controller.arm.points[10], ObjectKind.OBSTACLE, 1)
    before = arm.controller.arm.points
    target = arm.controller.target
    central.refresh_sensing(scene.objects)
    assert arm.state is ArmState.CONTACT
    assert arm.contacts
    assert not arm.active
    assert all(not other.contacts for other in central.arms[1:])
    central.update(1 / 120, scene.objects)
    assert arm.controller.arm.points == before
    assert arm.controller.target == target
    scene.move(identifier, (1000, 1000))
    central.refresh_sensing(scene.objects)
    assert arm.state is ArmState.IDLE
    assert arm.contacts == ()
    scene.move(identifier, before[-1])
    central.refresh_sensing(scene.objects)
    assert arm.contacts
    scene.remove(identifier)
    central.refresh_sensing(scene.objects)
    assert arm.contacts == ()


def test_food_sensing_does_not_reassign_or_steer_arms() -> None:
    """Food remains touchable and does not invoke the obstacle steering rules."""
    sensed = CentralController((0, 0))
    reference = CentralController((0, 0))
    obj = WorldObject(1, (0, 0), ObjectKind.FOOD, 500)
    for _ in range(60):
        sensed.update(1 / 120, (obj,))
        reference.update(1 / 120)
    for arm, other in zip(sensed.arms, reference.arms):
        assert arm.state is ArmState.CONTACT
        assert arm.controller.arm.points == other.controller.arm.points
        assert arm.controller.target == other.controller.target
        for contact in arm.contacts:
            assert hypot(contact.position[0], contact.position[1]) < 500


@pytest.mark.parametrize("radius", [-1, float("nan"), float("inf")])
def test_invalid_sensor_radius(radius: float) -> None:
    """Invalid sensor geometry must fail before producing misleading observations.

    :param radius: Invalid sensor radius in pixels.
    """
    with pytest.raises(ValueError):
        detect_contacts([(0, 0), (10, 0)], (), radius)


@pytest.mark.parametrize("radius", [0, -1, float("nan"), float("inf")])
def test_invalid_object_radius(radius: float) -> None:
    """Objects must occupy a finite positive circular area.

    :param radius: Invalid object radius in pixels.
    """
    with pytest.raises(ValueError):
        WorldObject(1, (0, 0), ObjectKind.FOOD, radius)


def test_invalid_scene_edits_preserve_snapshot() -> None:
    """Reject invalid positions, types and identities at the scene boundary."""
    scene = Environment()
    identifier = scene.add((0, 0), ObjectKind.FOOD)
    before = scene.objects
    with pytest.raises(ValueError):
        scene.move(identifier, (float("nan"), 0))
    with pytest.raises(ValueError):
        scene.add((0, 0), "Food")
    with pytest.raises(ValueError):
        WorldObject(0, (0, 0), ObjectKind.FOOD)
    with pytest.raises(ValueError):
        detect_contacts([(float("inf"), 0)], ())
    assert scene.objects == before
