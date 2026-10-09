"""Local segment contact tests against simple circular objects, without Pygame.

Each segment carries a narrow sensing strip along its full length. Treating the
strip as a line with a small radius avoids gaps between point-like suckers and
makes contact at joints and the tip detectable. This module reports geometry only;
obstacle movement constraints belong to the separate avoidance rules.
"""

from dataclasses import dataclass, replace
from enum import Enum
from math import hypot, isfinite

from octopus_controller.geometry import Point, closest_point, validate_point

SENSOR_RADIUS = 3.0
OBJECT_RADIUS = 14.0


class ObjectKind(Enum):
    """Types a local sensor can distinguish in this simplified experiment."""

    FOOD = "Food"
    OBSTACLE = "Obstacle"


@dataclass(frozen=True)
class WorldObject:
    """A circular sensing object with a stable identity, position and type."""

    identifier: int
    centre: Point
    kind: ObjectKind
    radius: float = OBJECT_RADIUS

    def __post_init__(self) -> None:
        """Reject invalid geometry or identities before they enter a sensor reading.

        :raises ValueError: If identity, coordinates, radius or type are invalid.
        """
        if type(self.identifier) is not int or self.identifier < 1:
            raise ValueError("Object identifier must be a positive integer")
        validate_point(self.centre)
        if not isfinite(self.radius) or self.radius <= 0:
            raise ValueError("Object radius must be finite and positive")
        if not isinstance(self.kind, ObjectKind):
            raise ValueError("Object kind must be an ObjectKind")

        # Copy coordinates so a caller's mutable list cannot move an object behind
        # the scene's back while a sensor is reading it.
        object.__setattr__(self, "centre", tuple(self.centre))


@dataclass(frozen=True)
class Contact:
    """One segment's current observation of one object; indices are zero-based."""

    segment_index: int
    object_id: int
    kind: ObjectKind
    position: Point  # Closest point on the segment, used to show the sensing site.


def detect_contacts(
    points: list[Point], objects: tuple[WorldObject, ...], sensor_radius: float = SENSOR_RADIUS
) -> tuple[Contact, ...]:
    """Read each segment independently and return only current contacts.

    :param points: Ordered arm joints in pixels, including the tip.
    :param objects: Immutable snapshot of circular objects in the environment.
    :param sensor_radius: Non-negative reach of the sensing strip in pixels.
    :return: Segment/object observations in segment order, then object order.
    :raises ValueError: If coordinates or sensing radius are invalid.
    """
    if not isfinite(sensor_radius) or sensor_radius < 0:
        raise ValueError("Sensor radius must be finite and non-negative")
    for point in points:
        validate_point(point)

    contacts = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        # Project the object's centre onto this segment. Clamping the fraction
        # to [0, 1] chooses an endpoint when the centre lies beyond either end,
        # rather than accidentally sensing along an infinitely extended line.
        for obj in objects:
            closest = closest_point(start, end, obj.centre)

            # Touching counts as contact. Adding the radii includes the sensing
            # strip's width; the decorative arm thickness is deliberately ignored.
            # A zero-length segment is safely treated as a single sensing point.
            if hypot(obj.centre[0] - closest[0], obj.centre[1] - closest[1]) <= (
                obj.radius + sensor_radius
            ):
                contacts.append(Contact(index, obj.identifier, obj.kind, closest))
    return tuple(contacts)


class Environment:
    """Editable scene with stable object IDs and immutable snapshots for sensing."""

    def __init__(self) -> None:
        """Create an empty scene; removing objects never reuses their IDs."""
        self._objects: dict[int, WorldObject] = {}
        self._next_identifier = 1

    @property
    def objects(self) -> tuple[WorldObject, ...]:
        """Return the current scene in insertion order without exposing its storage."""
        return tuple(self._objects.values())

    def add(self, centre: Point, kind: ObjectKind, radius: float = OBJECT_RADIUS) -> int:
        """Place a sensing object and return its permanent scene identity.

        :param centre: Object centre in world pixels.
        :param kind: Food or obstacle.
        :param radius: Positive circular radius in pixels.
        :return: Newly allocated object ID.
        :raises ValueError: If the object's geometry or kind is invalid.
        """
        obj = WorldObject(self._next_identifier, centre, kind, radius)
        self._objects[obj.identifier] = obj
        self._next_identifier += 1
        return obj.identifier

    def move(self, identifier: int, centre: Point) -> None:
        """Move one object without changing the identity held in sensor readings.

        :param identifier: Existing object ID.
        :param centre: New finite centre in world pixels.
        :raises KeyError: If the object no longer exists.
        :raises ValueError: If the position is invalid.
        """
        self._objects[identifier] = replace(self._objects[identifier], centre=centre)

    def remove(self, identifier: int) -> None:
        """Remove an object; the next sensing refresh clears its contacts.

        :param identifier: Existing object ID.
        :raises KeyError: If the object no longer exists.
        """
        del self._objects[identifier]

    def pick(self, position: Point) -> int | None:
        """Find the topmost object under a pointer, matching the drawing order.

        :param position: Pointer coordinates in world pixels.
        :return: Last drawn object's ID, or None when the pointer hits empty space.
        :raises ValueError: If pointer coordinates are invalid.
        """
        validate_point(position)
        for obj in reversed(self.objects):
            if hypot(position[0] - obj.centre[0], position[1] - obj.centre[1]) <= obj.radius:
                return obj.identifier
        return None
