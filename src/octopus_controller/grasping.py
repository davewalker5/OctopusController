"""Simple contact-based capture and rigid attachment geometry, independent of Pygame.

Several adjacent contacts establish a grip. Once captured, the object is latched
into one segment's local coordinate frame. This represents a secure hold without
simulating pressure, friction or a deformable object wrapped by several suckers.
"""

from dataclasses import dataclass
from math import atan2, ceil, cos, hypot, sin

from octopus_controller.geometry import Point, pose_points
from octopus_controller.segment_control import wrap_angle
from octopus_controller.sensing import Contact, ObjectKind, WorldObject

GRASP_CONTACT_SECONDS = 0.25
GRASP_SEGMENTS = 2
CARRY_TOLERANCE = 2.0


def grasp_candidate(contacts: tuple[Contact, ...], unavailable: set[int]) -> tuple[int, int] | None:
    """Find a food object touched by enough consecutive segments.

    :param contacts: Current local observations, including object type and identity.
    :param unavailable: Objects already owned by another arm.
    :return: Object ID and last segment of its first qualifying run, or None.
    """
    by_object: dict[int, set[int]] = {}
    for contact in contacts:
        if contact.kind is ObjectKind.FOOD and contact.object_id not in unavailable:
            by_object.setdefault(contact.object_id, set()).add(contact.segment_index)
    # Stable identity order makes simultaneous opportunities deterministic. This
    # is a tie-break only; richer competition between goals has not been implemented
    for identifier, segments in sorted(by_object.items()):
        run = 0
        previous = -2
        for segment in sorted(segments):
            run = run + 1 if segment == previous + 1 else 1
            if run >= GRASP_SEGMENTS:
                return identifier, segment
            previous = segment
    return None


@dataclass(frozen=True)
class Grip:
    """A food object's fixed offset from the end of its gripping segment."""

    object_id: int
    segment_index: int
    offset: Point  # Coordinates along and across the segment, measured in pixels.

    @classmethod
    def attach(cls, obj: WorldObject, segment: int, points: list[Point]) -> Grip:
        """Record the existing relative position without snapping the object.

        :param obj: Food object being captured.
        :param segment: Zero-based gripping segment, validated by local contacts.
        :param points: Current arm joints, including the tip.
        :return: Attachment preserving the object's current centre exactly.
        """
        start, end = points[segment], points[segment + 1]
        heading = atan2(end[1] - start[1], end[0] - start[0])
        dx, dy = obj.centre[0] - end[0], obj.centre[1] - end[1]
        return cls(
            obj.identifier,
            segment,
            (dx * cos(heading) + dy * sin(heading), -dx * sin(heading) + dy * cos(heading)),
        )

    def centre(self, points: list[Point]) -> Point:
        """Place the held object in this pose using its stored local offset.

        :param points: Connected joint positions with the same gripping segment.
        :return: Object centre in world pixels.
        """
        start, end = points[self.segment_index], points[self.segment_index + 1]
        heading = atan2(end[1] - start[1], end[0] - start[0])
        along, across = self.offset
        return (
            end[0] + along * cos(heading) - across * sin(heading),
            end[1] + along * sin(heading) + across * cos(heading),
        )

    def safe_motion(
        self,
        base: Point,
        before: list[float],
        after: list[float],
        length: float,
        radius: float,
        objects: tuple[WorldObject, ...],
    ) -> bool:
        """Check the carried circle's swept movement against every obstacle.

        :param base: Fixed arm attachment in pixels.
        :param before: Current relative joint angles.
        :param after: Proposed relative joint angles.
        :param length: Fixed segment length in pixels.
        :param radius: Carried object's circular radius in pixels.
        :param objects: Environment snapshot; only obstacle circles block a carry.
        :return: Whether the payload stays clear or does not worsen an initial overlap.
        """
        changes = [new - old for old, new in zip(before, after)]
        changes[0] = wrap_angle(changes[0])
        heading_change = travel = 0.0
        for change in changes[: self.segment_index + 1]:
            heading_change += change
            travel += length * abs(heading_change)
        # The attachment offset itself sweeps an arc as its segment turns.
        travel += hypot(*self.offset) * abs(heading_change)
        samples = max(1, ceil(travel / 2))
        old_centre = self.centre(pose_points(base, before, length))
        for sample in range(samples):
            fraction = (sample + 0.5) / samples
            angles = [old + fraction * change for old, change in zip(before, changes)]
            centre = self.centre(pose_points(base, angles, length))
            for obj in objects:
                if obj.kind is not ObjectKind.OBSTACLE:
                    continue
                old_distance = hypot(old_centre[0] - obj.centre[0], old_centre[1] - obj.centre[1])
                minimum = min(radius + obj.radius, old_distance)

                # Padding covers the unsampled motion between each midpoint and
                # the edges of its time interval, just as the arm's guard does.
                if (
                    hypot(centre[0] - obj.centre[0], centre[1] - obj.centre[1])
                    - travel / (2 * samples)
                    < minimum - 1e-8
                ):
                    return False
        return True
