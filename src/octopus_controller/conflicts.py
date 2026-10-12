"""Transparent arbitration over reported intentions, never over an environment map.

Priorities are experimental: withdrawal 100, local avoidance 80, grip 60,
reach 50, incidental discovery 20 and idle 0. Geometry remains arm-local.
Target coordinates use world pixels; event timestamps use simulation seconds.
This module selects broad actions. ControlledArm applies them, including releasing
food on withdrawal; its local controller still determines all joint movements.
"""

import logging
from collections import deque
from dataclasses import dataclass, replace
from math import dist, isfinite

from octopus_controller.geometry import Point

# These values label a fixed hierarchy, not weights to add together. Discovery
# is a competing opportunity; it never replaces an explicit food task here.
WITHDRAWAL = 100
AVOIDANCE = 80
GRIP = 60
REACH = 50
DISCOVERY = 20
IDLE = 0


@dataclass(frozen=True)
class ConflictPolicy:
    """Configure broadcast severity and coordinate-based resource matching.

    The caller checks outstanding P100 reports against critical_severity and
    passes the resulting broadcast flag to resolve. target_tolerance is an
    inclusive distance in world pixels, not a physical arm collision radius.
    Frozen settings can safely be shared without sharing reservation state.
    """

    critical_severity: float = 0.95
    target_tolerance: float = 2.0

    def __post_init__(self) -> None:
        """Validate finite experimental thresholds before building a scene.

        :raises ValueError: For invalid severity or a negative target tolerance.
        """
        # Reject bool despite its int ancestry: a toggle is not a severity or
        # distance. Finite values keep comparisons meaningful in both runtimes.
        if any(
            type(v) not in (int, float) or not isfinite(v)
            for v in (self.critical_severity, self.target_tolerance)
        ):
            raise ValueError("Conflict settings must be finite numbers")
        if not 0 < self.critical_severity <= 1 or self.target_tolerance < 0:
            raise ValueError("Require 0 < critical_severity <= 1 and target_tolerance >= 0")


@dataclass(frozen=True)
class ArmIntent:
    """One arm's assignment and locally observed resource claim, not raw world data.

    target is the tip or payload destination; active distinguishes a moving task
    from an idle hold. held_object is actual ownership, goal_object is an explicit
    food assignment, and candidate is food qualifying for a local grasp attempt.
    Object IDs are positive integers; None means that claim is absent. discovery
    identifies incidental food filtered out by the arm's existing task. avoiding
    reports local aversion, independently of a central withdrawal request.
    """

    active: bool
    target: Point
    held_object: int | None = None
    goal_object: int | None = None
    candidate: int | None = None
    avoiding: bool = False
    discovery: int | None = None


@dataclass(frozen=True)
class ConflictDecision:
    """Inspectible selected behaviour and the reason a competing intention lost.

    An empty conflict/reason denotes an ordinary unopposed action. scope labels
    whether the outcome depends on local intentions or central coordination;
    even Central decisions leave the movement geometry to the arm controller.
    """

    action: str
    priority: int
    conflict: str = ""
    reason: str = ""
    winner: int | None = None  # Zero-based winning arm for resource contention.
    scope: str = "Local"


@dataclass(frozen=True)
class ConflictEvent:
    """A changed conflict outcome, not a repeated per-frame notification."""

    simulation_seconds: float
    arm_index: int
    decision: ConflictDecision


class ConflictResolver:
    """Resolve withdrawals and resource contention with stable, explicit tie-breaks."""

    def __init__(self, policy: ConflictPolicy = ConflictPolicy()) -> None:
        """Start empty decisions, reservations and bounded event history.

        :param policy: Validated configuration; all behavioural priorities are fixed.
        """
        self.policy = policy
        self.decisions: tuple[ConflictDecision, ...] = ()
        self.events: deque[ConflictEvent] = deque(maxlen=64)
        self.conflict_count = 0
        # Applying an action can release food; the caller increments this only
        # after an actual release, rather than counting repeated Withdraw decisions.
        self.release_count = 0
        self.whole_withdrawal = False
        # Keep the previous resource as well as its winning arm. An arm cannot
        # carry its precedence across an unrelated reassignment.
        self._incumbents: dict[int, ArmIntent] = {}

    def resolve(
        self, intents: tuple[ArmIntent, ...], withdrawing: set[int], whole: bool, seconds: float
    ) -> tuple[ConflictDecision, ...]:
        """Choose behaviours from local intentions and already-reported threats.

        Claims are settled before movement: existing ownership wins, then an
        eligible previous winner, then the lowest arm index. Inputs are trusted
        arm-layer summaries and are not mutated. The caller applies each decision:
        ordinary reaches survive withdrawal, but a released carry is abandoned.
        No pose, route or grip mutation is calculated or performed here.

        :param intents: Arm-local summaries in stable arm-index order.
        :param withdrawing: Arms with outstanding high-priority hazard reports.
        :param whole: Whether a critical report requests every arm to withdraw.
        :param seconds: Validated simulation time, used only for event logging.
        :return: One immutable decision per arm; updates reservations and history.
        """
        # Resolve each arm locally before comparing resource claims. A broadcast
        # applies even to idle arms, which must not be excluded from safety commands.
        decisions = []
        for index, intent in enumerate(intents):
            if whole or index in withdrawing:
                conflict = (
                    "Hold versus withdraw"
                    if intent.held_object is not None
                    else ("Approach versus avoidance" if intent.active else "")
                )
                decisions.append(
                    ConflictDecision(
                        "Withdraw",
                        WITHDRAWAL,
                        conflict,
                        "Withdrawal 100 overrides grip 60; release food"
                        if intent.held_object is not None
                        else "Withdrawal 100 suspends reach 50"
                        if intent.active
                        else "Withdrawal 100 overrides idle 0",
                        scope="Central",
                    )
                )
            # Avoidance redirects active motion without necessarily releasing a
            # payload; an inactive owner can simply maintain its existing grip.
            elif intent.active and intent.avoiding:
                decisions.append(
                    ConflictDecision(
                        "Avoid hazard",
                        AVOIDANCE,
                        "Approach versus avoidance",
                        "Local avoidance 80 overrides reach 50",
                    )
                )
            elif intent.held_object is not None:
                decisions.append(ConflictDecision("Maintain grip", GRIP))
            elif intent.active:
                decisions.append(
                    ConflictDecision(
                        "Reach",
                        REACH,
                        "Discovery versus task" if intent.discovery is not None else "",
                        "Assigned food task 50 overrides incidental discovery 20"
                        if intent.discovery is not None
                        else "",
                    )
                )
            else:
                decisions.append(ConflictDecision("Idle", IDLE))

        # Each claimant joins the first matching resource group. Coordinate-only
        # requests use the first group's target as an anchor (no transitive chain
        # of increasingly distant points); named food and local contacts use IDs.
        groups: list[list[int]] = []
        for index, intent in enumerate(intents):
            # Retreating arms relinquish claims. Idle owners still reserve food,
            # whereas an idle arm with only an old destination reserves nothing.
            if decisions[index].action == "Withdraw" or not (
                intent.active or intent.held_object is not None
            ):
                continue
            for group in groups:
                if self._same_resource(intent, intents[group[0]]):
                    group.append(index)
                    break
            else:
                groups.append([index])
        incumbents = {}
        for group in groups:
            # A carrying owner may steer away locally without giving up its grip.
            # Its object remains unavailable even while safety redirects motion.
            eligible = [
                i
                for i in group
                if decisions[i].action != "Avoid hazard" or intents[i].held_object is not None
            ]
            if not eligible:
                continue
            # Python compares tuple fields in order, with False before True.
            # Ownership therefore wins before incumbency is considered; the final
            # index breaks exact ties reproducibly, regardless of UI selection.
            winner = min(
                eligible,
                key=lambda i: (
                    intents[i].held_object is None,
                    not (
                        i in self._incumbents
                        and self._same_resource(intents[i], self._incumbents[i])
                    ),
                    i,
                ),
            )
            # Remember even an uncontested claim, so a later newcomer cannot
            # interrupt it merely by having a lower arm number.
            incumbents[winner] = intents[winner]
            if len(group) < 2:
                continue
            # Annotate the winner without replacing its Reach/Grip/Avoid action.
            decisions[winner] = replace(
                decisions[winner],
                conflict="Competing arms",
                reason="Existing owner, then eligible incumbent, then lowest arm number wins",
                winner=winner,
                scope="Central",
            )
            # Yield stops ordinary task motion, not safety steering. The arm
            # retains its assignment and can compete again on the next tick.
            for index in group:
                if index == winner or decisions[index].action == "Avoid hazard":
                    continue
                decisions[index] = ConflictDecision(
                    "Yield",
                    IDLE,
                    "Competing arms",
                    f"Arm {winner + 1} keeps the resource claim; this arm waits",
                    winner,
                    "Central",
                )
        # Rebuild rather than merge: abandoned, withdrawn or wholly avoiding
        # claims must lose their previous reservation immediately.
        self._incumbents = incumbents
        self.whole_withdrawal = whole
        for index, decision in enumerate(decisions):
            previous = self.decisions[index] if index < len(self.decisions) else None
            # Frozen records compare by value. Log transitions into, within and
            # out of a conflict, but neither repeated frames nor routine actions.
            # Clearances remain visible without increasing the conflict count;
            # cumulative counts survive eviction from the 64-event history.
            if decision != previous and (decision.conflict or (previous and previous.conflict)):
                self.events.append(ConflictEvent(seconds, index, decision))
                self.conflict_count += bool(decision.conflict)
                logging.getLogger(__name__).info(
                    "%.3fs Arm %d: %s (%s)", seconds, index + 1, decision.action, decision.reason
                )
        self.decisions = tuple(decisions)
        return self.decisions

    def _same_resource(self, first: ArmIntent, second: ArmIntent) -> bool:
        """Compare reported claims, without looking up food in the environment.

        :param first: A prospective claimant.
        :param second: The stable anchor for an existing group.
        :return: True for the same reported food or a nearby coordinate assignment.
        """
        # Ownership is stronger evidence than an assignment, which in turn
        # outranks a tentative contact. Positive IDs make this fallback safe.
        first_id = first.held_object or first.goal_object or first.candidate
        second_id = second.held_object or second.goal_object or second.candidate
        # Two explicit identities are authoritative even at the same position.
        # If either is absent, compare destinations; this permits a coordinate
        # request to contend with a named resource without consulting a world map.
        if first_id is not None and second_id is not None:
            return first_id == second_id
        return dist(first.target, second.target) <= self.policy.target_tolerance
