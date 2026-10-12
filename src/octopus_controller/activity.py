"""Presentation-only event history, sampled after every complete simulation tick."""

from collections import Counter, deque
from dataclasses import dataclass

from octopus_controller.organism import CentralController


@dataclass
class ActivityEvent:
    """A related group of changes with filter metadata and a repetition count."""

    seconds: float
    category: str
    arms: tuple[int, ...]
    text: str
    count: int = 1


class ActivityHistory:
    """Keep significant events independently of the model's noisy diagnostic logs."""

    def __init__(self, central: CentralController) -> None:
        """Establish a baseline without inventing historical events for current state.

        :param central: Current controller, read without changing its state.
        """
        self.events = deque(maxlen=256)
        self.capture_count = central.capture_count
        self.release_count = central.conflicts.release_count
        self.total_releases = 0
        self.states = [arm.state.value for arm in central.arms]
        self.grips = [arm.grip is not None for arm in central.arms]
        self.whole_withdrawal = central.conflicts.whole_withdrawal
        self.reports = central.priority.report_count
        self.escalations = central.priority.escalation_count
        self.decisions = [arm.decision for arm in central.arms]

    def record(self, seconds, category, arms, text):
        """Collapse repeated matching events and move them to their latest time.

        :param seconds: Simulation timestamp of this occurrence.
        :param category: User-facing filter category.
        :param arms: Zero-based arms involved in the event.
        :param text: Description of related changes in this tick.
        :return: None; retains at most 256 event groups.
        """
        arms = tuple(arms)
        count = 1
        for event in list(self.events):
            if (event.category, event.arms, event.text) == (category, arms, text):
                count = event.count + 1
                self.events.remove(event)
                break
        self.events.append(ActivityEvent(seconds, category, arms, text, count))

    def observe(self, central: CentralController) -> None:
        """Collect captures, safety sequences and changed conflicts after one tick.

        :param central: Controller just advanced or edited by the application.
        :return: None; changes only presentation history and its baseline.
        """
        states = [arm.state.value for arm in central.arms]
        grips = [arm.grip is not None for arm in central.arms]
        seconds = central.simulation_seconds
        released = [i for i, held in enumerate(self.grips) if held and not grips[i]]
        self.total_releases += len(released)
        captures = central.capture_count - self.capture_count
        if captures:
            arms = sorted({r.arm_index for r in list(central.capture_reports)[-captures:]})
            self.record(seconds, "Food", arms, f"{captures} food captured")
        critical = central.conflicts.whole_withdrawal and not self.whole_withdrawal
        if critical:
            text = "Critical hazard -> all arms instructed to withdraw"
            if released:
                text += f" -> {len(released)} meals released"
            self.record(seconds, "Safety", range(len(states)), text)
        elif central.priority.escalation_count > self.escalations:
            arms = [i for i, a in enumerate(central.arms) if a.withdrawal_requested]
            text = "Hazard escalation -> withdrawal requested"
            if released:
                text += f" -> {len(released)} meals released"
            self.record(seconds, "Safety", arms, text)
        elif released:
            self.record(seconds, "Food", released, f"{len(released)} meals released")
        if (
            central.priority.report_count > self.reports
            and not critical
            and central.priority.escalation_count == self.escalations
        ):
            self.record(
                seconds,
                "Safety",
                tuple(
                    sorted(
                        {
                            r.arm_index
                            for r in central.priority.reports
                            if r.simulation_seconds == seconds
                        }
                    )
                ),
                "Local hazard reported to central control",
            )
        self.reports = central.priority.report_count
        if self.whole_withdrawal and not central.conflicts.whole_withdrawal:
            self.record(
                seconds,
                "Safety",
                range(len(states)),
                "Critical alert cleared -> retained tasks resume",
            )
        for i, arm in enumerate(central.arms):
            if "Blocked" in states[i] or "blocked" in states[i]:
                if states[i] != self.states[i]:
                    self.record(seconds, "Blocked", (i,), states[i])
            decision = arm.decision
            # Routine discovery/clear changes stay out of the demonstration summary.
            if decision != self.decisions[i] and decision.action == "Yield":
                self.record(seconds, "Conflict", (i,), decision.reason or "Yielding to another arm")
        self.capture_count = central.capture_count
        self.release_count = central.conflicts.release_count
        self.states, self.grips = states, grips
        self.whole_withdrawal = central.conflicts.whole_withdrawal
        self.escalations = central.priority.escalation_count
        self.decisions = [arm.decision for arm in central.arms]

    def filtered(self, arm: int | None = None, category: str = "All") -> list[ActivityEvent]:
        """Return chronological groups matching both filters.

        :param arm: Zero-based arm, or None for all arms.
        :param category: Event category, or All.
        :return: Matching events in latest-occurrence order.
        """
        return [
            e
            for e in self.events
            if (arm is None or arm in e.arms) and (category == "All" or e.category == category)
        ]


def state_summary(central: CentralController) -> str:
    """Count current states separately from historical events.

    :param central: Read-only controller whose arms supply the current states.
    :return: Compact human-readable state counts.
    """
    counts = Counter(arm.state.value.lower() for arm in central.arms)
    return " · ".join(f"{count} {state}" for state, count in counts.items())
