"""Significant-event presentation is separate from solver decisions and raw diagnostics."""

from octopus_controller.activity import ActivityHistory
from octopus_controller.scenario import load_scenario


def test_reef_groups_critical_withdrawal_and_releases():
    """The opening demo retains captures and combines cause, action and consequence."""
    central, environment = load_scenario("scenarios/reef-forage-extended.json").build()
    history = ActivityHistory(central)
    for _ in range(360):
        central.step(1 / 120, environment)
        history.observe(central)
    assert len(history.filtered(category="Food")) == 2
    critical = [e for e in history.events if "Critical hazard" in e.text]
    assert len(critical) == 1
    assert "2 meals released" in critical[0].text
    assert history.total_releases == 2
    assert critical[0] in history.filtered(arm=7, category="Safety")
    assert not any("Conflict cleared" in e.text for e in history.events)


def test_repeated_events_keep_count_latest_time_and_filters():
    """Repeat collapsing preserves metadata and history stays bounded."""
    central, _ = load_scenario("scenarios/reef-forage-extended.json").build()
    history = ActivityHistory(central)
    history.record(1, "Blocked", (2,), "Blocked")
    history.record(2, "Food", (1,), "Captured")
    history.record(3, "Blocked", (2,), "Blocked")
    assert len(history.events) == 2
    assert history.events[-1].count == 2
    assert history.events[-1].seconds == 3
    assert not history.filtered(arm=1, category="Blocked")
    assert len(history.filtered(arm=2, category="Blocked")) == 1
    for i in range(300):
        history.record(i, "Food", (0,), str(i))
    assert len(history.events) == 256
