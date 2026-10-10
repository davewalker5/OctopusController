"""Explicit populated scenes for movement tests, independent of UI startup defaults."""

import pytest


@pytest.fixture
def make_reaching_application():
    """Build a running reach/grasp/avoidance scene when a test needs movement."""
    from octopus_controller.app import BODY_CENTRE, Application
    from octopus_controller.organism import CentralController
    from octopus_controller.sensing import ObjectKind

    def build():
        application = Application()
        application.central = CentralController(BODY_CENTRE)
        application.environment.add(application.central.arm(0).controller.target, ObjectKind.FOOD)
        base = application.central.arm(2).controller.arm.base
        application.central.assign_reach(2, (base[0] + 100, base[1]))
        application.environment.add((base[0] + 60, base[1] + 67), ObjectKind.OBSTACLE)
        application.central.refresh_sensing(application.environment.objects)
        application.paused = False
        return application

    return build
