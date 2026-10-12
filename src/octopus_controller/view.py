"""Pygame drawing only; the renderer never changes the simulated arm."""

from dataclasses import dataclass
from math import degrees, hypot

import pygame

from octopus_controller.activity import state_summary
from octopus_controller.geometry import Point
from octopus_controller.organism import BODY_RADIUS, CentralController, ControlledArm
from octopus_controller.sensing import ObjectKind, WorldObject

HEADER_ICON_SIZE = (64, 64)
SIDEBAR_TOP = 20
SCENARIO_NAME_BUTTON = pygame.Rect(808, SIDEBAR_TOP + 30, 278, 23)
LOAD_SCENARIO_BUTTON = pygame.Rect(808, SIDEBAR_TOP + 56, 86, 26)
SAVE_SCENARIO_BUTTON = pygame.Rect(904, SIDEBAR_TOP + 56, 86, 26)
CLEAR_SCENARIO_BUTTON = pygame.Rect(1000, SIDEBAR_TOP + 56, 86, 26)
CONTROL_STACK_BOTTOM = SIDEBAR_TOP + 740
WINDOW_SIZE = (1120, CONTROL_STACK_BOTTOM + 20)
# Reserve three text rows below the scene, keeping the window/sidebar unchanged.
MESSAGE_PANEL = pygame.Rect(20, CONTROL_STACK_BOTTOM - 80, 760, 80)
HISTORY_BUTTON = pygame.Rect(MESSAGE_PANEL.right - 90, MESSAGE_PANEL.top + 8, 78, 24)
HISTORY_PANEL = pygame.Rect(45, 120, 1030, 570)
HISTORY_CLOSE = pygame.Rect(970, 135, 90, 28)
HISTORY_ARM = pygame.Rect(65, 178, 200, 30)
HISTORY_CATEGORY = pygame.Rect(280, 178, 200, 30)
HISTORY_LATEST = pygame.Rect(870, 178, 190, 30)
HISTORY_ROWS = 19
WORLD = pygame.Rect(20, 90, 760, MESSAGE_PANEL.top - 10 - 90)
BACKGROUND = (16, 26, 39)
TEXT = (220, 231, 239)
MUTED = (143, 164, 181)
ACCENT = (79, 212, 184)
ARM_COLOURS = (
    ACCENT,
    (109, 177, 244),
    (180, 150, 245),
    (239, 146, 197),
    (248, 167, 116),
    (226, 208, 115),
    (149, 206, 119),
    (114, 208, 223),
)


@dataclass(frozen=True)
class SceneCamera:
    """Uniform presentation transform; simulation positions remain in world units."""

    scale: float = 1.0
    offset: Point = (0.0, 0.0)

    @classmethod
    def fit(cls, central: CentralController) -> SceneCamera:
        """Centre the body and fit every arm's full possible reach with a margin.

        Using link lengths rather than the current bent pose prevents clipping
        when an arm later straightens. The camera stays fixed during playback.

        :param central: Loaded body and arm geometry, read without modification.
        :return: Uniform scale and translation into the scene viewport.
        """
        half_width = half_height = BODY_RADIUS
        for arm in central.arms:
            geometry = arm.controller.arm
            length = geometry.parameters.segment_count * geometry.parameters.segment_length
            half_width = max(half_width, abs(geometry.base[0] - central.centre[0]) + length)
            half_height = max(half_height, abs(geometry.base[1] - central.centre[1]) + length)
        margin = 24  # Screen pixels reserved for markers, line widths and labels.
        scale = min(
            (WORLD.width - 2 * margin) / (2 * half_width),
            (WORLD.height - 2 * margin) / (2 * half_height),
        )
        return cls(
            scale,
            (WORLD.centerx - central.centre[0] * scale, WORLD.centery - central.centre[1] * scale),
        )

    def project(self, point: Point) -> Point:
        """Map world coordinates to the displayed scene.

        :param point: Model position in world units.
        :return: Position in window pixels.
        """
        return (point[0] * self.scale + self.offset[0], point[1] * self.scale + self.offset[1])

    def unproject(self, point: Point) -> Point:
        """Invert the view for target assignment, object picking and dragging.

        :param point: Position in window pixels, already clamped if dragging.
        :return: Position in the original scenario coordinate system.
        """
        return ((point[0] - self.offset[0]) / self.scale, (point[1] - self.offset[1]) / self.scale)


def draw_scene(
    screen: pygame.Surface,
    central: CentralController,
    paused: bool,
    font: pygame.font.Font,
    heading_font: pygame.font.Font,
    header_icon: pygame.Surface,
    selected_index: int = 0,
    objects: tuple[WorldObject, ...] = (),
    selected_object: int | None = None,
    placement: ObjectKind | None = None,
    shortcuts_font: pygame.font.Font | None = None,
    scenario_name: str = "Empty scenario",
    load_error: str | None = None,
    message_scroll: int | None = None,
    camera: SceneCamera = SceneCamera(),
    playback_speed: int = 1,
    activity=None,
) -> None:
    """Draw eight arms, assignments, selected-arm settings and keyboard reference.

    :param activity: Optional significant-event history for the summary.
    :param screen: Destination window surface.
    :param central: Body and independent arm controllers to display.
    :param playback_speed: Wall-time multiplier; model tick duration remains unchanged.
    :param paused: Whether automatic updates are suspended.
    :param font: Font for labels and measurements.
    :param heading_font: Larger font for the window heading.
    :param header_icon: App icon already scaled to fit the heading.
    :param selected_index: Zero-based arm to highlight and inspect.
    :param objects: Circular sensing objects to draw beneath the arms.
    :param selected_object: Object ID highlighted for moving or deletion.
    :param placement: One-click placement tool, or None for normal target assignment.
    :param shortcuts_font: Smaller shortcut font; omitted callers use an 18-pixel default.
    :param scenario_name: Name of the currently loaded setup.
    :param load_error: File loading error, or None when no error is displayed.
    :param message_scroll: First visible message row, or None to follow the latest rows.
    :param camera: Read-only world-to-window transform chosen when loading the scene.
    """
    # Clear the previous frame so moving segments and labels leave no trails.
    # Pygame draws in call order: later shapes appear over earlier ones.
    screen.fill(BACKGROUND)

    # Centre the icon across both text rows so it belongs to the complete heading.
    # Keep the subtitle on the same left edge as the title for a clear hierarchy.
    screen.blit(header_icon, header_icon.get_rect(center=(52, 44)))
    title = heading_font.render("Distributed Octopus Controller", True, TEXT)
    screen.blit(title, (100, 16))
    subtitle = font.render("Simulation of an octopus-like distributed control system", True, MUTED)
    screen.blit(subtitle, (100, 52))
    pygame.draw.rect(screen, (22, 37, 51), WORLD, border_radius=12)

    # Clip geometry to the world so long experimental arms cannot cover controls.
    # Project model coordinates through the load-time camera; UI cards and
    # message rows stay in window pixels and do not move with the scene.
    screen.set_clip(WORLD)

    # Draw a faint grid behind the arm to make movement and distance easier to
    # judge. Its 40-pixel spacing is a visual reference, not a simulation rule.
    for x in range(WORLD.left, WORLD.right, 40):
        pygame.draw.line(screen, (29, 46, 61), (x, WORLD.top), (x, WORLD.bottom))
    for y in range(WORLD.top, WORLD.bottom, 40):
        pygame.draw.line(screen, (29, 46, 61), (WORLD.left, y), (WORLD.right, y))

    # Draw objects beneath the arms so the contact sites stay visible. Different
    # shapes inside the circles distinguish types without depending on colour.
    owners = {arm.grip.object_id: index for index, arm in enumerate(central.arms) if arm.grip}
    for obj in objects:
        centre = camera.project(obj.centre)
        radius = obj.radius * camera.scale
        colour = (131, 209, 120) if obj.kind is ObjectKind.FOOD else (229, 155, 105)
        if obj.kind is ObjectKind.HAZARD:
            colour = (240, 100, 160)
            pygame.draw.circle(
                screen, colour, centre, radius + obj.detection_range * camera.scale, 1
            )
        pygame.draw.circle(screen, colour, centre, radius, 2)
        label = font.render(
            "H" if obj.kind is ObjectKind.HAZARD else "F" if obj.kind is ObjectKind.FOOD else "O",
            True,
            colour,
        )

        # Keep the type label outside the circle: a target at its centre should
        # not cover the letter that distinguishes food from an obstacle.
        screen.blit(label, (centre[0] - radius, centre[1] - radius - 17))
        if obj.identifier in owners:
            # The owner's colour makes a latched object distinguishable from
            # ordinary contact, even when its segment markers overlap the circle.
            pygame.draw.circle(screen, ARM_COLOURS[owners[obj.identifier]], centre, radius + 4, 2)

        if obj.identifier == selected_object:
            pygame.draw.circle(screen, TEXT, centre, radius + 5, 1)

    # Draw the selected arm last so it is easiest to follow where arms overlap.
    # Draw order affects only visibility; it never gives an arm movement priority.
    order = [index for index in range(len(central.arms)) if index != selected_index]
    order.append(selected_index)
    for index in order:
        draw_arm(screen, central.arm(index), index, index == selected_index, font, camera)

    # The body marks fixed attachments. Only explicit circular obstacles block
    # movement; body and arm-to-arm collisions are outside this model.
    pygame.draw.circle(
        screen, (39, 61, 77), camera.project(central.centre), BODY_RADIUS * camera.scale
    )
    pygame.draw.circle(screen, MUTED, camera.project(central.centre), BODY_RADIUS * camera.scale, 2)

    # Number the bases after drawing the body so their labels remain readable.
    # Numbers, colours and the roster all refer to the same persistent arm IDs.
    for index, arm in enumerate(central.arms):
        base = camera.project(arm.controller.arm.base)
        label = font.render(str(index + 1), True, ARM_COLOURS[index])
        screen.blit(label, label.get_rect(center=base))

    # Messages occupy their own viewport, never the clipped simulation scene.
    screen.set_clip(None)
    draw_messages(
        screen,
        central,
        selected_index,
        shortcuts_font or pygame.font.Font(None, 18),
        message_scroll,
        playback_speed,
        paused,
        activity,
    )
    draw_sidebar(
        screen,
        central,
        selected_index,
        paused,
        font,
        shortcuts_font if shortcuts_font is not None else pygame.font.Font(None, 18),
        scenario_name,
        playback_speed,
    )
    if placement is not None:
        # Keep the active placement instruction close to the world where the
        # next click applies, separate from the permanent shortcut reference.
        hint = font.render(
            f"Click to place {placement.value.lower()}  /  press F, O or H to change tool",
            True,
            TEXT,
        )
        box = hint.get_rect(topleft=(36, WORLD.bottom - 33)).inflate(16, 12)
        pygame.draw.rect(screen, (39, 61, 77), box, border_radius=6)
        screen.blit(hint, (36, WORLD.bottom - 33))

    if load_error is not None:
        lines = wrap_text(load_error, font, WORLD.width - 64)
        box = pygame.Rect(WORLD.left + 16, WORLD.top + 16, WORLD.width - 32, 64 + 22 * len(lines))
        pygame.draw.rect(screen, (62, 36, 39), box, border_radius=8)
        screen.blit(
            font.render("Could not load scenario — Esc to dismiss", True, TEXT),
            (box.left + 16, box.top + 12),
        )
        for index, line in enumerate(lines):
            screen.blit(font.render(line, True, TEXT), (box.left + 16, box.top + 42 + index * 22))


def ellipsize(text, font, width):
    """Fit a single summary row without covering adjacent controls.

    :param text: Original row.
    :param font: Measurement font.
    :param width: Maximum pixel width.
    :return: Original text or a shortened label ending in an ellipsis.
    """
    if font.size(text)[0] <= width:
        return text
    while text and font.size(text + "…")[0] > width:
        text = text[:-1]
    return text + "…"


def draw_messages(
    screen,
    central,
    selected_index,
    font,
    first_row=None,
    playback_speed=1,
    paused=True,
    activity=None,
):
    """Draw a stable three-row summary with a separate history entry point.

    :param screen: Destination surface.
    :param central: Read-only controller.
    :param selected_index: Selected arm, unused by the overall summary.
    :param font: Compact summary font.
    :param first_row: Legacy argument; the summary never scrolls.
    :param playback_speed: Wall-time multiplier.
    :param paused: Current playback state.
    :param activity: Presentation history supplying the latest significant event.
    :return: None.
    """
    latest = activity.events[-1] if activity and activity.events else None
    rows = [
        f"{central.goal} · {'Paused' if paused else 'Playing'} {playback_speed}×"
        f" · {central.simulation_seconds:.2f}s",
        state_summary(central),
        "Latest: " + (event_label(latest) if latest else "No significant events yet"),
    ]
    pygame.draw.rect(screen, (22, 37, 51), MESSAGE_PANEL, border_radius=8)
    for index, text in enumerate(rows):
        width = MESSAGE_PANEL.width - (110 if index == 0 else 24)
        colour = (
            ACCENT
            if index == 2 and latest and central.simulation_seconds - latest.seconds < 2
            else TEXT
        )
        screen.blit(
            font.render(ellipsize(text, font, width), True, colour),
            (MESSAGE_PANEL.left + 12, MESSAGE_PANEL.top + 10 + index * 20),
        )
    pygame.draw.rect(screen, (39, 61, 77), HISTORY_BUTTON, border_radius=4)
    label = font.render("History", True, TEXT)
    screen.blit(label, label.get_rect(center=HISTORY_BUTTON.center))
    # Detailed food accounting remains in the expanded view to keep all three rows legible.


def event_label(event):
    """Format a retained event, identifying arms and the latest repeat time.

    :param event: ActivityEvent to present.
    :return: One logical line; the expanded view wraps it as needed.
    """
    arms = ", ".join(str(i + 1) for i in event.arms)
    scope = "All arms" if len(event.arms) == 8 else f"Arm {arms}" if arms else "Central"
    repeat = f" (×{event.count}, latest)" if event.count > 1 else ""
    return f"{event.seconds:.2f}s · {scope}: {event.text}{repeat}"


def history_lines(activity, arm, category, font):
    """Wrap filtered events before pagination so long messages remain readable.

    :param activity: Bounded presentation history.
    :param arm: Optional zero-based arm filter.
    :param category: Category filter.
    :param font: Font used to measure lines.
    :return: Wrapped display rows in chronological order.
    """
    return [
        line
        for event in activity.filtered(arm, category)
        for line in wrap_text(event_label(event), font, HISTORY_PANEL.width - 55)
    ]


def draw_history(screen, central, activity, font, arm, category, first_row):
    """Draw the paused inspection overlay, filters and explicit follow control.

    :param screen: Destination surface.
    :param central: Read-only state for food accounting.
    :param activity: Retained significant events.
    :param font: Compact history font.
    :param arm: Optional zero-based arm filter.
    :param category: Event filter.
    :param first_row: Explicit row, or None to follow latest.
    :return: None.
    """
    pygame.draw.rect(screen, (16, 26, 39), HISTORY_PANEL, border_radius=12)
    pygame.draw.rect(screen, ACCENT, HISTORY_PANEL, width=1, border_radius=12)
    held = sum(a.grip is not None for a in central.arms)
    title = (
        f"History — paused · {central.capture_count} captures"
        f" · {activity.total_releases} releases · {held} currently held"
    )
    screen.blit(font.render(title, True, TEXT), (65, 142))
    for rect, label in [
        (HISTORY_CLOSE, "Close / Esc"),
        (HISTORY_ARM, "All arms" if arm is None else f"Arm {arm + 1}"),
        (HISTORY_CATEGORY, f"Type: {category}"),
        (HISTORY_LATEST, "Jump to latest"),
    ]:
        pygame.draw.rect(screen, (39, 61, 77), rect, border_radius=4)
        rendered = font.render(label, True, TEXT)
        screen.blit(rendered, rendered.get_rect(center=rect.center))
    lines = history_lines(activity, arm, category, font)
    maximum = max(0, len(lines) - HISTORY_ROWS)
    start = maximum if first_row is None else max(0, min(maximum, first_row))
    for row, line in enumerate(lines[start : start + HISTORY_ROWS] or ["No matching events."]):
        screen.blit(font.render(line, True, TEXT), (65, 228 + row * 21))
    footer = (
        f"Rows {start + 1 if lines else 0}–{min(start + HISTORY_ROWS, len(lines))}"
        f" of {len(lines)} · Scroll / Up Down / Page Up Down"
        " · Click filters to cycle · Latest 256 groups"
    )
    screen.blit(font.render(footer, True, MUTED), (65, 653))
    track = pygame.Rect(1050, 228, 6, HISTORY_ROWS * 21)
    pygame.draw.rect(screen, (39, 61, 77), track, border_radius=3)
    height = max(12, round(track.height * min(1, HISTORY_ROWS / max(1, len(lines)))))
    offset = round((track.height - height) * start / maximum) if maximum else 0
    pygame.draw.rect(screen, ACCENT, (track.x, track.y + offset, 6, height), border_radius=3)


def wrap_text(text: str, font: pygame.font.Font, width: int) -> list[str]:
    """Wrap even unbroken filenames to fit a fixed-width panel."""
    lines = []
    line = ""
    for character in " ".join(text.split()):
        if line and font.size(line + character)[0] > width:
            lines.append(line)
            line = ""
        line += character
    if line:
        lines.append(line)
    return lines or [""]


def draw_card(
    screen: pygame.Surface, rect: pygame.Rect, title: str, font: pygame.font.Font
) -> None:
    """Draw a quiet panel background and a consistent section heading.

    :param screen: Destination window surface.
    :param rect: Panel bounds in screen pixels.
    :param title: Short section name.
    :param font: Shared label font.
    """
    pygame.draw.rect(screen, (22, 37, 51), rect, border_radius=10)
    screen.blit(font.render(title, True, MUTED), (rect.left + 14, rect.top + 12))


def draw_keycap(
    screen: pygame.Surface,
    label: str,
    position: tuple[int, int],
    font: pygame.font.Font,
    enabled: bool = True,
) -> None:
    """Render a shortcut as a key-shaped label, not an interactive button.

    :param screen: Destination surface.
    :param label: Keyboard key or mouse gesture shown in the reference.
    :param position: Top-left corner in pixels.
    :param font: Shared label font.
    :param enabled: Whether the associated action currently applies.
    """
    text = font.render(label, True, TEXT if enabled else MUTED)
    rect = pygame.Rect(position, (text.get_width() + 12, 22))
    pygame.draw.rect(screen, (39, 56, 72), rect, border_radius=4)
    pygame.draw.rect(screen, (60, 79, 96), rect, width=1, border_radius=4)
    screen.blit(text, text.get_rect(center=rect.center))


def draw_sidebar(
    screen: pygame.Surface,
    central: CentralController,
    selected_index: int,
    paused: bool,
    font: pygame.font.Font,
    shortcuts_font: pygame.font.Font,
    scenario_name: str = "Empty scenario",
    playback_speed: int = 1,
) -> None:
    """Group live arm status, selected settings and shortcuts into distinct cards.

    :param screen: Destination surface with world clipping removed.
    :param central: Live model to inspect without changing it.
    :param selected_index: Zero-based selected arm.
    :param playback_speed: Wall-time multiplier; model tick duration remains unchanged.
    :param paused: Whether automatic simulation updates are suspended.
    :param font: Shared label font.
    :param shortcuts_font: Smaller font for the keypress/action table.
    :param scenario_name: Current setup name; long labels are ellipsized.
    """
    # Anchor the cards to the window margin rather than the world grid.
    # Keeping their positions relative to one top edge preserves the stack spacing.
    draw_card(screen, pygame.Rect(794, SIDEBAR_TOP, 306, 88), "SCENARIO", font)
    name = " ".join(scenario_name.split())
    if font.size(name)[0] > 278:
        while name and font.size(name + "…")[0] > 278:
            name = name[:-1]
        name += "…"
    pygame.draw.rect(screen, (29, 46, 61), SCENARIO_NAME_BUTTON, border_radius=4)
    screen.blit(font.render(name, True, ACCENT), (808, SIDEBAR_TOP + 33))
    for rect, text in [
        (LOAD_SCENARIO_BUTTON, "Load"),
        (SAVE_SCENARIO_BUTTON, "Save"),
        (CLEAR_SCENARIO_BUTTON, "Clear"),
    ]:
        pygame.draw.rect(screen, (39, 61, 77), rect, border_radius=5)
        pygame.draw.rect(screen, ACCENT, rect, width=1, border_radius=5)
        label = shortcuts_font.render(text, True, TEXT)
        screen.blit(label, label.get_rect(center=rect.center))

    top = SIDEBAR_TOP + 100
    draw_card(screen, pygame.Rect(794, top, 306, 144), "ARMS", font)
    # Keep run state visible without competing with the main window heading.
    run_label = font.render(
        f"{'PAUSED' if paused else 'RUN'} {playback_speed}x", True, MUTED if paused else ACCENT
    )
    screen.blit(run_label, run_label.get_rect(topright=(1086, top + 12)))
    for index, arm in enumerate(central.arms):
        # Two columns shorten the roster while retaining stable number order:
        # read across each row, just as the shortcut pairs below are read.
        row = pygame.Rect(802 + (index % 2) * 146, top + 36 + (index // 2) * 24, 144, 23)
        if index == selected_index:
            pygame.draw.rect(screen, (39, 61, 77), row, border_radius=5)
            pygame.draw.rect(screen, ARM_COLOURS[index], row, width=1, border_radius=5)
        pygame.draw.circle(screen, ARM_COLOURS[index], (row.left + 10, row.centery), 3)
        state_name = {
            "Holding under alert": "Holding (alert)",
            "Withdrawal blocked": "Retreat blocked",
        }.get(arm.state.value, arm.state.value)
        state_label = f"{index + 1}  {state_name}"
        state_font = font if font.size(state_label)[0] <= row.width - 24 else shortcuts_font
        label = state_font.render(state_label, True, ARM_COLOURS[index])
        screen.blit(label, (row.left + 20, row.top + 4))

    selected = central.arm(selected_index)
    controller = selected.controller
    parameters = controller.arm.parameters
    draw_card(
        screen,
        pygame.Rect(794, top + 156, 306, 202),
        f"ARM {selected_index + 1}  /  SETTINGS",
        font,
    )
    # Use the same compact text size for settings and the shortcut reference.
    distance = (
        f"Tip distance  {controller.distance:.1f} px" if selected.active else "No active target"
    )
    if controller.status == "Beyond reach" and selected.active and selected.grip is None:
        distance = f"Beyond reach  /  {controller.distance:.1f} px"
    if selected.grip is not None:
        distance = f"Holding food {selected.grip.object_id}"
        if selected.active and selected.carry_goal is not None:
            centre = selected.grip.centre(controller.arm.points)
            remaining = hypot(
                centre[0] - selected.carry_goal[0], centre[1] - selected.carry_goal[1]
            )
            distance = f"Carry distance  {remaining:.1f} px"
    screen.blit(shortcuts_font.render(distance, True, TEXT), (808, top + 189))

    # Align values and adjustment keys into fixed columns so changing a number
    # does not move the shortcut hints. All four settings affect only this arm.
    settings = [
        ("Segments", str(parameters.segment_count), "[  ]"),
        ("Length", f"{parameters.segment_length:.0f} px", "-  ="),
        ("Bend", f"{degrees(parameters.maximum_bend):.0f} deg", ",  ."),
        ("Speed", f"{parameters.turning_speed:.2f} rad/s", ";  '"),
    ]
    for index, (label, value, keys) in enumerate(settings):
        y = top + 210 + index * 24
        screen.blit(shortcuts_font.render(label, True, MUTED), (808, y + 3))
        screen.blit(shortcuts_font.render(value, True, TEXT), (913, y + 3))
        draw_keycap(screen, keys, (1048, y), shortcuts_font)

    sensor_count = len({contact.segment_index for contact in selected.contacts})
    kinds = ", ".join(sorted({contact.kind.value for contact in selected.contacts})) or "None"
    pygame.draw.line(screen, (39, 56, 72), (808, top + 312), (1086, top + 312))
    screen.blit(
        shortcuts_font.render(
            (
                f"Hazard {max(controller.hazard.severity, selected.signaller.severity):.2f}"
                f" / encounters {controller.hazard.encounters}"
                if controller.hazard.encounters or selected.signaller.detected_count
                else f"Sensors  {sensor_count}/{parameters.segment_count}  /  {kinds}"
            ),
            True,
            MUTED,
        ),
        (808, top + 321),
    )
    last_capture = central.capture_reports[-1] if central.capture_reports else None
    report = (
        f"Last capture: arm {last_capture.arm_index + 1}, food {last_capture.object_id}"
        if last_capture
        else "No captures reported"
    )
    if selected.signaller.notification != "None":
        report = f"Central notice: {selected.signaller.notification}"
    screen.blit(
        shortcuts_font.render(report, True, ACCENT if last_capture else MUTED), (808, top + 342)
    )

    draw_card(screen, pygame.Rect(794, top + 370, 306, 270), "KEYBOARD & MOUSE", font)
    shortcuts = [
        ("1-8 / Tab", "Select arm", True),
        ("Click / drag", "Reach or carry", True),
        ("X", "Hold pose", True),
        ("R", "Reset pose", True),
        ("T", "Retract food", selected.grip is not None),
        ("G", "Release food", selected.grip is not None),
        ("F + click", "Place food", True),
        ("O / H + click", "Obstacle / hazard", True),
        ("Right drag", "Move object", True),
        ("Del", "Remove object", True),
        ("Space", "Pause / resume", True),
        ("N / V", "Step / speed", True),
        ("D", "Restart setup", True),
        ("Esc", "Quit", True),
    ]
    # Present each shortcut on one row: a fixed keypress column makes actions
    # easy to scan without alternating between separate groups of key labels.
    screen.blit(shortcuts_font.render("KEYPRESS", True, MUTED), (808, top + 401))
    screen.blit(shortcuts_font.render("ACTION", True, MUTED), (919, top + 401))
    pygame.draw.line(screen, (60, 79, 96), (808, top + 417), (1086, top + 417))
    for index, (keys, description, enabled) in enumerate(shortcuts):
        y = top + 425 + index * 15
        if index % 2 == 0:
            pygame.draw.rect(screen, (27, 43, 58), (804, y - 3, 286, 15), border_radius=3)
        screen.blit(shortcuts_font.render(keys, True, TEXT if enabled else MUTED), (808, y))
        screen.blit(shortcuts_font.render(description, True, MUTED), (919, y))


def draw_arm(
    screen: pygame.Surface,
    arm: ControlledArm,
    index: int,
    selected: bool,
    font: pygame.font.Font,
    camera: SceneCamera = SceneCamera(),
    playback_speed: int = 1,
) -> None:
    """Draw one independent arm and, while assigned, its numbered target.

    :param screen: Destination surface, already clipped to the world.
    :param arm: Local arm state to read without changing it.
    :param index: Zero-based identity used for colour and target numbering.
    :param selected: Whether to emphasise this arm's joints and target.
    :param font: Font for matching target and arm numbers.
    :param camera: Presentation transform; never changes arm geometry.
    """
    controller = arm.controller
    colour = ARM_COLOURS[index]

    # Read the pose once and pair neighbouring points to draw fixed segments.
    # Tapering is decoration only; width never changes the movement calculation.
    points = [camera.project(point) for point in controller.arm.points]
    for segment, (start, end) in enumerate(zip(points, points[1:])):
        width = max(2, 9 - segment // 3) + (2 if selected else 0)
        pygame.draw.line(screen, colour, start, end, width)
        if selected:
            pygame.draw.circle(screen, TEXT, start, 2)
    pygame.draw.circle(screen, TEXT if selected else colour, points[-1], 4)

    # White rings mark the nearest point of each sensing segment to an object.
    # Sensors are independent of target ownership: any arm can feel any object.
    for contact in arm.contacts:
        pygame.draw.circle(screen, TEXT, camera.project(contact.position), 5, 2)

    if not arm.active:
        return

    # Matching colours and numbers identify ownership even when targets overlap.
    # A filled centre indicates arrival, not sensing or grasping an object.
    target = (
        arm.carry_goal if arm.grip is not None and arm.carry_goal is not None else controller.target
    )
    target = camera.project(target)
    pygame.draw.circle(screen, colour, target, 12 if selected else 9, 2)
    if controller.status == "Reached":
        pygame.draw.circle(screen, colour, target, 3)
    label = font.render(str(index + 1), True, colour)
    screen.blit(label, (target[0] + 14, target[1] - 8))
