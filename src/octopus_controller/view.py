"""Pygame drawing only; the renderer never changes the simulated arm."""

from math import degrees, hypot

import pygame

from octopus_controller.organism import BODY_RADIUS, CentralController, ControlledArm
from octopus_controller.sensing import ObjectKind, WorldObject

HEADER_ICON_SIZE = (64, 64)
SIDEBAR_TOP = 20
CONTROL_STACK_BOTTOM = SIDEBAR_TOP + 740
WINDOW_SIZE = (1120, CONTROL_STACK_BOTTOM + 20)
# Share the lower edge with the final card so the world and controls stay aligned.
WORLD = pygame.Rect(20, 90, 760, CONTROL_STACK_BOTTOM - 90)
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
) -> None:
    """Draw eight arms, assignments, selected-arm settings and keyboard reference.

    :param screen: Destination window surface.
    :param central: Body and independent arm controllers to display.
    :param paused: Whether automatic updates are suspended.
    :param font: Font for labels and measurements.
    :param heading_font: Larger font for the window heading.
    :param header_icon: App icon already scaled to fit the heading.
    :param selected_index: Zero-based arm to highlight and inspect.
    :param objects: Circular sensing objects to draw beneath the arms.
    :param selected_object: Object ID highlighted for moving or deletion.
    :param placement: One-click placement tool, or None for normal target assignment.
    :param shortcuts_font: Smaller shortcut font; omitted callers use an 18-pixel default.
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
    # Model coordinates already use screen pixels, with y increasing downwards;
    # no scaling or coordinate conversion is needed before drawing them.
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
        colour = (131, 209, 120) if obj.kind is ObjectKind.FOOD else (229, 155, 105)
        pygame.draw.circle(screen, colour, obj.centre, obj.radius, 2)
        label = font.render("F" if obj.kind is ObjectKind.FOOD else "O", True, colour)

        # Keep the type label outside the circle: a target at its centre should
        # not cover the letter that distinguishes food from an obstacle.
        screen.blit(label, (obj.centre[0] - obj.radius, obj.centre[1] - obj.radius - 17))
        if obj.identifier in owners:
            # The owner's colour makes a latched object distinguishable from
            # ordinary contact, even when its segment markers overlap the circle.
            pygame.draw.circle(
                screen, ARM_COLOURS[owners[obj.identifier]], obj.centre, obj.radius + 4, 2
            )

        if obj.identifier == selected_object:
            pygame.draw.circle(screen, TEXT, obj.centre, obj.radius + 5, 1)

    # Draw the selected arm last so it is easiest to follow where arms overlap.
    # Draw order affects only visibility; it never gives an arm movement priority.
    order = [index for index in range(len(central.arms)) if index != selected_index]
    order.append(selected_index)
    for index in order:
        draw_arm(screen, central.arm(index), index, index == selected_index, font)

    # The body marks fixed attachments. Only explicit circular obstacles block
    # movement; body and arm-to-arm collisions are outside this model.
    pygame.draw.circle(screen, (39, 61, 77), central.centre, BODY_RADIUS)
    pygame.draw.circle(screen, MUTED, central.centre, BODY_RADIUS, 2)

    # Number the bases after drawing the body so their labels remain readable.
    # Numbers, colours and the roster all refer to the same persistent arm IDs.
    for index, arm in enumerate(central.arms):
        base = arm.controller.arm.base
        label = font.render(str(index + 1), True, ARM_COLOURS[index])
        screen.blit(label, label.get_rect(center=base))

    # Restore drawing across the whole window before rendering the side panel.
    screen.set_clip(None)
    draw_sidebar(
        screen,
        central,
        selected_index,
        paused,
        font,
        shortcuts_font if shortcuts_font is not None else pygame.font.Font(None, 18),
    )
    if placement is not None:
        # Keep the active placement instruction close to the world where the
        # next click applies, separate from the permanent shortcut reference.
        hint = font.render(
            f"Click to place {placement.value.lower()}  /  press F or O to change tool", True, TEXT
        )
        box = hint.get_rect(topleft=(36, WORLD.bottom - 33)).inflate(16, 12)
        pygame.draw.rect(screen, (39, 61, 77), box, border_radius=6)
        screen.blit(hint, (36, WORLD.bottom - 33))


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
) -> None:
    """Group live arm status, selected settings and shortcuts into distinct cards.

    :param screen: Destination surface with world clipping removed.
    :param central: Live model to inspect without changing it.
    :param selected_index: Zero-based selected arm.
    :param paused: Whether automatic simulation updates are suspended.
    :param font: Shared label font.
    :param shortcuts_font: Smaller font for the keypress/action table.
    """
    # Anchor all three cards to the window margin rather than the world grid.
    # Keeping their positions relative to one top edge preserves the stack spacing.
    top = SIDEBAR_TOP
    draw_card(screen, pygame.Rect(794, top, 306, 164), "ARMS", font)
    # Keep run state visible without competing with the main window heading.
    run_label = font.render("PAUSED" if paused else "RUNNING", True, MUTED if paused else ACCENT)
    screen.blit(run_label, run_label.get_rect(topright=(1086, top + 12)))
    for index, arm in enumerate(central.arms):
        # Two columns shorten the roster while retaining stable number order:
        # read across each row, just as the shortcut pairs below are read.
        row = pygame.Rect(802 + (index % 2) * 146, top + 42 + (index // 2) * 28, 144, 25)
        if index == selected_index:
            pygame.draw.rect(screen, (39, 61, 77), row, border_radius=5)
            pygame.draw.rect(screen, ARM_COLOURS[index], row, width=1, border_radius=5)
        pygame.draw.circle(screen, ARM_COLOURS[index], (row.left + 10, row.centery), 3)
        label = font.render(f"{index + 1}  {arm.state.value}", True, ARM_COLOURS[index])
        screen.blit(label, (row.left + 20, row.top + 4))

    selected = central.arm(selected_index)
    controller = selected.controller
    parameters = controller.arm.parameters
    draw_card(
        screen,
        pygame.Rect(794, top + 176, 306, 222),
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
    screen.blit(shortcuts_font.render(distance, True, TEXT), (808, top + 213))

    # Align values and adjustment keys into fixed columns so changing a number
    # does not move the shortcut hints. All four settings affect only this arm.
    settings = [
        ("Segments", str(parameters.segment_count), "[  ]"),
        ("Length", f"{parameters.segment_length:.0f} px", "-  ="),
        ("Bend", f"{degrees(parameters.maximum_bend):.0f} deg", ",  ."),
        ("Speed", f"{parameters.turning_speed:.2f} rad/s", ";  '"),
    ]
    for index, (label, value, keys) in enumerate(settings):
        y = top + 238 + index * 24
        screen.blit(shortcuts_font.render(label, True, MUTED), (808, y + 3))
        screen.blit(shortcuts_font.render(value, True, TEXT), (913, y + 3))
        draw_keycap(screen, keys, (1048, y), shortcuts_font)

    sensor_count = len({contact.segment_index for contact in selected.contacts})
    kinds = ", ".join(sorted({contact.kind.value for contact in selected.contacts})) or "None"
    pygame.draw.line(screen, (39, 56, 72), (808, top + 342), (1086, top + 342))
    screen.blit(
        shortcuts_font.render(
            f"Sensors  {sensor_count}/{parameters.segment_count}  /  {kinds}", True, MUTED
        ),
        (808, top + 352),
    )
    last_capture = central.capture_reports[-1] if central.capture_reports else None
    report = (
        f"Captured: arm {last_capture.arm_index + 1}, food {last_capture.object_id}"
        if last_capture
        else "No captures reported"
    )
    screen.blit(
        shortcuts_font.render(report, True, ACCENT if last_capture else MUTED), (808, top + 376)
    )

    draw_card(screen, pygame.Rect(794, top + 410, 306, 330), "KEYBOARD & MOUSE", font)
    shortcuts = [
        ("1-8 / Tab", "Select arm", True),
        ("Click / drag", "Reach or carry", True),
        ("X", "Hold pose", True),
        ("R", "Reset pose", True),
        ("T", "Retract food", selected.grip is not None),
        ("G", "Release food", selected.grip is not None),
        ("F + click", "Place food", True),
        ("O + click", "Place obstacle", True),
        ("Right drag", "Move object", True),
        ("Del", "Remove object", True),
        ("Space", "Pause / resume", True),
        ("N", "Step (paused)", paused),
        ("D", "Reset all", True),
        ("Esc", "Quit", True),
    ]
    # Present each shortcut on one row: a fixed keypress column makes actions
    # easy to scan without alternating between separate groups of key labels.
    screen.blit(shortcuts_font.render("KEYPRESS", True, MUTED), (808, top + 445))
    screen.blit(shortcuts_font.render("ACTION", True, MUTED), (919, top + 445))
    pygame.draw.line(screen, (60, 79, 96), (808, top + 462), (1086, top + 462))
    for index, (keys, description, enabled) in enumerate(shortcuts):
        y = top + 469 + index * 19
        if index % 2 == 0:
            pygame.draw.rect(screen, (27, 43, 58), (804, y - 3, 286, 19), border_radius=3)
        screen.blit(shortcuts_font.render(keys, True, TEXT if enabled else MUTED), (808, y))
        screen.blit(shortcuts_font.render(description, True, MUTED), (919, y))


def draw_arm(
    screen: pygame.Surface,
    arm: ControlledArm,
    index: int,
    selected: bool,
    font: pygame.font.Font,
) -> None:
    """Draw one independent arm and, while assigned, its numbered target.

    :param screen: Destination surface, already clipped to the world.
    :param arm: Local arm state to read without changing it.
    :param index: Zero-based identity used for colour and target numbering.
    :param selected: Whether to emphasise this arm's joints and target.
    :param font: Font for matching target and arm numbers.
    """
    controller = arm.controller
    colour = ARM_COLOURS[index]

    # Read the pose once and pair neighbouring points to draw fixed segments.
    # Tapering is decoration only; width never changes the movement calculation.
    points = controller.arm.points
    for segment, (start, end) in enumerate(zip(points, points[1:])):
        width = max(2, 9 - segment // 3) + (2 if selected else 0)
        pygame.draw.line(screen, colour, start, end, width)
        if selected:
            pygame.draw.circle(screen, TEXT, start, 2)
    pygame.draw.circle(screen, TEXT if selected else colour, points[-1], 4)

    # White rings mark the nearest point of each sensing segment to an object.
    # Sensors are independent of target ownership: any arm can feel any object.
    for contact in arm.contacts:
        pygame.draw.circle(screen, TEXT, contact.position, 5, 2)

    if not arm.active:
        return

    # Matching colours and numbers identify ownership even when targets overlap.
    # A filled centre indicates arrival, not sensing or grasping an object.
    target = (
        arm.carry_goal if arm.grip is not None and arm.carry_goal is not None else controller.target
    )
    pygame.draw.circle(screen, colour, target, 12 if selected else 9, 2)
    if controller.status == "Reached":
        pygame.draw.circle(screen, colour, target, 3)
    label = font.render(str(index + 1), True, colour)
    screen.blit(label, (target[0] + 14, target[1] - 8))
