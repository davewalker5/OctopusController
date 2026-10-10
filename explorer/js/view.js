/**
 * Read-only Canvas 2D presentation, corresponding to view.py.
 *
 * The renderer consumes arm/contact/ownership state already produced by the
 * model. It must not calculate sensing, adjust angles or advance time. Drawing
 * coordinates retain the Python world's pixel units; CSS and backing-store
 * scaling provide the responsive browser view.
 */

import { ObjectKind } from './sensing.js';
// Match the Python world rectangle, including its original offset within the window.
// These are presentation/clipping bounds, not walls or scenario validation limits.
export const WORLD = Object.freeze({
  left: 20,
  top: 90,
  width: 760,
  height: 670,
});
// Stable identities follow the Python palette; numbers also identify arms without colour.
export const ARM_COLOURS = [
  '#4fd4b8',
  '#6db1f4',
  '#b496f5',
  '#ef92c5',
  '#f8a774',
  '#e2d073',
  '#95ce77',
  '#72d0df',
];
/** Canvas-only presentation; methods never change simulation state. */
export class SceneView {
  /**
   * Bind the scene to a canvas with Canvas 2D support.
   * @param {HTMLCanvasElement} canvas Destination owned by the explorer page.
   */
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext('2d');
  }
  /**
   * Match the canvas backing resolution to its CSS size and device pixel density.
   * @returns {void} Updates dimensions only when changed; never touches the model.
   */
  resize() {
    const ratio = window.devicePixelRatio || 1,
      rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width * ratio)),
      height = Math.max(1, Math.round(rect.height * ratio));
    // Assigning either canvas dimension clears its pixels and drawing state.
    // Avoid redundant assignments every frame; draw() restores the transform
    // and styles when a resize really does recreate the backing store.
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
  }
  /**
   * Convert viewport pointer coordinates back into the original world rectangle.
   * @param {PointerEvent} event Pointer event with clientX/clientY in CSS pixels.
   * @returns {import('./geometry.js').Point} World position clamped to x 20–779, y 90–759.
   */
  point(event) {
    const rect = this.canvas.getBoundingClientRect();
    return [
      Math.max(20, Math.min(779, 20 + ((event.clientX - rect.left) / rect.width) * 760)),
      Math.max(90, Math.min(759, 90 + ((event.clientY - rect.top) / rect.height) * 670)),
    ];
  }
  /**
   * Draw a filled or outlined circle in the current world-to-canvas transform.
   * @param {import('./geometry.js').Point} point Centre in world pixels.
   * @param {number} radius Radius in world pixels.
   * @param {string} colour Canvas-compatible colour.
   * @param {boolean} [fill=false] Fill instead of stroke when true.
   * @param {number} [width=1] Outline width in world pixels; ignored for fill.
   * @returns {void} Mutates only the canvas context.
   */
  circle(point, radius, colour, fill = false, width = 1) {
    const c = this.context;
    c.beginPath();
    c.arc(...point, radius, 0, Math.PI * 2);
    c.lineWidth = width;
    c.strokeStyle = colour;
    c.fillStyle = colour;
    fill ? c.fill() : c.stroke();
  }
  /**
   * Stroke a connected polyline without changing any supplied coordinates.
   * @param {import('./geometry.js').Point[]} points Nonempty sequence of world positions.
   * @param {string} colour Canvas-compatible stroke colour.
   * @param {number} width Decorative stroke width in world pixels, not collision clearance.
   * @returns {void} Mutates only the canvas context.
   */
  line(points, colour, width) {
    const c = this.context;
    c.beginPath();
    c.moveTo(...points[0]);
    points.slice(1).forEach((p) => c.lineTo(...p));
    c.strokeStyle = colour;
    c.lineWidth = width;
    c.stroke();
  }
  /**
   * Render one complete scene from current state and presentation selection.
   * @param {import('./simulation.js').Simulation} simulation Source of arms and environment.
   * @param {number} selectedIndex Valid zero-based arm index, drawn last for visibility.
   * @param {number|null} selectedObject Object identity to highlight, or null.
   * @returns {void} Clears/redraws the canvas; never senses, assigns goals or advances time.
   */
  draw(simulation, selectedIndex, selectedObject) {
    this.resize();
    const c = this.context,
      central = simulation.central;
    // Translate the original (20, 90) origin to the canvas corner, then scale
    // world pixels to backing pixels. Resize changes this view only; targets,
    // link lengths and object coordinates retain their original world values.
    c.setTransform(
      this.canvas.width / 760,
      0,
      0,
      this.canvas.height / 670,
      (-20 * this.canvas.width) / 760,
      (-90 * this.canvas.height) / 670,
    );
    c.fillStyle = '#162533';
    c.fillRect(20, 90, 760, 670);
    c.lineCap = 'round';
    c.lineJoin = 'round';
    // A 40-pixel grid is a distance reference, not part of the movement rules.
    // Canvas edges clip long/off-screen geometry without imposing collisions.
    for (let x = 20; x < 780; x += 40)
      this.line(
        [
          [x, 90],
          [x, 760],
        ],
        '#1d2e3d',
        1,
      );
    for (let y = 90; y < 760; y += 40)
      this.line(
        [
          [20, y],
          [780, y],
        ],
        '#1d2e3d',
        1,
      );
    // Ownership rings reflect the model's grips; drawing never awards a claim.
    // Objects are painted before arms so local contact markers remain visible.
    const owners = new Map(
      central.arms.flatMap((arm, i) => (arm.grip ? [[arm.grip.objectId, i]] : [])),
    );
    c.font = '12px system-ui';
    c.textAlign = 'left';
    for (const obj of simulation.environment.objects) {
      const food = obj.kind === ObjectKind.FOOD,
        colour = food ? '#83d178' : '#e59b69';
      this.circle(obj.centre, obj.radius, food ? '#203f35' : '#44382e', true);
      this.circle(obj.centre, obj.radius, colour, false, 2);
      c.fillStyle = colour;
      // Put the type letter outside the circle so a destination cross at its
      // centre does not obscure the non-colour distinction between food/obstacle.
      c.fillText(
        `${food ? 'F' : 'O'}${obj.identifier}`,
        obj.centre[0] - obj.radius,
        obj.centre[1] - obj.radius - 6,
      );
      if (owners.has(obj.identifier))
        this.circle(obj.centre, obj.radius + 4, ARM_COLOURS[owners.get(obj.identifier)], false, 2);
      if (obj.identifier === selectedObject) this.circle(obj.centre, obj.radius + 7, '#e5e7eb');
    }
    // Selecting an arm changes paint order only, never scheduling priority.
    const order = central.arms
      .map((_, i) => i)
      .filter((i) => i !== selectedIndex)
      .concat(selectedIndex);
    for (const i of order) {
      const arm = central.arm(i),
        points = arm.controller.arm.points,
        selected = i === selectedIndex,
        colour = ARM_COLOURS[i];
      if (selected) this.line(points, '#c5edff24', 12);
      this.line(points, colour, selected ? 4 : 3);
      points.slice(1).forEach((p) => this.circle(p, selected ? 2.7 : 2, colour, true));
      for (const contact of arm.contacts)
        this.circle(
          contact.position,
          4,
          contact.kind === ObjectKind.FOOD ? '#fbbf24' : '#fb7185',
          true,
        );
      if (arm.active) {
        // A carrying arm's public destination is the food centre. Its internal
        // tip objective is continually corrected and would mislead the visitor.
        const target = arm.grip && arm.carryGoal ? arm.carryGoal : arm.controller.target;
        this.circle(target, 8, colour, false, selected ? 2 : 1);
        this.line(
          [
            [target[0] - 12, target[1]],
            [target[0] + 12, target[1]],
          ],
          colour,
          1,
        );
        this.line(
          [
            [target[0], target[1] - 12],
            [target[0], target[1] + 12],
          ],
          colour,
          1,
        );
        c.fillStyle = colour;
        c.fillText(String(i + 1), target[0] + 12, target[1] - 9);
      }
    }
    // The body marks fixed attachments but is not itself a collision obstacle.
    // Labels follow the body fill so all eight persistent identities stay legible.
    this.circle(central.centre, 32, '#273d4d', true);
    this.circle(central.centre, 32, '#8fa4b5', false, 1.5);
    c.font = 'bold 12px system-ui';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    central.arms.forEach((arm, i) => {
      const base = arm.controller.arm.base;
      this.circle(base, 10, '#162533', true);
      c.fillStyle = ARM_COLOURS[i];
      c.fillText(String(i + 1), ...base);
    });
    c.textBaseline = 'alphabetic';
  }
}
