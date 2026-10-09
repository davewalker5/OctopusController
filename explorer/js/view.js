import { ObjectKind } from './sensing.js';
export const WORLD = Object.freeze({
  left: 20,
  top: 90,
  width: 760,
  height: 670,
});
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
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext('2d');
  }
  /** Resize only the backing store, retaining scenario coordinates and aspect ratio. */
  resize() {
    const ratio = window.devicePixelRatio || 1,
      rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width * ratio)),
      height = Math.max(1, Math.round(rect.height * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
  }
  /** Convert a pointer to the same logical frame used by Python scenarios. */
  point(event) {
    const rect = this.canvas.getBoundingClientRect();
    return [
      Math.max(20, Math.min(779, 20 + ((event.clientX - rect.left) / rect.width) * 760)),
      Math.max(90, Math.min(759, 90 + ((event.clientY - rect.top) / rect.height) * 670)),
    ];
  }
  circle(point, radius, colour, fill = false, width = 1) {
    const c = this.context;
    c.beginPath();
    c.arc(...point, radius, 0, Math.PI * 2);
    c.lineWidth = width;
    c.strokeStyle = colour;
    c.fillStyle = colour;
    fill ? c.fill() : c.stroke();
  }
  line(points, colour, width) {
    const c = this.context;
    c.beginPath();
    c.moveTo(...points[0]);
    points.slice(1).forEach((p) => c.lineTo(...p));
    c.strokeStyle = colour;
    c.lineWidth = width;
    c.stroke();
  }
  draw(simulation, selectedIndex, selectedObject) {
    this.resize();
    const c = this.context,
      central = simulation.central;
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
      c.fillText(
        `${food ? 'F' : 'O'}${obj.identifier}`,
        obj.centre[0] - obj.radius,
        obj.centre[1] - obj.radius - 6,
      );
      if (owners.has(obj.identifier))
        this.circle(obj.centre, obj.radius + 4, ARM_COLOURS[owners.get(obj.identifier)], false, 2);
      if (obj.identifier === selectedObject) this.circle(obj.centre, obj.radius + 7, '#e5e7eb');
    }
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
