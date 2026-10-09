import { buildScenario, buildDemo } from './scenario.js';
import { validateDuration } from './geometry.js';
export const SIMULATION_STEP = 1 / 120;
export const MAXIMUM_FRAME_TIME = 0.1;
/** Playback state shared by browser controls and timing tests. */
export class Simulation {
  constructor(scenario = null) {
    this.scenario = scenario;
    this.restart();
  }
  restart() {
    Object.assign(this, this.scenario ? buildScenario(this.scenario) : buildDemo());
    this.paused = true;
    this.accumulator = 0;
  }
  setPaused(paused) {
    this.paused = paused;
    this.accumulator = 0;
  }
  singleStep() {
    if (this.paused) this.central.step(SIMULATION_STEP, this.environment);
  }
  /** Cap wall time as Python does, without changing numerical tick duration. */
  advance(seconds) {
    validateDuration(seconds);
    if (this.paused) return;
    this.accumulator += Math.min(seconds, MAXIMUM_FRAME_TIME);
    while (this.accumulator >= SIMULATION_STEP) {
      this.central.step(SIMULATION_STEP, this.environment);
      this.accumulator -= SIMULATION_STEP;
    }
  }
}
