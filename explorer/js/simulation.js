/**
 * Fixed-step playback adapter, corresponding to the timing part of app.py.
 *
 * The browser supplies elapsed wall time; this class alone converts it into
 * simulation ticks. Keeping it DOM-free lets tests exercise pause, step, frame
 * capping and restart with the same path used by interactive playback.
 */

import { buildScenario, buildDemo } from './scenario.js';
import { validateDuration } from './geometry.js';
// Tick duration in simulation seconds, matching the Python application.
export const SIMULATION_STEP = 1 / 120;
// Discard excess wall time after a stall rather than running a huge catch-up.
export const MAXIMUM_FRAME_TIME = 0.1;
/** Playback state shared by browser controls and timing tests. */
export class Simulation {
  /**
   * Create paused playback from retained initial conditions.
   * @param {import('./scenario.js').Scenario|null} [scenario=null] Validated setup, or the demo.
   */
  constructor(scenario = null) {
    this.scenario = scenario;
    this.restart();
  }
  /**
   * Rebuild poses, scene objects, time and capture history from the starting setup.
   *
   * Fresh mutable models prevent grips or moved food leaking across restarts.
   * Browser restarts always pause; app.js may start the initial demo explicitly.
   * @returns {void} Replaces central/environment and discards partial tick time.
   */
  restart() {
    Object.assign(this, this.scenario ? buildScenario(this.scenario) : buildDemo());
    this.paused = true;
    this.accumulator = 0;
  }
  /**
   * Set playback state and discard any partially accumulated tick.
   * @param {boolean} paused True to suspend normal advancement.
   * @returns {void} Does not reset poses, assignments, grip dwell or simulation time.
   */
  setPaused(paused) {
    this.paused = paused;
    this.accumulator = 0;
  }
  /**
   * Advance exactly one complete model tick only while paused.
   * @returns {void} Uses the full sensing/grasp/carry path, not only arm reaching.
   */
  singleStep() {
    if (this.paused) this.central.step(SIMULATION_STEP, this.environment);
  }
  /**
   * Accumulate frame time and run complete fixed-size model ticks.
   *
   * The unconsumed fraction carries into the next frame. The 0.1-second cap
   * intentionally discards extra wall time after a stall, so simulation time
   * need not match elapsed real time. Hidden-tab timestamp handling is in app.js.
   *
   * @param {number} seconds Finite non-negative elapsed frame time in seconds.
   * @returns {void} Advances the model unless paused; never performs drawing.
   * @throws {Error} If the supplied duration is invalid.
   */
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
