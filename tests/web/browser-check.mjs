/** Functional, responsive and performance checks against the deployed static folder. */
import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
const playwright = await import(process.env.PLAYWRIGHT_MODULE ?? 'playwright');
const engine = process.env.BROWSER ?? 'chromium';
const browser = await playwright[engine].launch({
  headless: true,
  ...(process.env.BROWSER_EXECUTABLE ? { executablePath: process.env.BROWSER_EXECUTABLE } : {}),
});
const url = process.env.BASE_URL ?? 'http://127.0.0.1:8765/explorer/';
const output = process.env.BROWSER_OUTPUT ?? '/tmp/octopus-browser-check';
await mkdir(output, { recursive: true });
const failures = [],
  metrics = [];
/** Observe errors and verify that the standalone page makes no external requests. */
function watch(page) {
  page.on('pageerror', (error) => failures.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') failures.push(message.text());
  });
  page.on('response', (response) => {
    if (response.status() >= 400) failures.push(`${response.status()} ${response.url()}`);
  });
  page.on('request', (request) => {
    if (new URL(request.url()).origin !== new URL(url).origin)
      failures.push(`External request: ${request.url()}`);
  });
}
/** Compare complete model state; canvas antialiasing can change with GPU readback. */
const snapshot = (page) =>
  page.evaluate(async () => {
    const { simulation } = await import('./js/app.js');
    return JSON.stringify({ central: simulation.central, objects: simulation.environment.objects });
  });
async function upload(page, text) {
  await page.locator('#scenario-file').setInputFiles({
    name: 'test.json',
    mimeType: 'application/json',
    buffer: Buffer.from(text),
  });
}
async function settled(page) {
  await page.waitForTimeout(160);
}
/** Collect actual display intervals, retaining slow frames rather than discarding them. */
async function measure(page, label) {
  const intervals = await page.evaluate(
    () =>
      new Promise((resolve) => {
        const values = [];
        let previous;
        function frame(now) {
          if (previous !== undefined) values.push(now - previous);
          previous = now;
          if (values.length === 120) resolve(values);
          else requestAnimationFrame(frame);
        }
        requestAnimationFrame(frame);
      }),
  );
  const sorted = [...intervals].sort((a, b) => a - b);
  metrics.push({
    label,
    medianMs: sorted[60],
    p95Ms: sorted[114],
    maxMs: sorted.at(-1),
  });
}
try {
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1100 },
    reducedMotion: 'reduce',
  });
  watch(page);
  await page.goto(url);
  await page.waitForSelector('#arm-7');
  await settled(page);
  assert.equal(await page.locator('#playback-state').textContent(), 'Paused');
  const initial = await snapshot(page);
  await settled(page);
  assert.equal(await snapshot(page), initial);
  await page.screenshot({
    path: `${output}/${engine}-desktop.png`,
    fullPage: true,
  });
  await page.locator('#step').click();
  await settled(page);
  assert.match(await page.locator('#clock').textContent(), /^0.01 s/);
  await page.locator('#restart').click();
  assert.equal(await snapshot(page), initial);
  await page.locator('#play').click();
  await settled(page);
  assert.equal(await page.locator('#step').isDisabled(), true);
  await page.locator('#play').click();
  assert.equal(await page.locator('#playback-state').textContent(), 'Paused');
  for (const preset of ['two-targets', 'behaviour-showcase']) {
    await page.locator('#scenario').selectOption(preset);
    await page.waitForFunction(() =>
      document.querySelector('#clock').textContent.startsWith('0.00'),
    );
    await settled(page);
    assert.equal(await page.locator('#playback-state').textContent(), 'Paused');
  }
  await page.locator('#arm-2').click();
  assert.equal(await page.locator('#selected-arm').inputValue(), '2');
  await page.locator('#target-x').fill('600');
  await page.locator('#target-y').fill('400');
  await page.locator('#target-form button').click();
  await page.locator('#target-x').focus();
  await page.keyboard.press('1');
  assert.equal(await page.locator('#selected-arm').inputValue(), '2');
  await page.locator('#arm-2').focus();
  await page.keyboard.press('8');
  assert.equal(await page.locator('#selected-arm').inputValue(), '7');
  await page.locator('#arm-7').focus();
  await page.keyboard.press('Tab');
  assert.notEqual(await page.evaluate(() => document.activeElement.id), 'arm-7');
  await page.locator('#segment-count').fill('24');
  await page.locator('#parameters-form button').click();
  assert.equal(await page.locator('#segment-count').inputValue(), '24');
  await page.locator('#object-x').fill('300');
  await page.locator('#object-y').fill('300');
  await page.locator('[name="operation"][value="Food"]').click();
  assert.equal(await page.locator('#selected-object').inputValue(), '3');
  await page.locator('#object-x').fill('310');
  await page.locator('#move-object').click();
  assert.equal(await page.locator('#object-x').inputValue(), '310');
  await page.locator('#delete-object').click();
  assert.equal(await page.locator('#delete-object').isDisabled(), true);
  const beforeInvalid = await snapshot(page),
    title = await page.locator('#world-title').textContent();
  await upload(page, '{"version":1,"version":1}');
  await page.waitForSelector('#error:not([hidden])');
  assert.match(await page.locator('#error').textContent(), /Duplicate/);
  assert.equal(await snapshot(page), beforeInvalid);
  assert.equal(await page.locator('#world-title').textContent(), title);
  await page.locator('#scenario-file').setInputFiles({
    name: 'invalid-utf8.json',
    mimeType: 'application/json',
    buffer: Buffer.concat([
      Buffer.from('{"version":1,"name":"'),
      Buffer.from([255]),
      Buffer.from('"}'),
    ]),
  });
  await page.waitForFunction(() =>
    document.querySelector('#error').textContent.includes('invalid-utf8.json'),
  );
  assert.equal(await snapshot(page), beforeInvalid);
  await upload(
    page,
    '{"version":1,"name":"Keyboard lab","defaults":{"segment_count":60,"turning_speed_degrees":2}}',
  );
  await page.waitForFunction(
    () => document.querySelector('#world-title').textContent === 'Keyboard lab',
  );
  assert.equal(await page.locator('#segment-count').inputValue(), '60');
  assert.equal(await page.locator('#turning-speed').inputValue(), '2');
  await page.locator('#parameters-form button').click();
  assert.equal(await page.locator('#segment-count').inputValue(), '60');
  assert.equal(await page.locator('#turning-speed').inputValue(), '2');
  await page.locator('#segment-count').fill('50');
  await page.locator('#parameters-form button').click();
  assert.equal(await page.locator('#segment-count').inputValue(), '40');
  assert.equal(await page.locator('#turning-speed').inputValue(), '2');
  // Explicit scene tools: one-click food placement, object drag, and target mapping.
  await page.locator('#tool').selectOption('Food');
  await page.locator('#world').scrollIntoViewIfNeeded();
  let box = await page.locator('#world').boundingBox();
  await page.mouse.click(box.x + box.width * 0.4, box.y + box.height * 0.3);
  assert.equal(await page.locator('#tool').inputValue(), 'target');
  assert.ok(Math.abs(Number(await page.locator('#object-x').inputValue()) - 324) < 2);
  await page.locator('#tool').selectOption('object');
  await page.locator('#world').scrollIntoViewIfNeeded();
  box = await page.locator('#world').boundingBox();
  await page.mouse.move(box.x + box.width * 0.4, box.y + box.height * 0.3);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.4);
  await page.mouse.up();
  assert.ok(Math.abs(Number(await page.locator('#object-x').inputValue()) - 400) < 2);
  const beforeResize = await snapshot(page);
  const beforeSize = await page
    .locator('#world')
    .evaluate((c) => [
      c.width,
      c.height,
      c.getBoundingClientRect().width,
      c.getBoundingClientRect().height,
    ]);
  await page.locator('#world').screenshot({ path: `${output}/before-resize.png` });
  await page.setViewportSize({ width: 800, height: 1100 });
  await settled(page);
  await page.setViewportSize({ width: 1440, height: 1100 });
  await settled(page);
  const afterSize = await page
    .locator('#world')
    .evaluate((c) => [
      c.width,
      c.height,
      c.getBoundingClientRect().width,
      c.getBoundingClientRect().height,
    ]);
  await page.locator('#world').screenshot({ path: `${output}/after-resize.png` });
  assert.deepEqual(afterSize, beforeSize);
  assert.equal(await snapshot(page), beforeResize, 'Paused model changes after resizing');
  // Deterministic UI grasp: food overlaps adjacent joints at the initial pose.
  const grasp = await page.evaluate(async () => {
    const { buildScenario, parseScenario } = await import('./js/scenario.js');
    const { central } = buildScenario(parseScenario('{"version":1}'));
    const point = central.arm(0).controller.arm.points[1];
    return JSON.stringify({
      version: 1,
      name: 'Grasp test',
      food: [{ position: point, radius: 4 }],
      arms: [{ number: 1, target: { position: point } }],
    });
  });
  await upload(page, grasp);
  await page.waitForFunction(
    () => document.querySelector('#world-title').textContent === 'Grasp test',
  );
  for (let i = 0; i < 30; i++) await page.locator('#step').click();
  assert.equal(await page.locator('#arm-state').textContent(), 'Holding');
  assert.match(await page.locator('#capture-log').textContent(), /captured food 1/);
  await page.locator('#turning-speed').fill('60');
  await page.locator('#parameters-form button').click();
  assert.equal(await page.locator('#arm-state').textContent(), 'Holding');
  await page.locator('[data-action="retract"]').click();
  assert.equal(await page.locator('#arm-state').textContent(), 'Retracting');
  await page.locator('[data-action="release"]').click();
  assert.equal(await page.locator('[data-action="release"]').isDisabled(), true);
  await page.locator('#restart').click();
  assert.match(await page.locator('#capture-log').textContent(), /No captures/);
  // Visibility lifecycle: emulate visibility transitions and make sure no hidden time advances.
  await page.locator('#play').click();
  await settled(page);
  await page.evaluate(() => {
    Object.defineProperty(document, 'hidden', {
      configurable: true,
      value: true,
    });
    document.dispatchEvent(new Event('visibilitychange'));
  });
  await settled(page);
  const hiddenClock = await page.locator('#clock').textContent();
  await page.waitForTimeout(300);
  assert.equal(await page.locator('#clock').textContent(), hiddenClock);
  await page.evaluate(() => {
    Object.defineProperty(document, 'hidden', {
      configurable: true,
      value: false,
    });
    document.dispatchEvent(new Event('visibilitychange'));
  });
  await settled(page);
  await page.locator('#play').click();
  for (const preset of ['demo', 'behaviour-showcase']) {
    await page.locator('#scenario').selectOption(preset);
    await settled(page);
    await page.locator('#play').click();
    await measure(page, preset);
    await page.locator('#play').click();
  }
  const heavy = {
    version: 1,
    name: 'Obstacle benchmark',
    defaults: { segment_count: 40 },
    obstacles: Array.from({ length: 16 }, (_, i) => ({
      position: [180 + (i % 4) * 145, 230 + Math.floor(i / 4) * 140],
      radius: 10,
    })),
    arms: Array.from({ length: 8 }, (_, i) => ({
      number: i + 1,
      target: {
        position: [
          400 + 180 * Math.cos((i * Math.PI) / 4),
          460 + 180 * Math.sin((i * Math.PI) / 4),
        ],
      },
    })),
  };
  await upload(page, JSON.stringify(heavy));
  await page.waitForFunction(
    () => document.querySelector('#world-title').textContent === 'Obstacle benchmark',
  );
  await page.locator('#play').click();
  await measure(page, '40 segments, 16 obstacles');
  await page.locator('#play').click();
  heavy.name = 'Large arm benchmark';
  heavy.defaults.segment_count = 100;
  await upload(page, JSON.stringify(heavy));
  await page.waitForFunction(
    () => document.querySelector('#world-title').textContent === 'Large arm benchmark',
  );
  await page.locator('#scene-play').click();
  await measure(page, '100 segments, 16 obstacles');
  await page.locator('#scene-play').click();
  for (const width of [800, 390, 320]) {
    await page.setViewportSize({ width, height: 1000 });
    await settled(page);
    assert.ok(
      await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth),
      `No page overflow at ${width}`,
    );
    assert.ok(
      await page
        .locator('select')
        .evaluateAll((elements) =>
          elements.every((element) => element.getBoundingClientRect().height >= 44),
        ),
      'Dropdowns retain touch-sized targets',
    );
    await page.screenshot({
      path: `${output}/${engine}-${width}.png`,
      fullPage: true,
    });
  }
  const mobile = await browser.newPage({
    viewport: { width: 390, height: 844 },
    deviceScaleFactor: 2,
    hasTouch: true,
    ...(engine === 'firefox' ? {} : { isMobile: true }),
    reducedMotion: 'reduce',
  });
  watch(mobile);
  await mobile.goto(url);
  await mobile.waitForSelector('#arm-7');
  await mobile.locator('#tool').selectOption('Food');
  await mobile.locator('#world').scrollIntoViewIfNeeded();
  const touchBox = await mobile.locator('#world').boundingBox();
  await mobile.touchscreen.tap(touchBox.x + touchBox.width / 2, touchBox.y + touchBox.height / 2);
  assert.equal(await mobile.locator('#selected-object').inputValue(), '3');
  await mobile.locator('#play').click();
  await measure(mobile, 'mobile emulation, default');
  await mobile.screenshot({
    path: `${output}/${engine}-touch.png`,
    fullPage: true,
  });
  assert.deepEqual(failures, []);
  const report = {
    engine,
    version: browser.version(),
    userAgent: await page.evaluate(() => navigator.userAgent),
    metrics,
    checks: 'All functional, layout, input, timing and local-resource checks passed.',
    limitations:
      'Mobile emulation is not a physical device benchmark. WebKit testing is not a release Safari run.',
  };
  await writeFile(`${output}/${engine}-report.json`, JSON.stringify(report, null, 2));
  console.log(JSON.stringify(report, null, 2));
} finally {
  await browser.close();
}
