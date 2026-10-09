# Octopus Controller — Web Explorer

A standalone HTML, CSS and JavaScript port of the Python controller, styled to match the Field Notes Chromatophore Explorer. Simulation and drawing run entirely in the browser. There are no runtime libraries, remote assets, uploads or build steps.

## Preview locally

From the repository root:

```sh
python3 -m http.server 8765 --bind 127.0.0.1
```

Open the explorer at the following URL:

```text
http://127.0.0.1:8765/explorer/
```

Any static HTTP server works; Python is only a convenient preview server. Opening JavaScript modules through `file://` is unsupported.

## Using the explorer

- **Starting conditions:** select the original demonstration, behaviour showcase or two-target example, or load a local version 1 JSON scenario. Files are read locally and are never uploaded. Loaded scenarios start paused. The original demonstration starts running on first visit unless reduced motion is requested.
- **Playback:** Play/Pause suspends or resumes the simulation. Step advances exactly 1/120 second while paused. Restart restores the selected starting conditions, clears time and capture history, selects Arm 1 and pauses.
- **Arm selection:** use the numbered arm cards or dropdown. Each arm has its own geometry, destination and state. A highlighted arm does not receive priority in the simulation.
- **Destinations:** use the destination canvas tool or enter X/Y coordinates and choose Assign. When an arm holds food, the destination applies to the food; otherwise it applies to the tip.
- **Arm actions:** Idle holds the pose and any grip. Reset pose rebuilds only the selected arm and releases food. Retract carries held food towards its attachment. Release leaves food in place and idles the arm.
- **Parameters:** change segment count, length, relative bend or turning speed, then Apply. Geometry edits reset the selected arm and release food; speed-only changes retain the pose and grip. Bend and speed are displayed in degrees and degrees per second.
- **Scene editing:** select a canvas tool for one-click food/obstacle placement or object selection and dragging. The object list and coordinate fields offer keyboard alternatives. Moving or deleting held food releases it. Scene edits while paused refresh contacts without advancing time.
- **Capture log:** shows the last five captures. The model retains the last 32 reports and a cumulative count.

Interactive parameter edits are clamped to the Python interface ranges: 2–40 segments, lengths 6–30 pixels, bends 5–180 degrees and speeds 0.25–4 radians/second (about 14.32–229.18 degrees/second). Scenario files may supply wider valid ranges, including up to 100 segments. Those values remain unchanged unless that particular field is edited. Applying an unchanged form does not reset geometry or round the underlying parameters.

World coordinates match Python: x increases rightwards, y downwards. The visible region is x = 20–779, y = 90–759. Resizing changes only the display scale. Off-screen coordinates in files or coordinate forms remain valid and are clipped in the view; pointer dragging is clamped to the visible world.

### Keyboard shortcuts

Shortcuts apply within the explorer and outside editable fields:

| Key    | Action                                                     |
| ------ | ---------------------------------------------------------- |
| 1–8    | Select arm                                                 |
| Space  | Play/pause; focused buttons retain native Space activation |
| N      | One tick while paused                                      |
| D      | Restart starting conditions                                |
| R      | Reset selected arm's pose                                  |
| X      | Idle selected arm                                          |
| T      | Retract held food                                          |
| G      | Release held food                                          |
| F / O  | Choose food / obstacle placement                           |
| Delete | Delete selected object                                     |

Tab retains normal navigation. All actions also have labelled controls. Hidden tabs suspend animation without catching up on return. A change to reduced-motion preference pauses playback; Play always remains available.

## Scenarios and validation

See the scenario specification. Both bundled files are copied unchanged from the repository's `scenarios/` directory; tests detect drift.

The strict loader retains duplicate keys and numeric notation during parsing, so rules such as rejecting `"version": 1.0` and `"segment_count": 20.0` agree with Python. Validation is completed before replacing the scene. Malformed UTF-8 files are rejected rather than silently repaired. Errors preserve the current experiment. Food references resolve to the initial food position, not a moving target.

Restart rebuilds the validated conditions held in memory. To load edits made on disk, choose the file again. The last loaded local file remains available in the scenario selector for the current page session. No saved-session or export format is introduced.

## Model and source organisation

| Module               | Responsibility                                                 |
| -------------------- | -------------------------------------------------------------- |
| `geometry.js`        | Coordinates, projection and connected joint positions          |
| `segment-control.js` | Neighbour requests and bounded local turns                     |
| `model.js`           | Arm parameters, poses and reach controller                     |
| `sensing.js`         | Immutable scene snapshots, object identities and contacts      |
| `avoidance.js`       | Local steering and conservative swept arm checks               |
| `grasping.js`        | Adjacent contact qualification and payload attachment/sweep    |
| `organism.js`        | Arm lifecycle, ownership and stable central scheduling         |
| `scenario.js`        | Strict JSON parsing, validation and fresh scene construction   |
| `simulation.js`      | Fixed-step playback, pause and restart                         |
| `view.js`            | Canvas rendering and pointer coordinate conversion             |
| `app.js`             | HTML controls, async loading and visibility-aware presentation |

The model retains 1/120-second ticks, the 0.1-second frame contribution cap and stable arm-update order. Each segment uses its previous tip-side neighbour's request; messages travel one segment per tick. The renderer never advances or modifies the model.

The controller is an illustrative experiment. Local rules can be trapped; body collision, arm-to-arm collision, friction and force-based grasping are not modelled. Arbitrarily large or extreme valid scenarios are not guaranteed to be interactive. The web port does not skip collision checks or simplify geometry to maintain frame rate.

## Development checks

Use Node.js 22 or newer and the project's Python 3.14 environment:

```sh
npm ci
npm test
npm run format:check
python -m pytest -q
python -m ruff check src tests
python -m ruff format --check src tests
```

The Node tests invoke `venv/bin/python` by default to produce fresh reference traces. Set `PYTHON` to another interpreter with access to the project when needed:

```sh
PYTHON=/path/to/python npm test
```

Parity checks compare joint angles, positions, neighbour messages, targets, contacts, states, ownership, capture reports and object positions with a 1e-7 absolute tolerance for continuous values and exact equality for discrete values. They exercise both supplied scenarios, contested food ownership, carrying, obstacle insertion/removal, retraction and external object edits. Strict scenario acceptance is compared against the Python loader. Separate tests cover collision boundaries, fixed-step timing, pause/restart and interrupted grasp dwell.

### Browser verification

Keep the preview server running, then install development browsers once:

```sh
npx playwright install chromium firefox webkit
npm run test:browser
BROWSER=firefox npm run test:browser
BROWSER=webkit npm run test:browser
```

The script checks visible controls, file errors, parameter preservation, grasp/release, keyboard shortcuts, pointer/touch placement, resizing, visibility handling and layouts down to 320 pixels. It records screenshots and 120 frame intervals per benchmark.

| Variable                   | Purpose                                                           |
| -------------------------- | ----------------------------------------------------------------- |
| `BASE_URL`                 | Explorer URL, including its hosting subdirectory                  |
| `BROWSER`                  | `chromium`, `firefox` or `webkit`                                 |
| `BROWSER_EXECUTABLE`       | Optional installed browser executable                             |
| `PLAYWRIGHT_MODULE`        | Optional path to a managed Playwright module                      |
| `PLAYWRIGHT_BROWSERS_PATH` | Optional test-browser cache location                              |
| `BROWSER_OUTPUT`           | Report/screenshot directory; default `/tmp/octopus-browser-check` |
