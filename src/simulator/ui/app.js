import { config, resetConfig } from "./lib/config.js";
import { appState, Modes } from "./lib/appState.js";
import { getDomRefs, createUi } from "./lib/dom.js";
import {
  resizeCanvas,
  worldToScreen,
  screenToWorld,
  zoomAt,
  centerView,
  recenterArenaView,
  recenterOnRobot,
} from "./lib/view.js";
import { createRenderer } from "./lib/render.js";
import { randomizeSigns, createSignControls } from "./lib/signsControl.js";
import { measurePoseDistances } from "./lib/pose.js";
import { createModeController } from "./lib/modes.js";
import { createSimulator } from "./lib/simulator.js";
import { createSerialController } from "./lib/serial.js";
import { laneBounds, trafficSigns } from "./lib/traffic.js";
import {
  setLog,
  setCursorRefresh,
  setLastMouseWorld,
  setPoseFromUser,
  noteCollisionOnce,
} from "./lib/pose.js";
import { createWsTelemetry } from "./lib/wsTelemetry.js";
import { collidesWithAnything } from "./lib/geometry.js";
import { createRace } from "./lib/race.js";
import { getSeed, newSeed, setSeed } from "./lib/rng.js";

const dom = getDomRefs();
const { log, setStatus, notify } = createUi(dom);

const ROBOT_DRAG_HIT_PAD_PX = 10;
const isPlayTestMode = () => appState.currentMode === Modes.PLAY || appState.currentMode === Modes.TEST;
let logCollapsed = false;

const laneTransforms = [
  {
    name: "bottom",
    number: 1,
    dirVec: { x: 1, y: 0 }, // CCW along +x
    compute: (x, y, bounds, w, h) => ({
      xRel: x,
      yRel: y,
      alongMin: bounds.x0,
      alongLen: bounds.x1 - bounds.x0,
      perpMin: bounds.y0,
      perpLen: bounds.y1 - bounds.y0,
    }),
  },
  {
    name: "right",
    number: 2,
    dirVec: { x: 0, y: 1 }, // CCW along +y
    compute: (x, y, bounds, w, h) => ({
      xRel: y,
      yRel: w - x,
      alongMin: bounds.y0,
      alongLen: bounds.y1 - bounds.y0,
      perpMin: w - bounds.x1,
      perpLen: bounds.x1 - bounds.x0,
    }),
  },
  {
    name: "top",
    number: 3,
    dirVec: { x: -1, y: 0 }, // CCW along -x
    compute: (x, y, bounds, w, h) => ({
      xRel: w - x,
      yRel: h - y,
      alongMin: w - bounds.x1,
      alongLen: bounds.x1 - bounds.x0,
      perpMin: h - bounds.y1,
      perpLen: bounds.y1 - bounds.y0,
    }),
  },
  {
    name: "left",
    number: 4,
    dirVec: { x: 0, y: -1 }, // CCW along -y
    compute: (x, y, bounds, w, h) => ({
      xRel: h - y,
      yRel: x,
      alongMin: h - bounds.y1,
      alongLen: bounds.y1 - bounds.y0,
      perpMin: bounds.x0,
      perpLen: bounds.x1 - bounds.x0,
    }),
  },
];

function inferLaneDirection(lane, pose) {
  const prev = appState.prevPose;
  let vx = 0;
  let vy = 0;
  if (prev) {
    vx = pose.x - prev.x;
    vy = pose.y - prev.y;
  }
  const speed = Math.hypot(vx, vy);
  if (speed < 1e-4 && Number.isFinite(pose.headingDeg)) {
    const rad = (pose.headingDeg * Math.PI) / 180;
    vx = Math.cos(rad);
    vy = Math.sin(rad);
  }
  const dot = vx * lane.dirVec.x + vy * lane.dirVec.y;
  if (!Number.isFinite(dot) || Math.abs(dot) < 1e-6) return 1; // default CCW
  return dot >= 0 ? 1 : -1;
}

function orientValue(base, min, len, dirSign) {
  if (!Number.isFinite(base) || !Number.isFinite(min) || !Number.isFinite(len)) return base;
  if (dirSign >= 0) return base;
  return min + len - (base - min);
}

function getLaneRelativePose(pose) {
  if (!pose) return null;
  const { x, y } = pose;
  const width = config.arenaWidthMeters;
  const height = config.arenaHeightMeters;
  for (const lane of laneTransforms) {
    const bounds = laneBounds[lane.name];
    if (!bounds) continue;
    const within =
      x >= bounds.x0 &&
      x <= bounds.x1 &&
      y >= bounds.y0 &&
      y <= bounds.y1;
    if (!within) continue;
    const { xRel, yRel, alongMin, alongLen, perpMin, perpLen } = lane.compute(x, y, bounds, width, height);
    const dirSign = inferLaneDirection(lane, pose);
    return {
      ...lane,
      direction: dirSign >= 0 ? "ccw" : "cw",
      xRel: orientValue(xRel, alongMin, alongLen, dirSign),
      yRel,
    };
  }
  return null;
}

function refreshCursorReadout() {
  const robotPart = appState.latestPose
    ? `robot x: ${(appState.latestPose.x * 100).toFixed(1)} cm, y: ${(appState.latestPose.y * 100).toFixed(1)} cm, h: ${appState.latestPose.headingDeg.toFixed(0)}°`
    : "robot x: -- cm, y: -- cm, h: --°";

  const mousePart = appState.lastMouseWorld
    ? `mouse x: ${(appState.lastMouseWorld.x * 100).toFixed(1)} cm, y: ${(appState.lastMouseWorld.y * 100).toFixed(1)} cm`
    : "mouse x: -- cm, y: -- cm";

  const laneInfo = appState.latestPose ? getLaneRelativePose(appState.latestPose) : null;
  const lanePart = laneInfo
    ? `lane ${laneInfo.number} (${laneInfo.name}) x_rel: ${(laneInfo.xRel * 100).toFixed(1)} cm, y_rel: ${(laneInfo.yRel * 100).toFixed(1)} cm`
    : "lane: --";

  dom.cursorPosEl.innerHTML = `${robotPart}<br>${mousePart}<br>${lanePart}`;
}

setLog(log);
setCursorRefresh(refreshCursorReadout);

const renderer = createRenderer({ ctx: dom.ctx, worldToScreen, screenToWorld, log });
const signControls = createSignControls({ worldToScreen, screenToWorld, log });
const simulator = createSimulator({ log });
const wsTelemetry = createWsTelemetry({ log, simulator });
const race = createRace({ log, onUpdate: renderRaceUpdate, onFinish: renderRaceFinished });
const modeController = createModeController({
  dom,
  log,
  ensureSimulatorForPlay: simulator.ensureSimulatorForPlay,
  recenterOnRobot: () => recenterOnRobot(worldToScreen),
});
const serial = createSerialController({
  log,
  setStatus,
  baudInput: dom.baudInput,
  connectBtn: dom.connectBtn,
  disconnectBtn: dom.disconnectBtn,
});

function getPointerInfo(e) {
  const rect = e.target.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  return { px, py, world: screenToWorld(px, py) };
}

function isPointerOnRobot(px, py) {
  if (!appState.latestPose) return false;
  const center = worldToScreen(appState.latestPose.x, appState.latestPose.y);
  const halfW = (config.robotWidthMeters * appState.camera.scale) / 2;
  const halfL = (config.robotLengthMeters * appState.camera.scale) / 2;
  const radius = Math.hypot(halfL, halfW) * 1.1 + ROBOT_DRAG_HIT_PAD_PX;
  const dx = px - center.x;
  const dy = py - center.y;
  return dx * dx + dy * dy <= radius * radius;
}

function startRobotDrag(pointer) {
  if (!appState.latestPose) return;
  appState.robotDrag.active = true;
  appState.robotDrag.offsetX = appState.latestPose.x - pointer.world.x;
  appState.robotDrag.offsetY = appState.latestPose.y - pointer.world.y;
  appState.robotDrag.lastPx = pointer.px;
  appState.robotDrag.lastPy = pointer.py;
  appState.robotDrag.moved = false;
  appState.camera.dragging = false;
  appState.camera.dragMoved = true; // prevent click handler from re-placing on drop
}

function updateRobotDrag(pointer) {
  if (!appState.robotDrag.active || !appState.latestPose) return;
  const dxPx = pointer.px - appState.robotDrag.lastPx;
  const dyPx = pointer.py - appState.robotDrag.lastPy;
  if (!appState.robotDrag.moved && (Math.abs(dxPx) > 1 || Math.abs(dyPx) > 1)) {
    appState.robotDrag.moved = true;
  }
  const target = {
    x: pointer.world.x + appState.robotDrag.offsetX,
    y: pointer.world.y + appState.robotDrag.offsetY,
    headingDeg: appState.latestPose.headingDeg,
  };
  const collision = collidesWithAnything(target);
  if (collision) {
    noteCollisionOnce(`Cannot drag robot here: collides with ${collision}.`);
    return;
  }
  setPoseFromUser(target);
  appState.robotDrag.lastPx = pointer.px;
  appState.robotDrag.lastPy = pointer.py;
}

function endRobotDrag() {
  appState.robotDrag.active = false;
  appState.robotDrag.moved = false;
}

function updateRobotRotate(pointer) {
  if (!appState.robotRotate.active || !appState.latestPose) return;
  const dx = pointer.world.x - appState.latestPose.x;
  const dy = pointer.world.y - appState.latestPose.y;
  if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) return;
  const headingDeg = ((Math.atan2(dy, dx) * 180) / Math.PI + 360) % 360;
  setPoseFromUser({ ...appState.latestPose, headingDeg });
}

function startRobotRotate(pointer, event) {
  if (!appState.latestPose) return;
  appState.robotRotate.active = true;
  event?.preventDefault();
  updateRobotRotate(pointer); // snap heading on initial press
}

function endRobotRotate() {
  appState.robotRotate.active = false;
}

function updateSimButton() {
  dom.simToggleBtn.textContent = appState.simRunning ? "Stop Simulator" : "Start Simulator";
}

// --------------------------------------------------------------- race panel

function formatRaceTime(ms) {
  return `${(Math.max(0, ms) / 1000).toFixed(1)} s`;
}

// What the camera is reading right now, with the field position of each sign.
// The firmware only consumes colour + bearing; the position is here so a wrong
// reading can be spotted (red at 2,00/0,60 versus red at 1,00/0,60).
function renderSignReadout() {
  if (!dom.signReadout) return;
  const signs = appState.detectedSigns || [];
  if (!signs.length) {
    dom.signReadout.textContent = "Signs detected: —";
    return;
  }
  const parts = signs.map((s) => {
    const colour = s.state === 1 ? "green" : "red";
    const pos = `${(s.x * 100).toFixed(0)},${(s.y * 100).toFixed(0)}`;
    return `${colour} @ ${pos} cm (${(s.distance * 100).toFixed(0)} cm, ${s.angleDeg.toFixed(0)}°)`;
  });
  dom.signReadout.textContent = `Signs detected: ${parts.join("  ·  ")}`;
}

function renderRaceUpdate(u) {
  renderSignReadout();
  if (dom.racePhase) dom.racePhase.textContent = u.phase;
  if (dom.raceTime) dom.raceTime.textContent = formatRaceTime(u.elapsedMs);
  if (dom.raceLaps) dom.raceLaps.textContent = `${u.laps}/${config.race.laps}`;
  if (dom.raceSections) dom.raceSections.textContent = String(u.sectionsDone);
  if (dom.raceScore) dom.raceScore.textContent = u.wallMoved ? "0" : String(u.score);
  if (dom.raceDirection) {
    dom.raceDirection.textContent = u.direction
      ? u.direction === "ccw"
        ? "counter-clockwise"
        : "clockwise"
      : "—";
  }
  if (dom.raceSection) dom.raceSection.textContent = u.sectionLabel || "—";
  if (dom.raceGhost) {
    dom.raceGhost.textContent = Number.isFinite(u.ghostError)
      ? `${(u.ghostError * 100).toFixed(1)} cm`
      : "—";
  }
  if (dom.raceFlags) {
    const flags = [];
    if (u.startInParking) flags.push("start in parking");
    if (u.signsMoved) flags.push("signs moved");
    if (u.wallMoved) flags.push("wall moved → 0");
    if (u.parking) flags.push(`parking ${u.parking}`);
    if (u.timedOut) flags.push("time out");
    if (u.signWrongSide) flags.push("sign on the wrong side");
    dom.raceFlags.textContent = flags.join(" · ");
  }
}

function renderRaceFinished(result) {
  // Freeze the world where the car stopped: the firmware keeps driving after the
  // score is known, and the car used to carry on around the track (or leave the
  // bay) before anyone could look at it.
  appState.simFrozen = true;
  if (dom.replayBar) dom.replayBar.hidden = false;
  // The bar is on screen from the moment the round ends: its end has to be the
  // run's end already, or the first drag does nothing (min == max == 0).
  // Park the draw on the final recorded frame (the run's real end), not on
  // whatever the firmware did after the whistle.
  replayApi?.showLast?.();
  if (dom.raceBreakdown) {
    const lines = (result.breakdown || []).map(
      (b) => `${String(b.code).padEnd(6)} ${String(b.points).padStart(3)}  ${b.label}`
    );
    const total = `${"".padEnd(6)} ${String(result.score).padStart(3)}  TOTAL${
      result.wallMoved ? "  (wall moved: round zeroed)" : ""
    }`;
    const meta = `time ${(result.elapsedMs / 1000).toFixed(1)} s · laps ${result.laps} · ${
      result.distanceMeters ?? "—"
    } m · direction ${result.direction ?? "—"}`;
    dom.raceBreakdown.textContent = [meta, "", ...lines, "", total].join("\n");
  }
  notify(`Round finished: ${result.score} points`, {
    variant: result.score >= 40 ? "info" : "warning",
  });
}

function conventionOverrides() {
  const overrides = {};
  if (Number.isFinite(config.imuSignParam) && config.imuSignParam !== 0) {
    overrides.imuSign = config.imuSignParam;
  }
  if (Number.isFinite(config.servoSignParam) && config.servoSignParam !== 0) {
    overrides.servoSign = config.servoSignParam;
  }
  return overrides;
}

// Applies [{x_cm, y_cm, state}] to the field and clears every other slot. Shared
// by window.__race.field.set and the ?signs= URL parameter, so both do exactly
// the same thing.
function applyFieldSigns(spec) {
  for (const s of trafficSigns) s.state = 0;
  for (const item of spec || []) {
    const target = trafficSigns.find(
      (s) => Math.abs(s.x * 100 - item.x_cm) < 0.5 && Math.abs(s.y * 100 - item.y_cm) < 0.5,
    );
    if (target && (item.state === 1 || item.state === 2)) target.state = item.state;
  }
  return trafficSigns.filter((s) => s.state !== 0).length;
}

// ?signs= wins over the seeded layout. Called both when the page arms itself
// (the ?start= path) and on every button start, so a reload keeps the field.
function applyUrlField() {
  if (!config.fieldSigns?.length) return 0;
  const placed = applyFieldSigns(config.fieldSigns);
  log(`URL field: ${placed} signs.`);
  return placed;
}

function startRaceRun(newTrack) {
  if (newTrack) newSeed();
  else setSeed(appState.trackSeed ?? getSeed());
  appState.trackSeed = getSeed();
  randomizeSigns();
  applyUrlField();
  race.reset();
  // New bay side is derived from the seeded starting direction.
  wsTelemetry.resetOrigin();
  simulator.resetToParking();
  // Fresh interpreter: the firmware's globals have no clean reset.
  wsTelemetry.send({ type: "race", action: "reset", ...conventionOverrides() });
  race.begin(appState.latestPose);
  if (dom.raceSeed) dom.raceSeed.textContent = `seed ${appState.trackSeed}`;
  if (dom.raceBreakdown) dom.raceBreakdown.textContent = "";
}

function toggleNoise() {
  const on = !(config.telemetryNoise.enabled || appState.realWorldMode);
  config.telemetryNoise.enabled = on;
  appState.realWorldMode = on;
  if (dom.noiseBtn) {
    dom.noiseBtn.textContent = `Noise: ${on ? "On" : "Off"}`;
    dom.noiseBtn.setAttribute("aria-pressed", String(on));
  }
  log(on ? "Sensors with noise (competition mode)." : "Ideal sensors (training mode).");
}

function toggleSpeed() {
  // 1x -> 2x -> 4x -> 1x. The firmware clock is scaled by the same factor, so
  // every step is a faster time lapse of the same race, not a faster car.
  appState.simScale = appState.simScale === 1 ? 2 : appState.simScale === 2 ? 4 : 1;
  if (appState.simRunning) {
    appState.simDt = (1 / config.simFps) * appState.simScale;
  }
  wsTelemetry.setScale?.();
  simulator.applySimScale?.();
  if (dom.speedBtn) {
    dom.speedBtn.textContent = `${appState.simScale}×`;
    dom.speedBtn.setAttribute("aria-pressed", String(appState.simScale > 1));
  }
  log(`Simulation speed: ${appState.simScale}×.`);
}

function handleCanvasClick(e) {
  if (appState.camera.dragMoved) {
    appState.camera.dragMoved = false;
    return;
  }
  if (isPlayTestMode()) {
    signControls.handleTestLeftClick(e);
  } else {
    signControls.toggleSignAtClick(e);
  }
  appState.camera.dragMoved = false;
}

function handleCanvasRightClick(e) {
  e.preventDefault();
  if (!isPlayTestMode()) return;
  if (appState.robotRotate.active) return;
  signControls.handleTestRightClick(e);
}

function handleWheel(e) {
  e.preventDefault();
  zoomAt(e.offsetX, e.offsetY, e.deltaY, (px, py) => screenToWorld(px, py));
}

function handleMouseMove(e) {
  const pointer = getPointerInfo(e);
  setLastMouseWorld(pointer.world);
  refreshCursorReadout();
  if (appState.robotRotate.active) {
    updateRobotRotate(pointer);
    return;
  }
  if (appState.robotDrag.active) {
    updateRobotDrag(pointer);
    return;
  }
  if (!appState.camera.dragging) return;
  const dx = e.clientX - appState.camera.lastX;
  const dy = e.clientY - appState.camera.lastY;
  appState.camera.offsetX += dx;
  appState.camera.offsetY += dy;
  appState.camera.lastX = e.clientX;
  appState.camera.lastY = e.clientY;
  if (!appState.camera.dragMoved && (Math.abs(dx) > 2 || Math.abs(dy) > 2)) {
    appState.camera.dragMoved = true;
  }
}

function handleMouseDown(e) {
  if (e.button === 2) {
    if (isPlayTestMode()) {
      startRobotRotate(getPointerInfo(e), e);
    }
    return;
  }
  if (e.button !== 0) return;
  const pointer = getPointerInfo(e);
  if (isPlayTestMode() && isPointerOnRobot(pointer.px, pointer.py)) {
    startRobotDrag(pointer);
    return;
  }
  appState.camera.dragging = true;
  appState.camera.dragMoved = false;
  appState.camera.lastX = e.clientX;
  appState.camera.lastY = e.clientY;
}

function handleMouseUp(e) {
  if (e?.button === 2 && appState.robotRotate.active) {
    endRobotRotate();
  }
  if (appState.robotDrag.active) {
    endRobotDrag();
  }
  appState.camera.dragging = false;
}

function handleKey(e, down) {
  simulator.handleKey(e, down);
}

function loadDefaultBackground() {
  const img = new Image();
  img.onload = () => {
    appState.backgroundImage = img;
    appState.defaultBgLoaded = true;
    log("Loaded default background: arena.png");
  };
  img.onerror = () => {
    log("No default background found (arena.png).");
  };
  img.src = "arena.png";
}

function handleBgChange(e) {
  const file = e.target.files?.[0];
  if (!file) return;
  const img = new Image();
  img.onload = () => {
    appState.backgroundImage = img;
    log(`Loaded background image: ${file.name}`);
  };
  img.onerror = () => log("Failed to load image.");
  img.src = URL.createObjectURL(file);
}

function syncLogCollapse() {
  if (!dom.logPanel || !dom.logArea || !dom.logToggleBtn) return;
  dom.logPanel.classList.toggle("collapsed", logCollapsed);
  dom.logArea.hidden = logCollapsed;
  dom.logToggleBtn.textContent = logCollapsed ? "Expand" : "Collapse";
  dom.logToggleBtn.setAttribute("aria-pressed", String(logCollapsed));
}

function toggleLogPanel(e) {
  e?.stopPropagation();
  logCollapsed = !logCollapsed;
  syncLogCollapse();
}

appState.onSimStep = (simNow) => wsTelemetry.pump?.(simNow);

let replayApi = null; // filled in by wireEvents(); window.__race is created elsewhere

function wireEvents() {
  dom.connectBtn.addEventListener("click", serial.connectSerial);
  dom.disconnectBtn.addEventListener("click", serial.disconnectSerial);
  dom.simToggleBtn.addEventListener("click", () => {
    simulator.toggleSimulator();
    updateSimButton();
  });
  dom.randomizeSignsBtn.addEventListener("click", randomizeSigns);
  dom.surpriseRuleBtn.addEventListener("click", () => {
    appState.surpriseRule = !appState.surpriseRule;
    dom.surpriseRuleBtn.setAttribute("aria-pressed", String(appState.surpriseRule));
    dom.surpriseRuleBtn.textContent = `Surprise Rule: ${appState.surpriseRule ? "On" : "Off"}`;
    randomizeSigns();
  });
  dom.modeTelemetryBtn.addEventListener("click", () => {
    modeController.setMode(Modes.TELEMETRY);
    updateSimButton();
  });
  dom.modePlayBtn.addEventListener("click", () => {
    modeController.setMode(Modes.PLAY);
    updateSimButton();
  });
  dom.followHeadingBtn.addEventListener("click", modeController.toggleFollowHeading);
  dom.debugDistancesBtn.addEventListener("click", modeController.toggleShowDistances);
  dom.signHudBtn.addEventListener("click", modeController.toggleShowSignHud);
  dom.realWorldBtn.addEventListener("click", modeController.toggleRealWorldMode);
  // ---- replay -----------------------------------------------------------
  // The run is recorded frame by frame by the physics (appState.replayFrames).
  // Scrubbing only changes what is drawn: the world is frozen while in replay
  // and nothing is sent to the firmware, so the recorded run cannot be altered.
  const replay = {
    index: 0,
    playing: false,
    speed: 1,
    raf: null,
    clockMs: 0,
    lastWall: 0,
  };

  function replayFrames() {
    return appState.replayFrames || [];
  }

  // The scrubber's end is the run's end. It used to be written once, when the
  // replay panel was opened, while the recorder was still appending frames: the
  // bar then ended in the middle of the run and the thumb could be dragged past
  // the last recorded frame. Keep it in step every time we draw.
  function syncReplaySlider() {
    const frames = replayFrames();
    if (!dom.replaySlider) return frames.length - 1;
    const last = Math.max(0, frames.length - 1);
    if (dom.replaySlider.max !== String(last)) dom.replaySlider.max = String(last);
    return last;
  }

  function replayLabel(f, i, last) {
    if (!dom.replayTime) return;
    const total = (replayFrames()[last]?.t ?? f.t) / 1000;
    dom.replayTime.textContent =
      `${(f.t / 1000).toFixed(1)} s / ${total.toFixed(1)} s · frame ${i}/${last}`;
  }

  function replayShow(index) {
    const frames = replayFrames();
    if (!frames.length) return;
    const last = syncReplaySlider();
    const i = Math.max(0, Math.min(last, Math.round(index)));
    replay.index = i;
    const f = frames[i];
    appState.latestPose = { x: f.x, y: f.y, headingDeg: f.h };
    appState.robotSpeed = f.v;
    if (dom.replaySlider && dom.replaySlider.value !== String(i)) {
      dom.replaySlider.value = String(i);
    }
    replayLabel(f, i, last);
  }

  function replayStop() {
    replay.playing = false;
    if (replay.raf !== null) { cancelAnimationFrame(replay.raf); replay.raf = null; }
    if (dom.replayPlay) dom.replayPlay.textContent = "▶";
  }

  // Playback walks the recorded frames by their own timestamps instead of
  // stepping one frame per browser tick. The old fixed 60 Hz stepping could
  // only ever play at 1x, ignored the real spacing of the frames, and never
  // stopped cleanly because it re-read the (still growing) frame array.
  function replayPlay() {
    const frames = replayFrames();
    if (!frames.length) return;
    replayStop();
    if (replay.index >= frames.length - 1) replay.index = 0; // replay from the top
    appState.replayMode = true;
    replay.playing = true;
    replay.clockMs = frames[replay.index].t;
    replay.lastWall = performance.now();
    if (dom.replayPlay) dom.replayPlay.textContent = "⏸";

    const step = (now) => {
      if (!replay.playing) return;
      const dtWall = Math.max(0, now - replay.lastWall);
      replay.lastWall = now;
      replay.clockMs += dtWall * replay.speed;
      const frames = replayFrames();
      const last = frames.length - 1;
      let i = replay.index;
      while (i < last && frames[i + 1].t <= replay.clockMs) i++;
      if (i !== replay.index) replayShow(i);
      if (i >= last) {
        // Freeze on the final frame, exactly where the round ended.
        replay.clockMs = frames[last].t;
        replayStop();
        return;
      }
      replay.raf = requestAnimationFrame(step);
    };
    replay.raf = requestAnimationFrame(step);
  }

  function replayToggle() {
    if (replay.playing) { replayStop(); return; }
    replayPlay();
  }

  function replayEnter() {
    const frames = replayFrames();
    if (!frames.length) { log("Nothing recorded in this run."); return; }
    appState.replayMode = true;
    if (dom.replayBar) dom.replayBar.hidden = false;
    syncReplaySlider();
    replayShow(0);
  }

  function replayExit() {
    replayStop();
    appState.replayMode = false;
  }

  // ---- exportar ---------------------------------------------------------
  // One JSON per round with everything needed to argue about it later: the
  // result and its breakdown, the configuration the round was driven with, the
  // firmware revision (from the log line the controller prints at boot), the
  // telemetry log, the signs that were on the field, and the recorded trajectory.
  function raceExport(format) {
    const result = race.result() || {};
    const state = race.getState() || {};
    const logText = dom.logArea ? dom.logArea.textContent : "";
    const revision = (logText.match(/revision ([0-9a-f]{7,40})/) || [])[1] || null;
    const frames = replayFrames().map((f) => ({
      t_ms: Math.round(f.t),
      x_cm: +(f.x * 100).toFixed(2),
      y_cm: +(f.y * 100).toFixed(2),
      heading_deg: +f.h.toFixed(2),
      speed_cm_s: +(f.v * 100).toFixed(2),
    }));

    if (format === "csv") {
      const head = "t_ms,x_cm,y_cm,heading_deg,speed_cm_s";
      const body = frames.map((f) => [f.t_ms, f.x_cm, f.y_cm, f.heading_deg, f.speed_cm_s].join(","));
      downloadFile(`round-${config.seed ?? "seed"}-${stamp()}.csv`, [head, ...body].join("\n"), "text/csv");
      return;
    }

    const payload = {
      exportedAt: new Date().toISOString(),
      sim: {
        seed: config.seed ?? null,
        direction: result.direction ?? config.startDirection ?? null,
        startMode: config.startMode,
        speedScale: appState.simScale,
        noise: Boolean(config.telemetryNoise?.enabled),
        firmwareRevision: revision,
        url: window.location.href,
      },
      result,
      // What the camera was seeing at the instant of the export - normally
      // nothing, because the round has ended. Kept for compatibility; the two
      // lists below are the ones that describe the round.
      signs: (detectedSignsSnapshot() || []).map((s) => ({
        lane: s.lane,
        colour: s.state === 1 ? "green" : "red",
        x_cm: +(s.x * 100).toFixed(1),
        y_cm: +(s.y * 100).toFixed(1),
        distance_cm: +(s.distance * 100).toFixed(1),
        angle_deg: +s.angleDeg.toFixed(1),
      })),
      // Every time a sign entered the camera's field of view, with the car pose
      // and the reading - this is what the firmware was actually fed.
      signSightings: (appState.signSightings || []).slice(),
      // Every sign driven past, right side or wrong side. This is the list that
      // says whether the car respected the signs.
      signPassages: (appState.signPassages || []).slice(),
      // Contacts with a sign. This is the part that costs points (1.4 / 1.6
      // instead of 1.5 / 1.7); which side the sign was passed on is not scored.
      signHits: (appState.signHits || []).slice(),
      // Trajectory samples after this instant are past the end of the round (the
      // car keeps running until the export is taken).
      raceEndedAtMs: state.phase === "finished" ? Math.round(result.elapsedMs || 0) : null,
      // Ground truth of the layout, so an exported round can be judged months
      // later without re-seeding the same track.
      fieldSigns: (trafficSigns || [])
        .filter((s) => s.state !== 0)
        .map((s) => ({
          lane: s.lane,
          colour: s.state === 1 ? "green" : "red",
          x_cm: +(s.x * 100).toFixed(1),
          y_cm: +(s.y * 100).toFixed(1),
        })),
      // Firmware odometry against the world truth, every 200 ms of simulated
      // time: `err_cm` is the drift, `heading_err_deg` its rotation part.
      odometry: (appState.odometrySamples || []).slice(),
      trajectory: frames,
      log: logText,
    };
    downloadFile(`round-${config.seed ?? "seed"}-${stamp()}.json`, JSON.stringify(payload, null, 2), "application/json");
  }

  function detectedSignsSnapshot() {
    return (appState.detectedSigns || []).slice();
  }

  function stamp() {
    const d = new Date();
    const p = (n) => String(n).padStart(2, "0");
    return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
  }

  function downloadFile(name, text, mime) {
    const blob = new Blob([text], { type: `${mime};charset=utf-8` });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
    log(`Round exported: ${name}`);
  }

  dom.exportJsonBtn?.addEventListener("click", () => raceExport("json"));
  dom.exportCsvBtn?.addEventListener("click", () => raceExport("csv"));

  dom.replayEnter?.addEventListener("click", replayEnter);
  dom.replayPlay?.addEventListener("click", replayToggle);
  dom.replayBack?.addEventListener("click", () => { appState.replayMode = true; replayShow(replay.index - 1); });
  dom.replayFwd?.addEventListener("click", () => { appState.replayMode = true; replayShow(replay.index + 1); });
  dom.replaySpeed?.addEventListener("change", (event) => {
    replay.speed = Number(event.target.value) || 1;
  });
  replayApi = {
    enter: replayEnter,
    exit: replayExit,
    show: replayShow,
    toggle: replayToggle,
    play: replayPlay,
    stop: replayStop,
    sync: syncReplaySlider,
    speed: (value) => { replay.speed = Number(value) || 1; },
    showLast: () => {
      const last = syncReplaySlider();
      if (last >= 0) replayShow(last);
    },
  };

  dom.replaySlider?.addEventListener("input", (event) => {
    appState.replayMode = true;
    replayShow(Number(event.target.value));
  });

  dom.newTrackBtn?.addEventListener("click", () => { replayExit(); startRaceRun(true); });
  dom.repeatTrackBtn?.addEventListener("click", () => { replayExit(); startRaceRun(false); });
  dom.noiseBtn?.addEventListener("click", toggleNoise);
  dom.speedBtn?.addEventListener("click", toggleSpeed);
  dom.clearLogBtn.addEventListener("click", () => (dom.logArea.textContent = ""));
  dom.logToggleBtn?.addEventListener("click", toggleLogPanel);
  dom.bgInput?.addEventListener("change", handleBgChange);

  dom.canvas.addEventListener("mousedown", handleMouseDown);
  dom.canvas.addEventListener("mousemove", handleMouseMove);
  dom.canvas.addEventListener("mouseup", handleMouseUp);
  dom.canvas.addEventListener("wheel", handleWheel, { passive: false });
  dom.canvas.addEventListener("mouseleave", () => {
    setLastMouseWorld(null);
    refreshCursorReadout();
  });
  dom.canvas.addEventListener("click", handleCanvasClick);
  dom.canvas.addEventListener("contextmenu", handleCanvasRightClick);

  window.addEventListener("resize", () => resizeCanvas(dom));
  window.addEventListener("keydown", (e) => handleKey(e, true));
  window.addEventListener("keyup", (e) => handleKey(e, false));
}

function initLayout() {
  resetConfig();
  resizeCanvas(dom);
  centerView();
  recenterArenaView();
  modeController.initModeDefaults();
  updateSimButton();
  setStatus("Idle", "#1f2937");
  refreshCursorReadout();
  syncLogCollapse();
}

function init() {
  wireEvents();
  initLayout();
  modeController.setMode(Modes.TELEMETRY, { force: true });
  updateSimButton();
  loadDefaultBackground();
  if (Number.isFinite(config.seed)) setSeed(config.seed);
  appState.trackSeed = getSeed();
  randomizeSigns();
  applyUrlField();
  renderer.renderFrame();
  wsTelemetry.start();
  window.addEventListener("beforeunload", () => wsTelemetry.stop());
  if (dom.raceSeed) dom.raceSeed.textContent = `seed ${appState.trackSeed}`;
  if (dom.noiseBtn) {
    const on = Boolean(config.telemetryNoise.enabled);
    dom.noiseBtn.textContent = `Noise: ${on ? "On" : "Off"}`;
    dom.noiseBtn.setAttribute("aria-pressed", String(on));
    // Scored runs use the real sensor model too: a ±15° cone instead of a single
    // ray, otherwise the ultrasonic stream is more optimistic than the hardware.
    appState.realWorldMode = on;
  }
  // Arm the round as soon as the world has a pose (the websocket resets the car
  // into the parking bay on connect, and the firmware boots with it).
  // ?speed=4 opens the page already at 4x (headless runs need it: the button is a
  // click away for a human, not for race_run).
  if (config.initialSimScale && config.initialSimScale > 1) {
    appState.simScale = config.initialSimScale;
    if (dom.speedBtn) dom.speedBtn.textContent = `${appState.simScale}×`;
    simulator.applySimScale();
    wsTelemetry.setScale();
  }

  const armTimer = setInterval(() => {
    if (appState.latestPose && race.getState().phase === "idle") {
      // The world has to be running while the car waits in the bay. The firmware
      // slaves its clock to the simulated one, and its first action is a one
      // second sleep: with the world stopped until the first command, that sleep
      // waited for a clock that only moves once the car moves - a deadlock that
      // ended in the shim's stall bail-out and a round that never started.
      simulator.ensureSimulatorForPlay();
      race.begin(appState.latestPose);
      clearInterval(armTimer);
    }
  }, 200);
  // Headless / console API: used by simulator/race_run.py and handy for poking
  // the world from devtools.
  window.__race = {
    replay: {
      enter: () => replayApi?.enter(),
      exit: () => replayApi?.exit(),
      show: (i) => replayApi?.show(i),
      toggle: () => replayApi?.toggle(),
    },
    getState: () => race.getState(),
    getResult: () => race.result(),
    start: (newTrack = true) => startRaceRun(newTrack),
    toggleNoise,
    toggleSpeed,
    appState,
    config,
    // Exact sign layout, for the sweep over every sign hypothesis. `set` takes
    // [{x_cm, y_cm, state}] with state 1=green 2=red; everything not listed is
    // cleared. Used by tools/sign_sweep.py, never by the product.
    // Diagnostic hook: what the emulated sonars see right now, per label.
    ranges: () => measurePoseDistances(appState.latestPose).map((m) => ({
      label: m.label, distance_cm: +(m.distance * 100).toFixed(1),
    })),
    field: {
      read: () => trafficSigns.map((s) => ({
        lane: s.lane,
        x_cm: +(s.x * 100).toFixed(1),
        y_cm: +(s.y * 100).toFixed(1),
        state: s.state,
      })),
      set: (spec) => applyFieldSigns(spec),
    },
  };
  log("Ready. Click Connect to open a serial port.");
}

init();

