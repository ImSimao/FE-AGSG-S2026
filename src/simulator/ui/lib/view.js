import { config } from "./config.js";
import { appState } from "./appState.js";
import { clamp } from "./math.js";

export function resizeCanvas(dom) {
  const dpr = window.devicePixelRatio || 1;
  appState.viewWidth = dom.canvas.clientWidth;
  appState.viewHeight = dom.canvas.clientHeight;
  dom.canvas.width = appState.viewWidth * dpr;
  dom.canvas.height = appState.viewHeight * dpr;
  dom.ctx.setTransform(1, 0, 0, 1, 0, 0);
  dom.ctx.scale(dpr, dpr);
  centerView();
}

export function centerView() {
  const { camera } = appState;
  const fitScale = 0.9 * (appState.viewHeight / config.arenaHeightMeters);
  camera.scale = Math.max(20, fitScale);
  if (config.origin === "center") {
    camera.offsetX = -(config.arenaWidthMeters * camera.scale) / 2;
    camera.offsetY = (config.arenaHeightMeters * camera.scale) / 2;
  } else if (config.origin === "bottom-left") {
    const arenaPxW = config.arenaWidthMeters * camera.scale;
    const arenaPxH = config.arenaHeightMeters * camera.scale;
    camera.offsetX = (appState.viewWidth - arenaPxW) / 2;
    camera.offsetY = (arenaPxH - appState.viewHeight) / 2;
  } else {
    const arenaPxW = config.arenaWidthMeters * camera.scale;
    const arenaPxH = config.arenaHeightMeters * camera.scale;
    camera.offsetX = (appState.viewWidth - arenaPxW) / 2;
    camera.offsetY = (appState.viewHeight - arenaPxH) / 2;
  }
}

export function recenterArenaView() {
  const { camera } = appState;
  if (config.origin === "center") {
    camera.offsetX = -(config.arenaWidthMeters * camera.scale) / 2;
    camera.offsetY = (config.arenaHeightMeters * camera.scale) / 2;
  } else if (config.origin === "bottom-left") {
    const arenaPxW = config.arenaWidthMeters * camera.scale;
    const arenaPxH = config.arenaHeightMeters * camera.scale;
    camera.offsetX = (appState.viewWidth - arenaPxW) / 2;
    camera.offsetY = (arenaPxH - appState.viewHeight) / 2;
  } else {
    const arenaPxW = config.arenaWidthMeters * camera.scale;
    const arenaPxH = config.arenaHeightMeters * camera.scale;
    camera.offsetX = (appState.viewWidth - arenaPxW) / 2;
    camera.offsetY = (appState.viewHeight - arenaPxH) / 2;
  }
}

export function recenterOnRobot(worldToScreen) {
  if (!appState.latestPose) return;
  const pos = worldToScreen(appState.latestPose.x, appState.latestPose.y);
  appState.camera.offsetX += appState.viewWidth / 2 - pos.x;
  appState.camera.offsetY += appState.viewHeight / 2 - pos.y;
}

export function worldToScreen(x, y) {
  const { camera } = appState;
  let px = x * camera.scale;
  let py = y * camera.scale;

  if (config.origin === "bottom-left") {
    py = appState.viewHeight - py;
  } else if (config.origin === "center") {
    px = appState.viewWidth / 2 + px;
    py = appState.viewHeight / 2 - py;
  }
  px += camera.offsetX;
  py += camera.offsetY;
  return { x: px, y: py };
}

export function screenToWorld(px, py) {
  const { camera } = appState;
  let sx = px;
  let sy = py;

  if (appState.followHeading && appState.followRotationRad !== 0) {
    const cx = appState.viewWidth / 2;
    const cy = appState.viewHeight / 2;
    const dx = px - cx;
    const dy = py - cy;
    const cos = Math.cos(-appState.followRotationRad);
    const sin = Math.sin(-appState.followRotationRad);
    sx = cx + dx * cos - dy * sin;
    sy = cy + dx * sin + dy * cos;
  }

  let x = sx - camera.offsetX;
  let y = sy - camera.offsetY;

  if (config.origin === "bottom-left") {
    y = appState.viewHeight - y;
  } else if (config.origin === "center") {
    x = x - appState.viewWidth / 2;
    y = appState.viewHeight / 2 - y;
  }

  x /= camera.scale;
  y /= camera.scale;
  return { x, y };
}

export function updateCursorPos(e, worldCb) {
  const rect = e.target.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  appState.lastMouseWorld = worldCb(px, py);
}

export function zoomAt(px, py, deltaY, worldCb) {
  const { camera } = appState;
  const zoomFactor = deltaY < 0 ? 1.1 : 0.9;
  const oldScale = camera.scale;
  const newScale = clamp(oldScale * zoomFactor, 20, 1000);
  if (newScale === oldScale) return;

  const worldBefore = worldCb(px, py);
  camera.scale = newScale;
  const screenAfter = worldToScreen(worldBefore.x, worldBefore.y);
  camera.offsetX += px - screenAfter.x;
  camera.offsetY += py - screenAfter.y;
}



