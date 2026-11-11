import { config } from "./config.js";
import { appState } from "./appState.js";
import { trafficSigns } from "./traffic.js";
import { bodyCentre, getParkingSpot, getRobotHalfDims, getSignHalfSize, getSensorHalfFovRad } from "./geometry.js";
import { rotateDir, degToRad } from "./math.js";
import { detectNearestTrafficSignInFov, formatSignedDeg, getDistanceMeasurements } from "./pose.js";
import { recenterOnRobot } from "./view.js";

function getSignColors(state) {
  switch (state) {
    case 1:
      return { fill: "#22c55e", stroke: "#0f172a" };
    case 2:
      return { fill: "#ef4444", stroke: "#0f172a" };
    default:
      return { fill: "rgba(0,0,0,0)", stroke: "#64748b" };
  }
}

export function createRenderer({ ctx, worldToScreen, screenToWorld, log }) {
  function drawGrid() {
    const stepPx = config.gridStepMeters * appState.camera.scale;
    if (stepPx < 15) return;

    ctx.save();
    ctx.globalCompositeOperation = "lighter"; // force ghost to stand out over robot
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(148, 163, 184, 0.2)";

    const topLeft = screenToWorld(0, 0);
    const bottomRight = screenToWorld(appState.viewWidth, appState.viewHeight);
    const startX = Math.floor(topLeft.x / config.gridStepMeters) * config.gridStepMeters;
    const endX = Math.ceil(bottomRight.x / config.gridStepMeters) * config.gridStepMeters;
    const startY = Math.floor(bottomRight.y / config.gridStepMeters) * config.gridStepMeters;
    const endY = Math.ceil(topLeft.y / config.gridStepMeters) * config.gridStepMeters;

    for (let x = startX; x <= endX; x += config.gridStepMeters) {
      const p1 = worldToScreen(x, startY);
      const p2 = worldToScreen(x, endY);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    for (let y = startY; y <= endY; y += config.gridStepMeters) {
      const p1 = worldToScreen(startX, y);
      const p2 = worldToScreen(endX, y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawArenaBounds() {
    ctx.save();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#ff0000";
    const corners = [
      worldToScreen(0, 0),
      worldToScreen(config.arenaWidthMeters, 0),
      worldToScreen(config.arenaWidthMeters, config.arenaHeightMeters),
      worldToScreen(0, config.arenaHeightMeters),
    ];
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    for (let i = 1; i < corners.length; i++) {
      ctx.lineTo(corners[i].x, corners[i].y);
    }
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  function drawInternalWalls() {
    const size = config.internalSquareSizeMeters;
    if (!size || size <= 0) return;
    ctx.save();
    ctx.lineWidth = Math.max(1, config.wallThicknessMeters * appState.camera.scale);
    ctx.strokeStyle = "#ff0000";

    const centerX = config.arenaWidthMeters / 2;
    const centerY = config.arenaHeightMeters / 2;
    const half = size / 2;

    const corners = [
      worldToScreen(centerX - half, centerY - half),
      worldToScreen(centerX + half, centerY - half),
      worldToScreen(centerX + half, centerY + half),
      worldToScreen(centerX - half, centerY + half),
    ];
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    for (let i = 1; i < corners.length; i++) {
      ctx.lineTo(corners[i].x, corners[i].y);
    }
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  function drawParkingSpot() {
    const spot = getParkingSpot();
    if (!spot) return;
    const { leftX, rightX, y0, y1 } = spot;
    const t = spot.wallThickness || config.wallThicknessMeters;
    ctx.save();

    // The two magenta elements at their real size (200 x 20 x 100 mm, rule
    // 13.25), drawn *outside* the free rectangle: the gap you see between them
    // is the gap the robot gets.
    ctx.fillStyle = "#ff00ff";
    for (const [x0, x1] of [
      [leftX - t, leftX],
      [rightX, rightX + t],
    ]) {
      const a = worldToScreen(x0, y1);
      const b = worldToScreen(x1, y0);
      ctx.fillRect(a.x, a.y, b.x - a.x, b.y - a.y);
    }

    // Free rectangle: this is the grey area the robot must sit fully inside.
    ctx.setLineDash([4, 3]);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
    ctx.lineWidth = 1;
    const corner = worldToScreen(leftX, y1);
    const opposite = worldToScreen(rightX, y0);
    ctx.strokeRect(corner.x, corner.y, opposite.x - corner.x, opposite.y - corner.y);
    ctx.setLineDash([]);

    // The gap in cm, so it can be checked against a tape measure.
    const gapCm = ((rightX - leftX) * 100).toFixed(2).replace(".", ",");
    const middle = worldToScreen((leftX + rightX) / 2, (y0 + y1) / 2);
    ctx.fillStyle = "#ffffff";
    ctx.font = "11px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(`${gapCm} cm`, middle.x, middle.y);
    ctx.restore();
  }

  function drawTrafficSigns() {
    const size = config.signSizeMeters * appState.camera.scale;
    const half = size / 2;
    const circleR = ((config.signSeatCircleDiameterMeters || 0.085) / 2) * appState.camera.scale;
    ctx.save();
    ctx.lineWidth = Math.max(1, (config.wallThicknessMeters * appState.camera.scale) / 2);
    for (const sign of trafficSigns) {
      const { fill, stroke } = getSignColors(sign.state);
      // The seat circle: the sign is only "moved" once its projection leaves
      // this (rule 13.14/13.15).
      if (sign.state !== 0) {
        const c = worldToScreen(sign.x, sign.y);
        ctx.beginPath();
        ctx.arc(c.x, c.y, circleR, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.lineWidth = Math.max(1, (config.wallThicknessMeters * appState.camera.scale) / 2);
      }
      ctx.fillStyle = fill;
      ctx.strokeStyle = stroke;
      const p = worldToScreen(sign.x + (sign.dx || 0), sign.y + (sign.dy || 0));
      ctx.beginPath();
      ctx.rect(p.x - half, p.y - half, size, size);
      ctx.fill();
      ctx.stroke();
      if (sign.modified) {
        // Surprise Rule: the sign keeps its ORIGINAL colour but is flagged with
        // a dashed amber ring so the operator can see which one is "inverted".
        ctx.save();
        ctx.strokeStyle = "#f59e0b";
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 3]);
        ctx.beginPath();
        ctx.arc(p.x, p.y, half + 6, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }
    }
    ctx.restore();
  }

  function drawSonarFov(measurements) {
    if (!appState.realWorldMode || !measurements.length) return;
    const half = getSensorHalfFovRad();
    ctx.save();
    ctx.fillStyle = "rgba(59, 130, 246, 0.12)";
    ctx.strokeStyle = "rgba(59, 130, 246, 0.35)";
    ctx.lineWidth = 1;
    for (const m of measurements) {
      if (!m || m.distance <= 0) continue;
      const dir = m.baseDir || m.dir || { x: 0, y: 0 };
      const start = worldToScreen(m.start.x, m.start.y);
      const leftDir = rotateDir(dir, -half);
      const rightDir = rotateDir(dir, half);
      const left = worldToScreen(
        m.start.x + leftDir.x * m.distance,
        m.start.y + leftDir.y * m.distance
      );
      const right = worldToScreen(
        m.start.x + rightDir.x * m.distance,
        m.start.y + rightDir.y * m.distance
      );
      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(left.x, left.y);
      ctx.lineTo(right.x, right.y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawDistanceMeasurements() {
    if (!appState.latestPose) return;
    const measurements = getDistanceMeasurements();
    if (!measurements.length) return;

    ctx.save();
    drawSonarFov(measurements);
    ctx.setLineDash([6, 6]);
    ctx.lineWidth = 2;
    ctx.strokeStyle = "rgba(59, 130, 246, 0.9)";
    ctx.font = "12px sans-serif";
    ctx.textBaseline = "middle";
    ctx.textAlign = "left";

    for (const m of measurements) {
      const start = worldToScreen(m.start.x, m.start.y);
      const end = worldToScreen(m.end.x, m.end.y);
      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(end.x, end.y);
      ctx.stroke();

      const label = `${m.label}: ${(m.distance * 100).toFixed(0)} cm`;
      const midX = start.x + (end.x - start.x) * 0.6;
      const midY = start.y + (end.y - start.y) * 0.6;
      const pad = 3;
      const textWidth = ctx.measureText(label).width;
      ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
      ctx.fillRect(midX - pad, midY - 8, textWidth + pad * 2, 16);
      ctx.fillStyle = "#0f172a";
      ctx.fillText(label, midX, midY);
      ctx.strokeStyle = "rgba(59, 130, 246, 0.9)";
    }

    ctx.restore();
  }

  function drawSignDetection(detection) {
    if (!appState.latestPose) return;

    const headingRad = degToRad(appState.latestPose.headingDeg);
    const forward = { x: Math.cos(headingRad), y: Math.sin(headingRad) };
    const { halfL } = getRobotHalfDims();
    const halfFov = degToRad(config.signVisionHalfFovDeg || 60);
    const origin = detection?.origin || {
      x: appState.latestPose.x + forward.x * halfL,
      y: appState.latestPose.y + forward.y * halfL,
    };

    const fovRangeMeters = 1.2;
    const originPx = worldToScreen(origin.x, origin.y);
    const leftDir = rotateDir(forward, halfFov);
    const rightDir = rotateDir(forward, -halfFov);
    const left = worldToScreen(origin.x + leftDir.x * fovRangeMeters, origin.y + leftDir.y * fovRangeMeters);
    const right = worldToScreen(origin.x + rightDir.x * fovRangeMeters, origin.y + rightDir.y * fovRangeMeters);

    ctx.save();
    ctx.fillStyle = "rgba(234, 179, 8, 0.10)";
    ctx.strokeStyle = "rgba(234, 179, 8, 0.35)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(originPx.x, originPx.y);
    ctx.lineTo(left.x, left.y);
    ctx.lineTo(right.x, right.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    if (detection) {
      const signPos = worldToScreen(detection.sign.x, detection.sign.y);
      const colors = getSignColors(detection.sign.state);
      ctx.strokeStyle = colors.fill;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(originPx.x, originPx.y);
      ctx.lineTo(signPos.x, signPos.y);
      ctx.stroke();
    }

    const labelPos = worldToScreen(
      origin.x + forward.x * (halfL + 0.12),
      origin.y + forward.y * (halfL + 0.12)
    );
    const label = detection
      ? `${detection.sign.state === 1 ? "Green" : "Red"} · ${(detection.distance * 100).toFixed(0)} cm · ${formatSignedDeg(detection.angleDeg)}`
      : "No sign in 120°";
    const pad = 6;
    ctx.font = "12px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const w = ctx.measureText(label).width + pad * 2;
    const h = 20;

    ctx.fillStyle = "rgba(15, 23, 42, 0.9)";
    ctx.strokeStyle = "rgba(148, 163, 184, 0.35)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(labelPos.x - w / 2, labelPos.y - h / 2, w, h);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#e2e8f0";
    ctx.fillText(label, labelPos.x, labelPos.y);

    ctx.restore();
  }

  function drawBackground() {
    if (!appState.backgroundImage) return;
    const topLeft = worldToScreen(0, config.arenaHeightMeters);
    const size = {
      w: config.arenaWidthMeters * appState.camera.scale,
      h: config.arenaHeightMeters * appState.camera.scale,
    };
    ctx.save();
    ctx.globalAlpha = 0.85;
    ctx.drawImage(appState.backgroundImage, topLeft.x, topLeft.y, size.w, size.h);
    ctx.restore();
  }

  function drawRobot() {
    if (!appState.latestPose) return;
    const { headingDeg } = appState.latestPose;
    const centre = bodyCentre(appState.latestPose);
    const pos = worldToScreen(centre.x, centre.y);
    const halfW = (config.robotWidthMeters * appState.camera.scale) / 2;
    const body = config.bodyLengthMeters || config.robotLengthMeters;
    const halfL = (body * appState.camera.scale) / 2;

    ctx.save();
    ctx.translate(pos.x, pos.y);
    ctx.rotate((-headingDeg * Math.PI) / 180);

    const cornerRadius = Math.min(halfL, halfW) * 0.35;
    ctx.fillStyle = "#22c55e";
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-halfL + cornerRadius, -halfW);
    ctx.lineTo(halfL - cornerRadius, -halfW);
    ctx.arcTo(halfL, -halfW, halfL, -halfW + cornerRadius, cornerRadius);
    ctx.lineTo(halfL, halfW - cornerRadius);
    ctx.arcTo(halfL, halfW, halfL - cornerRadius, halfW, cornerRadius);
    ctx.lineTo(-halfL + cornerRadius, halfW);
    ctx.arcTo(-halfL, halfW, -halfL, halfW - cornerRadius, cornerRadius);
    ctx.lineTo(-halfL, -halfW + cornerRadius);
    ctx.arcTo(-halfL, -halfW, -halfL + cornerRadius, -halfW, cornerRadius);
    ctx.fill();
    ctx.stroke();

    // The nose stick, drawn to scale: 33 x 20 mm of the 160 mm measured length.
    const noseL = (config.noseLengthMeters || 0) * appState.camera.scale;
    if (noseL > 0) {
      const noseW = Math.max(2, (config.noseWidthMeters || 0.02) * appState.camera.scale);
      ctx.fillStyle = "#4ade80";
      ctx.fillRect(halfL, -noseW / 2, noseL, noseW);
      ctx.strokeRect(halfL, -noseW / 2, noseL, noseW);
    }

    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(halfL * 1.1, 0);
    ctx.stroke();

    ctx.restore();
  }

  function drawGhostRobot() {
    if (!appState.ghostPose) return;
    const { x, y, headingDeg } = appState.ghostPose;
    const pos = worldToScreen(x, y);
    const halfW = (config.robotWidthMeters * appState.camera.scale) / 2;
    const body = config.bodyLengthMeters || config.robotLengthMeters;
    const halfL = (body * appState.camera.scale) / 2;

    ctx.save();
    ctx.translate(pos.x, pos.y);
    ctx.rotate((-headingDeg * Math.PI) / 180);

    const cornerRadius = Math.min(halfL, halfW) * 0.35;
    ctx.strokeStyle = "rgba(59, 130, 246, 0.98)";
    ctx.lineWidth = 3;
    ctx.setLineDash([8, 5]);
    ctx.shadowColor = "rgba(59, 130, 246, 0.6)";
    ctx.shadowBlur = 6;
    ctx.beginPath();
    ctx.moveTo(-halfL + cornerRadius, -halfW);
    ctx.lineTo(halfL - cornerRadius, -halfW);
    ctx.arcTo(halfL, -halfW, halfL, -halfW + cornerRadius, cornerRadius);
    ctx.lineTo(halfL, halfW - cornerRadius);
    ctx.arcTo(halfL, halfW, halfL - cornerRadius, halfW, cornerRadius);
    ctx.lineTo(-halfL + cornerRadius, halfW);
    ctx.arcTo(-halfL, halfW, -halfL, halfW - cornerRadius, cornerRadius);
    ctx.lineTo(-halfL, -halfW + cornerRadius);
    ctx.arcTo(-halfL, -halfW, -halfL + cornerRadius, -halfW, cornerRadius);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(halfL * 1.1, 0);
    ctx.stroke();

    ctx.restore();
    ctx.setLineDash([]);
  }

  function applyFollowTransform() {
    if (!appState.followHeading || !appState.latestPose) {
      appState.followRotationRad = 0;
      return null;
    }
    recenterOnRobot(worldToScreen);
    appState.followRotationRad = ((appState.latestPose.headingDeg - 90) * Math.PI) / 180;
    ctx.save();
    ctx.translate(appState.viewWidth / 2, appState.viewHeight / 2);
    ctx.rotate(appState.followRotationRad);
    ctx.translate(-appState.viewWidth / 2, -appState.viewHeight / 2);
    return () => ctx.restore();
  }

  function renderFrame() {
    ctx.clearRect(0, 0, appState.viewWidth, appState.viewHeight);
    const restoreTransform = applyFollowTransform();
    const signDetection = appState.showSignHud ? detectNearestTrafficSignInFov(appState.latestPose) : null;
    drawBackground();
    drawGrid();
    drawArenaBounds();
    drawParkingSpot();
    drawInternalWalls();
    drawTrafficSigns();
    drawRobot();
    if (appState.showSignHud) drawSignDetection(signDetection);
    if (appState.showDistances) drawDistanceMeasurements();
    drawGhostRobot(); // render ghost last so it overlays everything
    restoreTransform?.();
    requestAnimationFrame(renderFrame);
  }

  return { renderFrame };
}

