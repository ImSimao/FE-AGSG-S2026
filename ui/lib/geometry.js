import { config } from "./config.js";
import { trafficSigns } from "./traffic.js";
import { clamp, degToRad, rotateDir } from "./math.js";

// The world pose is the odometry origin, which on the real car sits under the
// rear axle. The body is an arm of `rearAxleToCentreMeters` ahead of it: every
// geometric test (collisions, parking rectangle, drawing) works on the body.
export function bodyCentre(pose) {
  const arm = config.rearAxleToCentreMeters || 0;
  if (!arm) return { x: pose.x, y: pose.y };
  const rad = (pose.headingDeg * Math.PI) / 180;
  return { x: pose.x + Math.cos(rad) * arm, y: pose.y + Math.sin(rad) * arm };
}

export function getRobotHalfDims() {
  const body = config.bodyLengthMeters || config.robotLengthMeters;
  return {
    halfL: body / 2,
    halfW: config.robotWidthMeters / 2,
  };
}

export function getSignHalfSize() {
  return config.signSizeMeters / 2;
}

// The nose is a thin stick ahead of the body, 33 x 20 mm, carried on the
// centreline: it is what makes the measured length 160 mm (and the bay 240 mm)
// while adding as little swept volume as possible. It carries no sensors.
function noseCorners(pose) {
  const len = config.noseLengthMeters || 0;
  if (len <= 0) return null;
  const { halfL } = getRobotHalfDims();
  const halfN = (config.noseWidthMeters || 0) / 2;
  const centre = bodyCentre(pose);
  const rad = (pose.headingDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  const at = (lx, ly) => ({ x: centre.x + cos * lx - sin * ly, y: centre.y + sin * lx + cos * ly });
  // Same order as the body box: front-left, front-right, rear-right, rear-left.
  return [at(halfL + len, halfN), at(halfL + len, -halfN), at(halfL, -halfN), at(halfL, halfN)];
}

// Every solid part of the robot, as quads: the body box and the nose stick.
export function robotParts(pose) {
  const parts = [robotCorners(pose)];
  const nose = noseCorners(pose);
  if (nose) parts.push(nose);
  return parts;
}

// Flat list of every corner of every part - collisions, the parking rectangle
// and the "whole body past the sign" rule all work on this outline.
export function robotOutline(pose) {
  return robotParts(pose).flat();
}

function robotCorners(pose) {
  const { halfL, halfW } = getRobotHalfDims();
  const centre = bodyCentre(pose);
  const rad = (pose.headingDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  // Front-left, front-right, rear-right, rear-left
  return [
    { x: centre.x + cos * halfL - sin * halfW, y: centre.y + sin * halfL + cos * halfW },
    { x: centre.x + cos * halfL + sin * halfW, y: centre.y + sin * halfL - cos * halfW },
    { x: centre.x - cos * halfL + sin * halfW, y: centre.y - sin * halfL - cos * halfW },
    { x: centre.x - cos * halfL - sin * halfW, y: centre.y - sin * halfL + cos * halfW },
  ];
}

function projectOnAxis(points, ax, ay) {
  let min = Infinity;
  let max = -Infinity;
  for (const p of points) {
    const proj = p.x * ax + p.y * ay;
    if (proj < min) min = proj;
    if (proj > max) max = proj;
  }
  return { min, max };
}

function axesOverlap(pointsA, pointsB, ax, ay) {
  const len = Math.hypot(ax, ay);
  if (len === 0) return true;
  const ux = ax / len;
  const uy = ay / len;
  const projA = projectOnAxis(pointsA, ux, uy);
  const projB = projectOnAxis(pointsB, ux, uy);
  return projA.max >= projB.min && projB.max >= projA.min;
}

function obbIntersectsAabb(robotPts, rx0, rx1, ry0, ry1) {
  const aabbPts = [
    { x: rx0, y: ry0 },
    { x: rx1, y: ry0 },
    { x: rx1, y: ry1 },
    { x: rx0, y: ry1 },
  ];

  // Axes to test: world X/Y and robot local axes
  const axes = [
    { x: 1, y: 0 },
    { x: 0, y: 1 },
    { x: robotPts[1].x - robotPts[0].x, y: robotPts[1].y - robotPts[0].y },
    { x: robotPts[3].x - robotPts[0].x, y: robotPts[3].y - robotPts[0].y },
  ];

  for (const axis of axes) {
    if (!axesOverlap(robotPts, aabbPts, axis.x, axis.y)) return false;
  }
  return true;
}

export function collidesWithOuterWalls(pose) {
  const corners = robotOutline(pose);
  for (const c of corners) {
    if (c.x < 0 || c.y < 0 || c.x > config.arenaWidthMeters || c.y > config.arenaHeightMeters) {
      return true;
    }
  }
  return false;
}

export function collidesWithInnerWalls(pose) {
  const size = config.internalSquareSizeMeters;
  if (!size || size <= 0) return false;
  const half = size / 2;
  const t = config.wallThicknessMeters;
  const cx = config.arenaWidthMeters / 2;
  const cy = config.arenaHeightMeters / 2;
  const parts = robotParts(pose);

  // vertical walls at x = cx ± half, spanning the square height
  const leftX0 = cx - half - t / 2;
  const leftX1 = cx - half + t / 2;
  const rightX0 = cx + half - t / 2;
  const rightX1 = cx + half + t / 2;
  const y0 = cy - half;
  const y1 = cy + half;

  for (const robotPts of parts) {
    if (
      obbIntersectsAabb(robotPts, leftX0, leftX1, y0, y1) ||
      obbIntersectsAabb(robotPts, rightX0, rightX1, y0, y1)
    ) {
      return true;
    }
  }

  // horizontal walls at y = cy ± half
  const bottomY0 = cy - half - t / 2;
  const bottomY1 = cy - half + t / 2;
  const topY0 = cy + half - t / 2;
  const topY1 = cy + half + t / 2;
  const x0 = cx - half;
  const x1 = cx + half;

  for (const robotPts of parts) {
    if (
      obbIntersectsAabb(robotPts, x0, x1, bottomY0, bottomY1) ||
      obbIntersectsAabb(robotPts, x0, x1, topY0, topY1)
    ) {
      return true;
    }
  }
  return false;
}

export function getParkingSpot() {
  const depth = config.parkingLengthMeters;
  const gap = config.parkingGapMeters || config.parkingGapFactor * config.robotLengthMeters;
  if (depth <= 0 || gap <= 0) return null;
  const centerX = config.arenaWidthMeters / 2;
  const halfSquare = config.internalSquareSizeMeters / 2;
  // Fixed at the right-hand end of the starting section, for both driving
  // directions (coach's call: one bay to practise in, not two). The firmware
  // instead assumes the bay mirrors with the direction (105 cm clockwise,
  // 160 cm otherwise), so the clockwise start is far off its own anchor -
  // see FIRMWARE_NOTES.md.
  //
  // The right magenta element sits right next to the dotted line, so the free
  // area ends one element thickness before x = 2.00; the returned rectangle is
  // the free space between the two elements, not their centrelines.
  const t = config.parkingWallThicknessMeters || config.wallThicknessMeters;
  const rightX = centerX + halfSquare - t;
  const leftX = rightX - gap;
  const y0 = 0;
  const y1 = depth;
  return { leftX, rightX, y0, y1, side: "right", wallThickness: t };
}

export function collidesWithParkingWalls(pose) {
  const spot = getParkingSpot();
  if (!spot) return false;
  const { leftX, rightX, y0, y1 } = spot;
  const t = spot.wallThickness || config.wallThicknessMeters;
  const parts = robotParts(pose);

  // The elements stand outside the free rectangle: the car may touch the free
  // edge without hitting wood.
  const leftX0 = leftX - t;
  const leftX1 = leftX;
  const rightX0 = rightX;
  const rightX1 = rightX + t;

  for (const robotPts of parts) {
    if (
      obbIntersectsAabb(robotPts, leftX0, leftX1, y0, y1) ||
      obbIntersectsAabb(robotPts, rightX0, rightX1, y0, y1)
    ) {
      return true;
    }
  }
  return false;
}

// Which sign the body is touching, or null. Signs do not block the car - rule
// 9.20 allows touching them, it just costs points - so the caller wants the
// sign itself, to say which one was hit and when.
export function trafficSignCollision(pose) {
  const parts = robotParts(pose);
  const half = getSignHalfSize();
  for (const sign of trafficSigns) {
    if (sign.state === 0) continue; // ignore inactive/empty signs
    const sx = sign.x + (sign.dx || 0);
    const sy = sign.y + (sign.dy || 0);
    const x0 = sx - half;
    const x1 = sx + half;
    const y0 = sy - half;
    const y1 = sy + half;
    for (const robotPts of parts) {
      if (obbIntersectsAabb(robotPts, x0, x1, y0, y1)) return sign;
    }
  }
  return null;
}

export function collidesWithTrafficSigns(pose) {
  return trafficSignCollision(pose) !== null;
}

export function resetSignDisplacements() {
  for (const s of trafficSigns) {
    s.dx = 0;
    s.dy = 0;
  }
}

export function collidesWithAnything(pose) {
  if (collidesWithOuterWalls(pose)) return "outer wall";
  if (collidesWithInnerWalls(pose)) return "inner wall";
  if (collidesWithParkingWalls(pose)) return "parking wall";
  // Traffic signs do NOT block: rule 9.20 allows touching, moving or knocking
  // them down. It costs points (1.4 / 1.6), so the race module records contact
  // instead of stopping the car. See FIRMWARE_NOTES.md.
  return null;
}

function wallNormalForHit(point, rect) {
  const distLeft = Math.abs(point.x - rect.x0);
  const distRight = Math.abs(point.x - rect.x1);
  const distBottom = Math.abs(point.y - rect.y0);
  const distTop = Math.abs(point.y - rect.y1);
  const min = Math.min(distLeft, distRight, distBottom, distTop);
  if (min === distLeft) return { x: -1, y: 0 };
  if (min === distRight) return { x: 1, y: 0 };
  if (min === distBottom) return { x: 0, y: -1 };
  return { x: 0, y: 1 };
}

function angleToNormal(dir, normal) {
  const dot = Math.abs(dir.x * normal.x + dir.y * normal.y);
  const clamped = clamp(dot, -1, 1);
  return Math.acos(clamped);
}

export function rayAabbIntersection(origin, dir, rect) {
  const dx = dir.x;
  const dy = dir.y;

  let txMin;
  let txMax;
  if (dx === 0) {
    if (origin.x < rect.x0 || origin.x > rect.x1) return null;
    txMin = -Infinity;
    txMax = Infinity;
  } else {
    const tx1 = (rect.x0 - origin.x) / dx;
    const tx2 = (rect.x1 - origin.x) / dx;
    txMin = Math.min(tx1, tx2);
    txMax = Math.max(tx1, tx2);
  }

  let tyMin;
  let tyMax;
  if (dy === 0) {
    if (origin.y < rect.y0 || origin.y > rect.y1) return null;
    tyMin = -Infinity;
    tyMax = Infinity;
  } else {
    const ty1 = (rect.y0 - origin.y) / dy;
    const ty2 = (rect.y1 - origin.y) / dy;
    tyMin = Math.min(ty1, ty2);
    tyMax = Math.max(ty1, ty2);
  }

  const tEnter = Math.max(txMin, tyMin);
  const tExit = Math.min(txMax, tyMax);
  if (tExit < 0 || tEnter > tExit) return null;
  return tEnter >= 0 ? tEnter : tExit; // if inside, exit distance is positive
}

export function buildWallAabbs() {
  const walls = [
    {
      x0: 0,
      x1: config.arenaWidthMeters,
      y0: 0,
      y1: config.arenaHeightMeters,
      label: "outer wall",
    },
  ];

  const t = config.wallThicknessMeters;
  const size = config.internalSquareSizeMeters;
  if (size > 0 && t > 0) {
    const cx = config.arenaWidthMeters / 2;
    const cy = config.arenaHeightMeters / 2;
    const half = size / 2;
    walls.push(
      { x0: cx - half - t / 2, x1: cx - half + t / 2, y0: cy - half, y1: cy + half, label: "inner wall" }, // left
      { x0: cx + half - t / 2, x1: cx + half + t / 2, y0: cy - half, y1: cy + half, label: "inner wall" }, // right
      { x0: cx - half, x1: cx + half, y0: cy - half - t / 2, y1: cy - half + t / 2, label: "inner wall" }, // bottom
      { x0: cx - half, x1: cx + half, y0: cy + half - t / 2, y1: cy + half + t / 2, label: "inner wall" }, // top
    );
  }

  const spot = getParkingSpot();
  if (spot) {
    const tb = spot.wallThickness || t;
    walls.push(
      { x0: spot.leftX - tb, x1: spot.leftX, y0: spot.y0, y1: spot.y1, label: "parking wall" },
      { x0: spot.rightX, x1: spot.rightX + tb, y0: spot.y0, y1: spot.y1, label: "parking wall" },
    );
  }

  return walls;
}

export function buildSignAabbs() {
  const half = getSignHalfSize();
  if (half <= 0) return [];
  return trafficSigns
    .filter((s) => s.state !== 0) // only active signs block distance
    .map((s) => ({
      x0: s.x - half,
      x1: s.x + half,
      y0: s.y - half,
      y1: s.y + half,
      label: "traffic sign",
    }));
}

export function raycastToWalls(origin, dir, walls) {
  const hits = [];
  for (const wall of walls) {
    const t = rayAabbIntersection(origin, dir, wall);
    if (t === null || t < 0) continue;
    const point = { x: origin.x + dir.x * t, y: origin.y + dir.y * t };
    const normal = wallNormalForHit(point, wall);
    const angle = angleToNormal(dir, normal);
    hits.push({
      distance: t,
      point,
      label: wall.label,
      angle,
    });
  }
  return hits;
}

export function nearestHitAlongDir(origin, dir, walls, signs, { ignorePerpLimit = false } = {}) {
  const hits = [
    ...raycastToWalls(origin, dir, walls),
    ...raycastToWalls(origin, dir, signs),
  ];
  if (!hits.length) return null;
  const nearest = hits.reduce((best, h) => (!best || h.distance < best.distance ? h : best), null);
  if (!nearest) return null;
  if (!ignorePerpLimit && nearest.angle > getMaxPerpRad()) return null;
  return nearest;
}

export function getSensorHalfFovRad() {
  return degToRad(config.sensorHalfFovDeg || 0);
}

export function getMaxPerpRad() {
  return degToRad(config.maxPerpDeg || 0);
}

export function getSampleAngles(steps = 29) {
  const half = getSensorHalfFovRad();
  const angles = [];
  for (let i = 0; i < steps; i++) {
    const t = steps === 1 ? 0 : i / (steps - 1);
    angles.push(-half + t * (2 * half));
  }
  return angles;
}

export function nearestHitInCone(origin, baseDir, walls, signs) {
  // Sample across ±22.5° to mimic ultrasonic beam spread.
  const samples = getSampleAngles();
  let best = null;
  for (const angle of samples) {
    const dir = rotateDir(baseDir, angle);
    const hit = nearestHitAlongDir(origin, dir, walls, signs);
    if (!hit) continue;
    if (!best || hit.distance < best.distance) {
      best = { ...hit, dir };
    }
  }
  return best;
}

