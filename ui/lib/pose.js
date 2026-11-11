import { config } from "./config.js";
import { appState, Modes } from "./appState.js";
import { trafficSigns, laneBounds } from "./traffic.js";
import {
  buildSignAabbs,
  buildWallAabbs,
  collidesWithAnything,
  getRobotHalfDims,
  getSensorHalfFovRad,
  getSignHalfSize,
  nearestHitAlongDir,
  nearestHitInCone,
  rayAabbIntersection,
} from "./geometry.js";
import { degToRad } from "./math.js";

let log = () => {};
let onCursorRefresh = () => {};

export function setLog(fn) {
  log = fn;
}

export function setCursorRefresh(fn) {
  onCursorRefresh = fn;
}

export function setLastMouseWorld(world) {
  appState.lastMouseWorld = world;
}

export function getLastMouseWorld() {
  return appState.lastMouseWorld;
}

export function setLatestTelemetryDistances(distances) {
  appState.latestTelemetryDistances = distances;
}

export function noteCollisionOnce(message) {
  const now = (typeof performance !== "undefined" && performance.now) ? performance.now() : Date.now();
  if (now - appState.lastCollisionMessageAt > 500) {
    log(message);
    appState.lastCollisionMessageAt = now;
  }
}

function warnTelemetryWallCollision(pose) {
  if (appState.currentMode !== Modes.TELEMETRY || !pose) return;
  const collisionType = collidesWithAnything(pose);
  if (collisionType && collisionType.includes("wall")) {
    noteCollisionOnce(`Collision with ${collisionType} (telemetry).`);
  }
}

export function updateLatestPose(pose) {
  if (appState.latestPose) appState.prevPose = appState.latestPose;
  appState.latestPose = pose;
  warnTelemetryWallCollision(appState.latestPose);
  recordSignPassages(appState.prevPose, appState.latestPose);
  onCursorRefresh?.();
}

// Use when the user sets the pose (click/drag). Keeps simulator in sync.
export function setPoseFromUser(pose) {
  updateLatestPose(pose);
  if (appState.simRunning) {
    appState.simPose = { ...pose };
  }
}

function pointInRect(p, rect, tol = 0) {
  return (
    p.x >= rect.x0 - tol &&
    p.x <= rect.x1 + tol &&
    p.y >= rect.y0 - tol &&
    p.y <= rect.y1 + tol
  );
}

// localY is the sign's offset along the car's left-hand normal, so a positive
// value means the sign is on the left of the direction of travel.
//
// Careful: "the sign is on my left" and "I passed on the left of the sign" are
// opposite statements about the same pass. Everything the round records and the
// log prints is the second one - the side of the SIGN the car went past - which
// is what the WRO wording means ("pass on the left of the sign").
function passingSide(localY) {
  return localY >= 0 ? "right" : "left";
}

function recordSignPassages(prev, curr) {
  if (!prev || !curr) return;
  const dx = curr.x - prev.x;
  const dy = curr.y - prev.y;
  const dist = Math.hypot(dx, dy);
  if (dist < 1e-4) return;
  // A jump this big is a reset, a teleport or a page reload - not a pass. Without
  // this, every reconnect fired one warning per sign the car "crossed" on the way.
  if (dist > 0.25) return;

  const cos = dx / dist;
  const sin = dy / dist;
  const tol = 0.01; // meters tolerance for lane proximity

  // One entry per lane *visit*. The firmware stops, reverses and nudges while it
  // lines up, and each of those moves can carry the car back across the plane of
  // a sign it has already passed - backwards, from the other side - which was
  // recorded as a second passage with the opposite side. Measured on the sweep:
  // `top_rr_a12` recorded the red at (150,260) as "right" from (149,223) at
  // t=27.0 s and as "left" from (151,223) at t=30.8 s, two centimetres apart.
  // Under the current rule a wrong side ends the round, so a correct pass was
  // ending the round. A lane visit resets when the car leaves the lane rectangle.
  const visits = appState.laneVisits || (appState.laneVisits = {});
  const wasInside = appState.laneInside || (appState.laneInside = {});
  for (const lane of Object.keys(laneBounds)) {
    const inside = pointInRect(curr, laneBounds[lane], tol * 2);
    if (inside && !wasInside[lane]) visits[lane] = (visits[lane] || 0) + 1;
    wasInside[lane] = inside;
  }

  trafficSigns.forEach((sign, idx) => {
    if (sign.state === 0) return; // inactive
    const rect = laneBounds[sign.lane];
    if (!rect) return;

    // both poses should be near the lane rectangle to *start* a pass, but the
    // sign often stands on the lane's own boundary (the corner signs at x=2.00
    // and y=2.00), so the car leaves the rectangle one frame after crossing it
    // and the commit below has to happen outside.
    const rectOk = pointInRect(prev, rect, tol) && pointInRect(curr, rect, tol);

    // A pass only counts once the robot has gone *completely* past the sign: the
    // whole 127 mm body has to clear the sign's plane, and it has to get there
    // running the lane. Before this, the crossing fired as soon as the centre
    // brushed the plane while turning into the parking bay - on
    // prova-4111204647-...-211055 the car was 22 cm south of the green at
    // (200,60) on a 35 degree diagonal and the round died at 66.2 s for a pass
    // it never made. The crossing is remembered here and committed below, once
    // the tail is past.
    const laneAlongX = sign.lane === "bottom" || sign.lane === "top";
    const step = laneAlongX ? dx : dy; // displacement along the lane
    const across = laneAlongX ? Math.abs(dy) : Math.abs(dx); // off the lane axis
    const diagonal = across > Math.abs(step) * 0.577; // over 30 deg off the lane

    const clock = appState.simClockMs || 0;
    const visit = visits[sign.lane] || 0;
    const pendings = appState.signPending || (appState.signPending = {});
    // Progress along the lane axis, signed by the run direction: a turn after the
    // pass must not make it go backwards and cancel the pending crossing.
    const progress = (pose, dir) =>
      (laneAlongX ? pose.x - sign.x : pose.y - sign.y) * dir;

    let pass = pendings[idx];
    if (pass && (clock - pass.t > 4000 || pass.visit !== visit ||
                 progress(curr, pass.dir) < 0)) {
      delete pendings[idx]; // backed out, restarted the lane, or took too long
      pass = undefined;
    }

    if (!pass && rectOk && step !== 0) {
      const dir = step > 0 ? 1 : -1;
      if (progress(prev, dir) < 0 && progress(curr, dir) >= 0 && !diagonal) {
        // Side, taken as the car crosses the plane so the turn after does not
        // skew it: positive localY = the sign is on the car's left.
        const localY = (sign.y - curr.y) * cos - (sign.x - curr.x) * sin;
        pendings[idx] = { side: passingSide(localY), t: clock, visit, dir };
        pass = pendings[idx];
      }
    }
    if (!pass) return;

    // "Completely past": the sign is at least half a body behind the centre, so
    // the rear of the robot has cleared it. Only then is the side it took a fact.
    if (progress(curr, pass.dir) < config.robotLengthMeters / 2) return;
    delete pendings[idx];

    // The line the firmware documents (field.py: "#1 left side (green), -1 right
    // side (red)"): green => pass on the left of the sign, red => pass on its
    // right. Nothing here is scored - only touching a sign costs points - so this
    // just says whether the car took the line the firmware aimed for.
    const sideOfSign = pass.side;
    const intendedSide = sign.state === 1 ? "left" : "right";
    const onIntendedSide = sideOfSign === intendedSide;

    // Every pass goes into the round record, right or wrong: an exported round
    // has to show which signs the car actually met, not only the ones it got
    // wrong. One entry per sign per lane visit.
    const lastPass = appState.signPassedAt[idx];
    const seenThisVisit = appState.signPassedVisit && appState.signPassedVisit[idx] === visit;
    if (!seenThisVisit && (lastPass === undefined || clock - lastPass > 2000 || clock < lastPass)) {
      appState.signPassedAt[idx] = clock;
      (appState.signPassedVisit || (appState.signPassedVisit = {}))[idx] = visit;
      appState.signPassages.push({
        t_ms: Math.round(clock),
        lane: sign.lane,
        colour: sign.state === 1 ? "green" : "red",
        side: sideOfSign, // side of the SIGN the car went past
        intended_side: intendedSide, // the line the firmware documents
        on_intended_side: onIntendedSide, // not a rule: only touching a sign costs points

        sign_x_cm: +(sign.x * 100).toFixed(1),
        sign_y_cm: +(sign.y * 100).toFixed(1),
        car_x_cm: +(curr.x * 100).toFixed(1),
        car_y_cm: +(curr.y * 100).toFixed(1),
      });
    }

    if (onIntendedSide) return;

    const now = Date.now();
    const last = appState.lastSignWarningAt[idx] || 0;
    if (now - last < 800) return;
    appState.lastSignWarningAt[idx] = now;

    // A note, not a warning: taking the other side of a sign breaks no rule.
    log(
      `Sinal ${sign.state === 1 ? "verde" : "vermelho"} em ${sign.lane} passado pelo lado ${sideOfSign} (a linha do firmware é pelo lado ${intendedSide}).`
    );
  });
}

function sonarSpecs(forward, right) {
  // The world pose IS the odometry origin (under the rear axle), which is the
  // same reference the firmware uses, so its offsets apply unchanged.
  const s = config.sonar;
  const frontOffset = s.frontMeters;
  const rearOffset = s.rearMeters;
  const sideForward = s.sideForwardMeters;
  return [
    { label: "Front", dir: forward, offset: frontOffset },
    { label: "Rear", dir: { x: -forward.x, y: -forward.y }, offset: rearOffset },
    {
      label: "Left",
      dir: { x: -right.x, y: -right.y },
      offset: s.sideLateralMeters,
      startShift: { x: forward.x * sideForward, y: forward.y * sideForward },
    },
    {
      label: "Right",
      dir: right,
      offset: s.sideLateralMeters,
      startShift: { x: forward.x * sideForward, y: forward.y * sideForward },
    },
  ];
}

export function measurePoseDistances(pose) {
  const rad = (pose.headingDeg * Math.PI) / 180;
  const forward = { x: Math.cos(rad), y: Math.sin(rad) };
  const right = { x: forward.y, y: -forward.x };
  const { halfL, halfW } = getRobotHalfDims();
  const origin = { x: pose.x, y: pose.y };
  const walls = buildWallAabbs();
  const signs = buildSignAabbs();

  const specs = sonarSpecs(forward, right);

  return specs
    .map((spec) => {
      const base = {
        x: origin.x + (spec.startShift?.x || 0),
        y: origin.y + (spec.startShift?.y || 0),
      };
      const nearest = appState.realWorldMode
        ? nearestHitInCone(base, spec.dir, walls, signs)
        : nearestHitAlongDir(base, spec.dir, walls, signs, { ignorePerpLimit: true });
      if (!nearest) return null;
      const start = {
        x: base.x + spec.dir.x * spec.offset,
        y: base.y + spec.dir.y * spec.offset,
      };
      const clearance = Math.max(0, nearest.distance - spec.offset);
      const dir = nearest.dir || spec.dir;
      const end = {
        x: start.x + dir.x * clearance,
        y: start.y + dir.y * clearance,
      };
      return { ...spec, distance: clearance, start, end, dir, baseDir: spec.dir };
    })
    .filter(Boolean);
}

function buildTelemetryMeasurements(pose, distancesByLabel) {
  if (!distancesByLabel) return [];
  const rad = (pose.headingDeg * Math.PI) / 180;
  const forward = { x: Math.cos(rad), y: Math.sin(rad) };
  const right = { x: forward.y, y: -forward.x };
  const specs = sonarSpecs(forward, right);

  return specs
    .map((spec) => {
      const distance = distancesByLabel[spec.label];
      if (distance === undefined || distance === null || Number.isNaN(distance)) return null;
      const baseX = pose.x + (spec.startShift?.x || 0);
      const baseY = pose.y + (spec.startShift?.y || 0);
      const start = {
        x: baseX + spec.dir.x * spec.offset,
        y: baseY + spec.dir.y * spec.offset,
      };
      const clampedDistance = Math.max(0, distance);
      const end = {
        x: start.x + spec.dir.x * clampedDistance,
        y: start.y + spec.dir.y * clampedDistance,
      };
      return { ...spec, distance: clampedDistance, start, end, baseDir: spec.dir };
    })
    .filter(Boolean);
}

export function getDistanceMeasurements() {
  if (!appState.latestPose) return [];
  if (appState.currentMode === Modes.TELEMETRY) {
    if (appState.latestTelemetryDistances) {
      return buildTelemetryMeasurements(appState.latestPose, appState.latestTelemetryDistances);
    }
    return [];
  }
  return measurePoseDistances(appState.latestPose);
}

export function detectTrafficSignsInFov(pose = appState.latestPose) {
  if (!pose) return [];
  const activeSigns = trafficSigns.filter((s) => s.state !== 0);
  if (!activeSigns.length) return [];

  const headingRad = degToRad(pose.headingDeg);
  const forward = { x: Math.cos(headingRad), y: Math.sin(headingRad) };
  // The pose is the rear axle (the odometry origin). The camera is at the front
  // of the car, 13.5 cm ahead of that origin - the same constant camera.py uses.
  const cameraOffset = config.cameraOffsetFromAxleMeters ?? 0.135;
  const origin = { x: pose.x + forward.x * cameraOffset, y: pose.y + forward.y * cameraOffset };
  const halfFov = degToRad(config.signVisionHalfFovDeg || 60);
  const walls = buildWallAabbs();
  const signHalf = getSignHalfSize();

  const detections = [];
  for (const sign of activeSigns) {
    const dx = sign.x - origin.x;
    const dy = sign.y - origin.y;
    const distance = Math.hypot(dx, dy);
    if (distance <= 0) continue;

    // Signed angle between forward vector and vector to the sign.
    const cross = forward.x * dy - forward.y * dx;
    const dot = forward.x * dx + forward.y * dy;
    const angleRad = Math.atan2(cross, dot);
    if (Math.abs(angleRad) > halfFov) continue;

    // Line-of-sight: stop if a wall intersects before the sign.
    const dir = { x: dx / distance, y: dy / distance };
    let blocked = false;
    for (const wall of walls) {
      const t = rayAabbIntersection(origin, dir, wall);
      if (t === null || t < 0) continue;
      if (t < distance - signHalf) {
        blocked = true;
        break;
      }
    }
    if (blocked) continue;

    detections.push({
      sign,
      distance,
      angleRad,
      angleDeg: (angleRad * 180) / Math.PI,
      origin,
    });
  }

  detections.sort((a, b) => a.distance - b.distance);
  return detections;
}

export function detectNearestTrafficSignInFov(pose = appState.latestPose) {
  const all = detectTrafficSignsInFov(pose);
  return all.length ? all[0] : null;
}

export function formatSignedDeg(deg) {
  if (!Number.isFinite(deg)) return "--°";
  const rounded = deg.toFixed(0);
  return `${deg >= 0 ? "+" : ""}${rounded}°`;
}

