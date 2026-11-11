import { appState, Modes } from "./appState.js";
import { config } from "./config.js";
import { collidesWithAnything, getParkingSpot } from "./geometry.js";
import { updateLatestPose, noteCollisionOnce } from "./pose.js";
import { rngNext } from "./rng.js";

export function createSimulator({ log }) {
  // Staleness of the last control command, in SIMULATED milliseconds. It used to
  // be wall-clock: at 4x the same 400 ms covered four times as much driving, so a
  // hiccup that dropped the command for 0.4 s of simulated time at 1x dropped it
  // for 1.6 s at 4x and the car coasted further.
  const CONTROL_STALE_MS = 400;
  const CONTROL_STALE_SIM_MS = 400;

  function toggleSimulator() {
    if (appState.currentMode !== Modes.PLAY) {
      log("Switch to Play mode to use the simulator.");
      return;
    }
    if (appState.simRunning) {
      stopSimulator();
    } else {
      startSimulator(appState.latestPose);
    }
  }

  function pickSimStartPose(initialPose) {
    if (!initialPose && config.startPose) {
      initialPose = { ...config.startPose };
    }
    if (initialPose && !collidesWithAnything(initialPose)) {
      const deg = ((initialPose.headingDeg % 360) + 360) % 360;
      const startClockwise = deg > 90 && deg < 270;
      appState.parkingSide = startClockwise ? "left" : "right";
      return { pose: { ...initialPose }, startClockwise, reason: "custom" };
    }
    const forced = config.startDirection;
    const startClockwise =
      forced === "cw" ? true : forced === "ccw" ? false : rngNext() < 0.5;
    // Clockwise around the track = driving west along the bottom straight, with
    // the outer wall on the left. The bay always sits at the end of the start
    // section the car drives towards (rule: right magenta element next to the
    // dotted line), which is what the firmware assumes (parking at 105 / 160 cm).
    const startHeadingDeg = startClockwise ? 180 : 0;
    appState.parkingSide = startClockwise ? "left" : "right";
    const spot = getParkingSpot();
    // Reach of the solid part from the body centre: the nose stick counts in
    // front, the rear overhang behind (the pose is the axle, one arm back).
    const body = config.bodyLengthMeters || config.robotLengthMeters;
    const frontReach = body / 2 + (config.noseLengthMeters || 0);
    const rearReach = body / 2 - (config.rearAxleToCentreMeters || 0);
    let startX = spot ? (spot.leftX + spot.rightX) / 2 : config.arenaWidthMeters / 2;
    if (spot && config.startMode && config.startMode !== "centre") {
      // "nose": the wall the car is facing is 1.75 cm away. "nose_in": the same
      // distance to the wall behind it. Both mirror with the heading, not with
      // the bay (the bay is fixed).
      const facingEast = !startClockwise;
      const opposite = facingEast ? spot.rightX : spot.leftX;
      const behind = facingEast ? spot.leftX : spot.rightX;
      const sign = facingEast ? -1 : 1;
      startX =
        config.startMode === "nose"
          ? opposite + sign * (0.0175 + frontReach)
          : behind - sign * (0.0175 + rearReach);
    }
    // startX/startY place the BODY; the pose is the axle, one arm behind it.
    const arm = config.rearAxleToCentreMeters || 0;
    const rad = (startHeadingDeg * Math.PI) / 180;
    // "above": the middle zone over the parking lot, which the rules allow as an
    // alternative start (no 1.8.1 points). The bay itself is only 190.5 mm long
    // (1.5 x the 127 mm chassis) and the firmware opens with a 70 cm/s move, so a
    // bay start jams immediately.
    let startY = spot
      ? spot.y1 - config.robotWidthMeters / 2 - 0.01
      : config.arenaHeightMeters / 2;
    if (spot && config.startMode === "above") {
      startY = spot.y1 + 0.14;
    }
    if (spot && config.startMode === "wall") {
      // Parked like the real car: rear face 1 cm from the magenta wall behind it.
      const facingEast2 = !startClockwise;
      const rearX = facingEast2 ? spot.leftX + 0.01 : spot.rightX - 0.01;
      startX = rearX + (facingEast2 ? 1 : -1) * (0.0255 + arm);
    }
    return {
      pose: {
        x: startX - Math.cos(rad) * arm,
        y: startY - Math.sin(rad) * arm,
        headingDeg: startHeadingDeg,
      },
      startClockwise,
      reason: "random",
    };
  }

  function markRaceStart(pose) {
    if (!pose) return;
    appState.steerActualDeg = 0; // a folga arranca sempre do zero
    appState.steerContact = 0;
    appState.raceStartPose = { x: pose.x, y: pose.y, headingDeg: pose.headingDeg };
    appState.raceStartHeading = pose.headingDeg;
  }

  function startSimulator(initialPose) {
    appState.simRunning = true;
    appState.simTime = 0;
    appState.simVel = 0;
    const startInfo = pickSimStartPose(initialPose);
    if (initialPose && startInfo.reason !== "custom") {
      log("Play start pose collided with an obstacle; using default simulator start.");
    }
    appState.simPose = startInfo.pose;
    appState.prevPose = null;
    updateLatestPose({ ...appState.simPose });
    markRaceStart(appState.simPose);
    const simFps = Math.max(1, config.simFps || 60);
    appState.simDt = (1 / simFps) * (appState.simScale || 1);
    appState.simRunning = true;
    appState.simClockMs = 0;
    appState.simStats = { steps: 0, reversals: 0, path: 0, turn: 0, lastSign: 0 };
    appState.replayFrames = [];
    appState.replayMode = false;
    appState.simFrozen = false;
    appState.controlQueue = [];
    appState.wsControl = null;
    applySimScale();
    if (startInfo.reason === "custom") {
      log("Simulator started from current pose (W/A/S/D control).");
    } else {
      log(`Simulator started (${startInfo.startClockwise ? "clockwise" : "counter-clockwise"} start; W/A/S/D control).`);
    }
  }

  function stopSimulator() {
    appState.simRunning = false;
    clearInterval(appState.simTimer);
    appState.simTimer = null;
    appState.simPose = null;
    appState.simVel = 0;
    log("Simulator stopped.");
  }

  const REPLAY_MAX_FRAMES = 30000; // ~8 minutes at 60 frames/s
  const CONTROL_QUEUE_MAX = 400;

  // The firmware issues commands at its own control rate (~50 Hz at 1x, ~200 at
  // 4x) while the physics redraws at 60 Hz. Applying only "the newest command"
  // discarded three out of four at 4x and the car followed a broken staircase of
  // the intended steering - the zigzag. The commands are queued with the sim time
  // they arrived at and consumed in order, one per physics sub-step.
  function queueControl(velocity, steeringRadius, omega, atSimMs) {
    const queue = appState.controlQueue || (appState.controlQueue = []);
    // Queue at the instant the controller says the command belongs to, not at
    // the browser's clock, which only advances once per display frame. At 4x a
    // frame carries 4 commands and they all landed on the same timestamp, so the
    // physics saw one of them per 66.7 ms instead of one per 20 ms of simulated
    // time - the car was steered 3x coarser than at 1x, which is why the same
    // seed drove measurably differently.
    const t = Number.isFinite(atSimMs) ? atSimMs : (appState.simClockMs || 0);
    queue.push({ t, velocity, steeringRadius, omega });
    while (queue.length > CONTROL_QUEUE_MAX) queue.shift();
  }

  function drainControl(untilMs) {
    const queue = appState.controlQueue;
    if (!queue || !queue.length) return;
    let applied = null;
    while (queue.length && queue[0].t <= untilMs) applied = queue.shift();
    if (!applied) return;
    appState.wsControl = {
      velocity: applied.velocity,
      steeringRadius: applied.steeringRadius,
      omegaDegPerSec: applied.omega,
    };
    appState.wsControlUpdatedAt = Date.now();
    appState.wsControlSimMs = applied.t;
  }

  function simTick() {
    // Frozen when the round is over (so the car stays where it stopped and can be
    // inspected) or while a replay is being scrubbed.
    if (appState.replayMode || appState.simFrozen) return;
    // In lock-step the controller owns time: stepTo() advances the world and this
    // timer only keeps the display alive.
    if (appState.simLockStep) return;
    // One display frame carries `simScale` frames of simulated time, integrated in
    // equal sub-steps so the step stays 1/60 s of simulated time. Rendering stays
    // at 60 Hz (a canvas redraw cannot run at 240 Hz), but the *steering command*
    // is interpolated across the sub-steps: at 4x the firmware issues ~200
    // commands a second and the physics used to apply only one of them per frame,
    // so the decimation aliased into a zigzag.
    const steps = Math.max(1, Math.min(4, Math.ceil(appState.simScale || 1)));
    const dt = appState.simDt / steps;
    const control = appState.wsControl;
    for (let i = 0; i < steps; i += 1) {
      drainControl(appState.simClockMs + dt);
      simStep(dt);
    }
  }

  // Keep the step constant and scale the tick rate. Called on start and whenever
  // the speed button changes the multiplier.
  function applySimScale() {
    const simFps = Math.max(1, config.simFps || 60);
    const scale = Math.max(1, appState.simScale || 1);
    appState.simDt = (1 / simFps) * scale;
    if (!appState.simRunning) return;
    if (appState.simTimer) clearInterval(appState.simTimer);
    appState.simTimer = setInterval(simTick, 1000 / simFps);
  }

  function setLockStep(on) {
    appState.simLockStep = !!on;
  }

  // Advance the world to `targetMs` in fixed physics steps, applying the queued
  // control commands at the instants they belong to. Bounded: a stalled tab must
  // not be able to integrate for ever in one call.
  function stepTo(targetMs, stepMs) {
    if (!appState.simPose || appState.replayMode || appState.simFrozen) return appState.simClockMs;
    // 5 ms in lock-step: the firmware's 20 ms control period then lands exactly on
    // a step boundary, so it still runs at 50 Hz. On the 16.7 ms display grid a
    // 20 ms sleep rounds up to 33.3 ms and the firmware would drive at 30 Hz.
    const dtMs = Number.isFinite(stepMs) && stepMs > 0 ? stepMs : 1000 / (config.simFps || 60);
    let guard = 0;
    while (appState.simClockMs + dtMs <= targetMs + 1e-6 && guard < 64) {
      drainControl(appState.simClockMs + dtMs);
      simStep(dtMs / 1000);
      guard += 1;
    }
    return appState.simClockMs;
  }

  // Smallest free pose reachable from a wedged one: 16 directions, 1 mm steps up
  // to 8 mm. Bounded on purpose - it resolves a millimetre of penetration, it can
  // never walk the car through a wall.
  function unstick(pose, headingDeg) {
    for (let radius = 1; radius <= 8; radius += 1) {
      for (let i = 0; i < 16; i += 1) {
        const angle = (i * Math.PI) / 8;
        const candidate = {
          x: pose.x + Math.cos(angle) * 0.001 * radius,
          y: pose.y + Math.sin(angle) * 0.001 * radius,
          headingDeg,
        };
        if (!collidesWithAnything(candidate)) return candidate;
      }
    }
    return null;
  }

  function simStep(dt) {
    if (!appState.simPose) return;
    // Everything that measures the round - the timer, the stop hold, the time
    // limit - has to count simulated seconds, otherwise 4x speeds the car up but
    // leaves the clock at 1x and a "180 s" round is really 45 s of race.
    appState.simClockMs = (appState.simClockMs || 0) + dt * 1000;
    // Sensor streams fire on simulated time, not on browser timers.
    appState.onSimStep?.(appState.simClockMs);

    // Recorder: one compact frame per physics step, so the replay is the run as
    // the physics saw it and not a 10 Hz sampling of it.
    const frames = appState.replayFrames;
    if (frames && frames.length < REPLAY_MAX_FRAMES) {
      frames.push({
        t: appState.simClockMs,
        x: appState.simPose.x,
        y: appState.simPose.y,
        h: appState.simPose.headingDeg,
        v: appState.simVel,
      });
    }
    const vMag = config.simSpeedMetersPerSec;
    const turnRadius = config.simTurnRadiusMeters;
    const accel = config.simAccelMetersPerSec2;
    const brake = config.simBrakeMetersPerSec2;

    const hasWsControl =
      appState.wsControl &&
      Number.isFinite(appState.wsControlSimMs) &&
      appState.simClockMs - appState.wsControlSimMs < CONTROL_STALE_SIM_MS;

    let throttle = 0;
    let desired;
    if (hasWsControl) {
      const cap = config.maxControlSpeedMetersPerSec ?? vMag;
      desired = Math.max(-cap, Math.min(cap, appState.wsControl.velocity ?? 0));
    } else {
      if (appState.keyState.w) throttle += 1;
      if (appState.keyState.s) throttle -= 1;
      desired = throttle * vMag;
    }

    // Brake whenever the car has to lose speed or change direction, accelerate
    // only when the command asks for more speed in the same direction. The old
    // `throttle !== 0` test used the brake rate while accelerating under
    // firmware control, which made every start look instantaneous.
    const dv = desired - appState.simVel;
    const sameDirection =
      appState.simVel === 0 || Math.sign(desired) === Math.sign(appState.simVel);
    const slowing = !sameDirection || Math.abs(desired) < Math.abs(appState.simVel);
    const rate = slowing ? brake : accel;
    const maxStep = rate * dt;
    if (Math.abs(dv) <= maxStep) {
      appState.simVel = desired;
    } else {
      appState.simVel += Math.sign(dv) * maxStep;
    }

    let omega = 0;
    // Steering is a mechanical system: the servo and linkage have a dead band
    // ("folga") and take time to reach the commanded angle. Both are applied to
    // the angle, and the yaw rate is then derived from the angle that the wheels
    // actually reached, not from the one that was asked for.
    const realism = config.realism || {};
    let steerTargetDeg = 0;
    let keyboardTurn = false;
    if (hasWsControl) {
      // Geometric steering: R comes from the servo angle (R = L / tan(delta)),
      // and the yaw rate follows the live speed rather than the speed at the
      // moment the command arrived.
      const radius = Number(appState.wsControl.steeringRadius);
      if (Number.isFinite(radius) && Math.abs(radius) > 1e-6) {
        steerTargetDeg = Math.atan((realism.wheelbaseM || 0.07) / radius) * (180 / Math.PI);
      }
    } else if (appState.simVel !== 0) {
      const kb = Math.atan((realism.wheelbaseM || 0.07) / turnRadius) * (180 / Math.PI);
      if (appState.keyState.a) steerTargetDeg += kb;
      if (appState.keyState.d) steerTargetDeg -= kb;
      keyboardTurn = true;
    }

    const deadband = realism.enabled ? Number(realism.steerBacklashDeg) || 0 : 0;
    const lagS = realism.enabled ? Number(realism.steerLagS) || 0 : 0;
    // Backlash is lost motion, not a permanent offset: while the servo keeps
    // pushing the same way the wheels track it exactly; when it reverses, they
    // stand still until the play (2 x deadband) is taken up. Modelling it as an
    // offset made every command permanently 1.5 deg short, which put a bias on
    // every straight and destroyed the parking manoeuvre.
    const outDeg = appState.steerActualDeg || 0;
    const gap = steerTargetDeg - outDeg;
    let contact = appState.steerContact || 0;
    let moved = false;
    if (deadband <= 0) {
      moved = true;
      contact = Math.sign(gap) || 0;
    } else if (contact !== 0 && Math.sign(gap) === contact) {
      moved = true; // still pushing the same way: the wheels track the servo
    } else if (Math.abs(gap) > deadband) {
      moved = true; // the play has been taken up on the other side
      contact = Math.sign(gap);
    } else {
      contact = 0; // inside the play: the wheels do not move
    }
    if (moved) {
      const alpha = lagS > 0 ? Math.min(1, dt / lagS) : 1;
      appState.steerActualDeg = outDeg + (steerTargetDeg - outDeg) * alpha;
    }
    appState.steerContact = contact;

    const wheelDeg = appState.steerActualDeg || 0;
    if (Math.abs(wheelDeg) > 1e-3 && Math.abs(appState.simVel) > 1e-6) {
      const wheelRad = (wheelDeg * Math.PI) / 180;
      const effRadius = (realism.wheelbaseM || 0.07) / Math.tan(wheelRad);
      omega = (appState.simVel / effRadius) * (180 / Math.PI);
    } else if (!hasWsControl && keyboardTurn && appState.simVel !== 0) {
      omega = (appState.simVel / turnRadius) * (180 / Math.PI) * Math.sign(steerTargetDeg);
    }
    appState.robotSpeed = appState.simVel;

    // Understeer and lateral slip. At lateral acceleration a_lat = v^2 / R the
    // tyres slip: the car turns less than the geometry says and the body slides
    // outward, so the direction of travel lags the heading.
    let travelHeadingDeg = null;
    if (realism.enabled && omega !== 0 && Math.abs(appState.simVel) > 0.02) {
      const omegaRad = (omega * Math.PI) / 180;
      const radiusAbs = Math.abs(appState.simVel / omegaRad);
      const aLat = Math.abs((appState.simVel * appState.simVel) / Math.max(radiusAbs, 0.02));
      const loss = 1 + (Number(realism.gripLossK) || 0) * aLat;
      omega = omega / loss;
      const slipDeg =
        ((Number(realism.slipLateralK) || 0) * aLat * Math.sign(omega) * 180) / Math.PI;
      // The direction of travel lags the heading by the slip angle: the body
      // runs wide in the corner instead of tracking the front wheels.
      travelHeadingDeg = slipDeg;
    }

    const nextHeading = (appState.simPose.headingDeg + omega * dt) % 360;
    const rad = ((nextHeading - (travelHeadingDeg || 0)) * Math.PI) / 180;
    const nextPose = {
      x: appState.simPose.x + appState.simVel * dt * Math.cos(rad),
      y: appState.simPose.y + appState.simVel * dt * Math.sin(rad),
      headingDeg: nextHeading,
    };

    const collisionType = collidesWithAnything(nextPose);
    if (collisionType) {
      // A car that clips a wall at a shallow angle scrapes along it; blocking the
      // whole step made it stick to the wall and stay there, pushing, which is
      // what "ficou preso na parede" looked like. Keep whichever axis is free and
      // report only the speed into the wall: a rub at 20 cm/s along the wall is
      // not a 20 cm/s impact.
      const radNow = (nextHeading * Math.PI) / 180;
      const cur = appState.simPose;
      // Rotation and translation are separate freedoms. A real chassis pivots on
      // its wheels with the bumper against a wall; when the turn only exists as
      // part of a rejected step, the car can never face out of a corner and stays
      // pinned - the exports show the pose frozen to a hundredth of a millimetre
      // for the last seconds of a round while the firmware keeps commanding.
      const turnOnly = { x: cur.x, y: cur.y, headingDeg: nextHeading };
      const canTurn = Math.abs(omega) > 1e-6 && !collidesWithAnything(turnOnly);
      const slideX = { x: nextPose.x, y: cur.y, headingDeg: nextHeading };
      const slideY = { x: cur.x, y: nextPose.y, headingDeg: nextHeading };
      let normalSpeed = Math.abs(appState.simVel);
      let slid = null;
      if (!collidesWithAnything(slideX)) {
        slid = slideX;
        normalSpeed = Math.abs(appState.simVel * Math.sin(radNow));
      } else if (!collidesWithAnything(slideY)) {
        slid = slideY;
        normalSpeed = Math.abs(appState.simVel * Math.cos(radNow));
      }
      appState.collisionEvent = {
        type: collisionType,
        speed: normalSpeed,
        at: appState.simClockMs || 0,
      };
      if (slid) {
        appState.simVel *= 0.75;
        appState.simPose = slid;
        updateLatestPose({ ...appState.simPose });
        noteCollisionOnce(`Collision with ${collisionType}. Sliding along it.`);
        return;
      }
      appState.simVel = 0;
      appState.robotSpeed = 0;
      if (canTurn) {
        appState.simPose = turnOnly;
        updateLatestPose({ ...turnOnly });
        noteCollisionOnce(`Collision with ${collisionType}. Rotating in place.`);
        return;
      }
      // Nothing fits and the car is inside the geometry: walk it out along the
      // shortest free direction (bounded to 8 mm) instead of pinning it there for
      // the rest of the round.
      if (collidesWithAnything(cur)) {
        const free = unstick(cur, nextHeading);
        if (free) {
          appState.simPose = free;
          updateLatestPose({ ...free });
          noteCollisionOnce(`Collision with ${collisionType}. A desencravar.`);
          return;
        }
      }
      updateLatestPose({ ...cur });
      noteCollisionOnce(`Collision with ${collisionType}. Movement blocked.`);
      return;
    }

    // Scale independent QA counters: an outside observer polling the pose takes
    // 4x bigger steps at 4x, which by itself makes the path look smoother at 1x.
    // Counting here, at the physics rate, removes that bias.
    const dh = ((nextHeading - appState.simPose.headingDeg + 540) % 360) - 180;
    const st = appState.simStats || (appState.simStats = {
      steps: 0, reversals: 0, path: 0, turn: 0, lastSign: 0,
    });
    st.steps += 1;
    st.path += Math.abs(appState.simVel) * dt;
    st.turn += Math.abs(dh);
    if (Math.abs(dh) > 0.05) {
      const sign = dh > 0 ? 1 : -1;
      if (st.lastSign && sign !== st.lastSign) st.reversals += 1;
      st.lastSign = sign;
    }

    appState.simPose = nextPose;
    updateLatestPose({ ...appState.simPose });
  }

  function resetToParking() {
    stopSimulator();
    const startInfo = pickSimStartPose(null);
    appState.simPose = startInfo.pose;
    appState.simVel = 0;
    appState.simTime = 0;
    updateLatestPose({ ...appState.simPose });
    markRaceStart(appState.simPose);
    log("Reset to parking spot (WS connect).");
  }

  function handleKey(e, down) {
    const k = e.key.toLowerCase();
    if (k === "w" || k === "a" || k === "s" || k === "d") {
      appState.keyState[k] = down;
      if (down) e.preventDefault();
    }
  }

  function ensureSimulatorForPlay(forceStop) {
    if (forceStop && appState.simRunning) {
      stopSimulator();
      return;
    }
    if (appState.simRunning) return;
    const candidate = appState.latestPose ? { ...appState.latestPose } : null;
    startSimulator(candidate);
  }

  return {
    toggleSimulator,
    stopSimulator,
    applySimScale,
    ensureSimulatorForPlay,
    setLockStep,
    stepTo,
    handleKey,
    applyExternalControl(control) {
      const atSimMs = Number(control.atSimMs);
      const velocity = Number(control.speed ?? control.velocity ?? control.v ?? 0);
      const steeringRadius = Number(control.steeringRadius ?? control.steering_radius);
      const omega = Number(control.omegaDegPerSec ?? control.omega ?? control.turnRateDegPerSec);
      queueControl(
        Number.isFinite(velocity) ? velocity : 0,
        Number.isFinite(steeringRadius) && Math.abs(steeringRadius) > 1e-6 ? steeringRadius : Infinity,
        Number.isFinite(omega) ? omega : null,
        Number.isFinite(atSimMs) ? atSimMs : undefined,
      );
      appState.wsControlLatest = {
        velocity: Number.isFinite(velocity) ? velocity : 0,
        // Keep the geometric radius; the yaw rate is derived every tick from the
        // live speed so a speed change mid-corner bends the arc correctly.
        steeringRadius: Number.isFinite(steeringRadius) && Math.abs(steeringRadius) > 1e-6
          ? steeringRadius
          : null,
        omegaDegPerSec: Number.isFinite(omega) ? omega : 0,
      };
      appState.wsControlUpdatedAt = Date.now();
      if (!appState.simRunning) {
        startSimulator(appState.latestPose);
      }
    },
    clearExternalControl() {
      appState.wsControl = null;
    },
    resetToParking,
  };
}

