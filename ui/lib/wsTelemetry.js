import { appState, Modes } from "./appState.js";
import { config } from "./config.js";
import {
  detectTrafficSignsInFov,
  measurePoseDistances,
  setLatestTelemetryDistances,
} from "./pose.js";
import { rngNext } from "./rng.js";

export function createWsTelemetry({ log, simulator, url = null } = {}) {
  // Resolved lazily: config.resetConfig() runs after this module is constructed
  // and derives the endpoint from window.location.
  const endpoint = () => url || config.wsUrl || "ws://localhost:8780";
  let ws = null;
  const timers = [];

  // The sensor cadence is expressed in simulated time: at 2x the firmware's clock
  // runs twice as fast, so it has to receive twice as many samples per real
  // second or it sees a car that reacts, steers and accelerates more slowly than
  // it should. Without this, 2x and 4x "drift" even with the clock scaled.
  // Sensor cadence in SIMULATED milliseconds. Wall-clock timers cannot go below
  // the browser's ~4 ms floor, so at 4x they silently stretched to 16 simulated ms
  // and the firmware got coarser odometry than it does at 1x. The physics calls
  // pump() on every step instead and each stream fires when its simulated period
  // is due.
  const STREAMS = [
    { fn: sendEncoder, period: 10, due: 0 },
    { fn: sendHeading, period: 20, due: 0 },
    { fn: sendUltrasonic, period: 50, due: 0 },
    { fn: sendSigns, period: 100, due: 0 },
  ];

  let fallbackMs = 0;
  let lastWallMs = 0;

  function startTimers() {
    while (timers.length) clearInterval(timers.pop());
    for (const stream of STREAMS) stream.due = 0;
    // The streams are pumped by the physics, but the physics only runs once the
    // firmware asks it to - and the firmware needs sensor frames to ask. This
    // low-rate timer keeps the streams alive on wall time while the car is not
    // moving, so the first frames go out and the loop can bootstrap.
    timers.push(setInterval(() => {
      const wall = performance.now();
      if (!appState.simRunning) {
        fallbackMs += lastWallMs ? Math.min(wall - lastWallMs, 100) : 5;
        pump(fallbackMs);
      }
      lastWallMs = wall;
    }, 5));
  }

  function pump(simNow) {
    if (appState.replayMode || !Number.isFinite(simNow)) return;
    for (const stream of STREAMS) {
      if (simNow < stream.due) continue;
      stream.due = simNow + stream.period;
      try {
        stream.fn();
      } catch (err) {
        // A bad frame must never stop the physics loop.
        console.warn("telemetry stream failed", err);
      }
    }
  }
  let reconnectTimer = null;
  let stopRequested = false;
  let lastEncoderPose = null;
  let headingZeroAbs = null;
  let offlineNotified = false;

  function setConnected(on) {
    appState.wsConnected = on;
  }

  function noiseEnabled() {
    return Boolean(config.telemetryNoise?.enabled || appState.realWorldMode);
  }

  function gaussianNoise(sigma) {
    if (!Number.isFinite(sigma) || sigma <= 0) return 0;
    // Seeded, not Math.random: with an unseeded generator two runs of the same
    // round with the same seed draw different sensor noise, so a 1x run and a
    // 4x run could never be compared and "it behaved differently" was not even
    // measurable.
    const u1 = rngNext();
    const u2 = rngNext();
    const mag = Math.sqrt(-2.0 * Math.log(Math.max(u1, 1e-9)));
    return sigma * mag * Math.cos(2 * Math.PI * u2);
  }

  function maybeNoisy(value, sigma, { clampMin } = {}) {
    if (!noiseEnabled()) return value;
    if (!Number.isFinite(value)) return value;
    const noisy = value + gaussianNoise(sigma);
    if (clampMin !== undefined) return Math.max(clampMin, noisy);
    return noisy;
  }

  // A MEMS gyro has a fixed bias for the round plus a slow random walk, on top
  // of the per-sample jitter. Drawn from the seeded generator so the same seed
  // still reproduces the same round.
  let gyroBiasDeg = null;
  let gyroDriftDeg = 0;
  let gyroLastSimMs = null;
  const sonarBias = new Map();

  function gyroReading(headingRel, simMs) {
    const cfg = config.telemetryNoise || {};
    if (!noiseEnabled() || !Number.isFinite(headingRel)) return headingRel;
    if (gyroBiasDeg === null) gyroBiasDeg = gaussianNoise(cfg.headingBiasDegSigma || 0);
    const dt = gyroLastSimMs === null ? 0 : Math.max(0, (simMs - gyroLastSimMs) / 1000);
    gyroLastSimMs = simMs;
    if (dt > 0) {
      gyroDriftDeg += gaussianNoise(cfg.headingDriftDegPerSqrtS || 0) * Math.sqrt(dt);
    }
    return headingRel + gyroBiasDeg + gyroDriftDeg + gaussianNoise(cfg.headingDegSigma || 0);
  }

  function sonarOffset(label) {
    const cfg = config.telemetryNoise || {};
    if (!noiseEnabled()) return 0;
    if (!sonarBias.has(label)) sonarBias.set(label, gaussianNoise(cfg.sonarBiasMetersSigma || 0));
    return sonarBias.get(label);
  }

  function safeSend(payload) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    try {
      ws.send(JSON.stringify(payload));
    } catch (err) {
      log(`WS send error: ${err.message || err}`);
    }
  }

  function sendHeading() {
    const pose = appState.latestPose;
    if (!pose) return;
    if (headingZeroAbs === null) headingZeroAbs = pose.headingDeg;
    const headingRel = pose.headingDeg - headingZeroAbs;
    const headingDeg = gyroReading(headingRel, appState.simClockMs || 0);
    safeSend({
      type: "heading",
      headingDeg,
      headingDegAbs: pose.headingDeg,
      // The firmware's clock is sped up by the same factor, so 2x is a time
      // lapse of the same race and not a car twice as fast.
      scale: appState.simScale || 1,
      simClockMs: appState.simClockMs || 0,
      timestampMs: Date.now(),
    });
  }

  function sendEncoder() {
    const pose = appState.latestPose;
    if (!pose) {
      lastEncoderPose = null;
      return;
    }
    if (!lastEncoderPose) {
      lastEncoderPose = { ...pose };
      return;
    }
    const dx = pose.x - lastEncoderPose.x;
    const dy = pose.y - lastEncoderPose.y;
    const headingRad = Number.isFinite(pose.headingDeg) ? (pose.headingDeg * Math.PI) / 180 : null;
    // Project displacement onto the heading vector to preserve forward/backward sign.
    const projected = headingRad === null ? null : dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
    lastEncoderPose = { ...pose };
    if (!Number.isFinite(projected) || Math.abs(projected) <= 1e-6) return;
    const distNoisy = maybeNoisy(projected, config.telemetryNoise.encoderMetersSigma);
    safeSend({
      type: "encoder",
      distance: distNoisy,
      // Every frame carries the multiplier: the controller forwards it to the
      // firmware, which must arrive at the right clock speed even when its
      // process restarted mid round.
      scale: appState.simScale || 1,
      timestampMs: Date.now(),
    });
  }

  function sendUltrasonic() {
    if (appState.currentMode !== Modes.PLAY && appState.currentMode !== Modes.TELEMETRY) return;
    const pose = appState.latestPose;
    if (!pose) return;
    const measurements = measurePoseDistances(pose);
    // Never put a non-finite range on the wire: the firmware maps NaN to -1
    // ("no echo"), and a boot that reads -1 on all four channels builds its start
    // geometry from nothing and picks the driving direction from a comparison of
    // two sentinels. When this fires, the ray casting produced a NaN - log it
    // once per round with the pose, because that is the only way to see which
    // geometry went bad.
    if (measurements.some((entry) => !Number.isFinite(entry.distance))) {
      if (!appState.nanRangeWarned) {
        appState.nanRangeWarned = true;
        log(
          `Ranges NaN: ${measurements
            .map((entry) => `${entry.label}=${entry.distance}`)
            .join(" ")} para=((${(pose.x * 100).toFixed(1)},${(pose.y * 100).toFixed(1)}) h=${pose.headingDeg.toFixed(0)})`,
        );
      }
      return;
    }
    for (const entry of measurements) {
      if (!Number.isFinite(entry.distance)) entry.distance = 0;
    }
    // A beam whose origin sits inside a wall (the car is brushing the bay wall
    // while it manoeuvres) hits nothing and is dropped from the list. Sending
    // nothing means "no echo" = -1 to the firmware, which then believes it is
    // blind and pushes harder. Touching is the honest reading: zero.
    const seen = new Set(measurements.map((entry) => entry.label));
    const distances = measurements.reduce((acc, entry) => {
      const noisy = maybeNoisy(entry.distance + sonarOffset(entry.label),
                              config.telemetryNoise.distanceMetersSigma, { clampMin: 0 });
      acc[entry.label] = noisy;
      return acc;
    }, {});
    for (const label of ["Front", "Rear", "Left", "Right"]) {
      if (!seen.has(label)) distances[label] = 0;
    }
    setLatestTelemetryDistances(distances);
    safeSend({
      type: "ultrasonic",
      distances,
      scale: appState.simScale || 1,
      timestampMs: Date.now(),
    });
  }

  // Keys of the signs visible in the previous frame, so a sign is logged once
  // when it enters the field of view instead of once per 10 ms sample. The
  // epoch is bumped whenever a round is reset, so the first frame of a new
  // round starts from an empty board even if the camera never lost the sign.
  let lastSignKeys = new Set();
  let signEpoch = appState.signSightEpoch || 0;

  function recordSignSightings(signs, pose) {
    if ((appState.signSightEpoch || 0) !== signEpoch) {
      signEpoch = appState.signSightEpoch || 0;
      lastSignKeys = new Set();
    }
    const keys = new Set(signs.map((s) => `${s.lane}|${s.state}`));
    for (const sign of signs) {
      const key = `${sign.lane}|${sign.state}`;
      if (lastSignKeys.has(key)) continue;
      appState.signSightings.push({
        t_ms: Math.round(appState.simClockMs || 0),
        lane: sign.lane,
        colour: sign.state === 1 ? "green" : "red",
        distance_cm: +(sign.distance * 100).toFixed(1),
        angle_deg: +sign.angleDeg.toFixed(1),
        x_cm: +(sign.x * 100).toFixed(1),
        y_cm: +(sign.y * 100).toFixed(1),
        car_x_cm: +(pose.x * 100).toFixed(1),
        car_y_cm: +(pose.y * 100).toFixed(1),
      });
    }
    lastSignKeys = keys;
  }

  function sendSigns() {
    const pose = appState.latestPose;
    if (!pose) return;
    // Every sign inside the camera's FOV, not only the nearest one: the
    // firmware picks the zone itself, and feeding it a pre-filtered list would
    // test a perception the real camera does not have.
    const detections = detectTrafficSignsInFov(pose);
    const signs = detections.map((detection) => ({
      lane: detection.sign.lane,
      state: detection.sign.state,
      distance: maybeNoisy(detection.distance, config.telemetryNoise.signDistanceMetersSigma, {
        clampMin: 0,
      }),
      angleDeg: maybeNoisy(detection.angleDeg, config.telemetryNoise.signAngleDegSigma),
      // Field position of the sign the camera is looking at. The firmware only
      // needs colour + bearing, but the pose is what lets an operator (or a future
      // localiser) check whether the reading is the sign it thinks it is.
      x: detection.sign.x,
      y: detection.sign.y,
    }));
    appState.detectedSigns = signs;
    recordSignSightings(signs, pose);
    safeSend({ type: "signs", signs, scale: appState.simScale || 1, timestampMs: Date.now() });
  }

  // The firmware keeps an absolute field position in its own 0..303 cm frame
  // (state.set_relative_odom writes it): for a clockwise run the x axis is
  // mirrored around the outer wall. Anchor the heading once, at the first frame,
  // because the BNO085 yaw zero is arbitrary.
  let ghostHeadingOffset = null;

  function firmwareFrameToWorld(odom) {
    // x/y arrive in metres (the controller divides the firmware's cm by 100).
    const { x, y, headingDeg, cw } = odom;
    if (![x, y, headingDeg].every((v) => Number.isFinite(v))) return null;
    const outerWallM = 3.03; // firmware constant (State.outer_wall = 303 cm)
    const worldX = cw === 1 ? outerWallM - x : x;
    const worldY = y;
    if (ghostHeadingOffset === null && appState.latestPose) {
      ghostHeadingOffset = appState.latestPose.headingDeg - headingDeg;
    }
    const offset = ghostHeadingOffset ?? 0;
    return {
      x: worldX,
      y: worldY,
      headingDeg: ((headingDeg + offset) % 360 + 360) % 360,
    };
  }

  // The firmware pose in a control frame is one control period older than the
  // frame itself, and the frame takes a pipe + websocket hop to arrive. Pairing
  // it with "the latest world pose" therefore measured transport latency, not
  // odometry: at 50 cm/s a 200 ms age reads as 10 cm of "error" that vanishes on
  // the next sample. The replay buffer holds the world at every physics step, so
  // look the frame's own instant up instead.
  // How old the firmware's pose is when it reaches us, measured: sweeping the
  // comparison offset from 0 to 100 ms of simulated time took the worst error
  // from 12.2 cm to 6.1 cm and then flattened, and at zero speed the offset
  // makes no difference at all - which is what tells you it is stream age (one
  // telemetry period plus the pipe) and not drift.
  const POSE_STREAM_AGE_MS = 100;

  function poseAtSimClock(clockMs) {
    const frames = appState.replayFrames;
    if (!frames || !frames.length || !Number.isFinite(clockMs)) return null;
    clockMs -= POSE_STREAM_AGE_MS;
    let lo = 0;
    let hi = frames.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (frames[mid].t < clockMs) lo = mid + 1;
      else hi = mid;
    }
    const frame = frames[lo];
    if (!frame || Math.abs(frame.t - clockMs) > 250) return null;
    return { x: frame.x, y: frame.y, headingDeg: frame.h };
  }

  function handleControl(msg) {
    if (!simulator?.applyExternalControl) return;
    const speed = Number(msg.speed ?? msg.velocity ?? msg.v);
    const steeringRadius = Number(msg.steering_radius ?? msg.steeringRadius);
    const omegaDegPerSec = Number(msg.omegaDegPerSec ?? msg.omega ?? msg.turnRateDegPerSec);
    simulator.applyExternalControl({
      // The instant this command belongs to, on the simulated clock. The
      // controller stamps every frame with it; without it the command was queued
      // at the browser's current clock, which only moves once per display frame -
      // so at 4x four commands shared one timestamp and only the last survived.
      atSimMs: Number.isFinite(Number(msg.simClockMs)) ? Number(msg.simClockMs) : undefined,
      speed: Number.isFinite(speed) ? speed : 0,
      steeringRadius: Number.isFinite(steeringRadius) ? steeringRadius : null,
      omegaDegPerSec: Number.isFinite(omegaDegPerSec) ? omegaDegPerSec : undefined,
    });
    if (msg.debug && typeof msg.debug === "object") {
      appState.firmwareDistances = msg.debug.sonar || null;
      appState.firmwareCamera = msg.debug.camera || null;
    }
    if (msg.odom && typeof msg.odom === "object") {
      // "telemetry" frames come straight from the firmware's own telemetry.py,
      // which already outputs the visualiser's units and orientation. Its origin
      // is under the rear axle, so shift it to the body centre to compare with
      // the world pose.
      let pose;
      if (msg.odom.frame === "telemetry") {
        // Both the world pose and the firmware's odometry origin are under the
        // rear axle, so nothing has to be shifted: this is directly comparable.
        pose = {
          x: Number(msg.odom.x),
          y: Number(msg.odom.y),
          headingDeg: Number(msg.odom.headingDeg),
        };
      } else if (msg.odom.frame === "firmware") {
        pose = firmwareFrameToWorld(msg.odom);
      } else {
        pose = msg.odom;
      }
      if (pose) {
        const { x, y, headingDeg } = pose;
        if ([x, y, headingDeg].every((v) => Number.isFinite(v))) {
          appState.ghostRaw = { ...msg.odom };
          appState.ghostPose = { x: Number(x), y: Number(y), headingDeg: Number(headingDeg) };
          appState.ghostSpeed = Number.isFinite(speed) ? speed : 0;
          appState.ghostSteeringRadius = Number.isFinite(steeringRadius) ? steeringRadius : null;
          appState.ghostTimestampMs = Date.now();
          const truth = poseAtSimClock(msg.simClockMs) || appState.latestPose;
          appState.ghostErrorMeters = truth
            ? Math.hypot(truth.x - x, truth.y - y)
            : null;
          // Keep a series, not only the latest value: the *shape* of the drift
          // (a constant offset is the anchor, a ramp is a scale error) is what
          // says which constant to touch. Sampled on simulated time so the
          // multiplier cannot change how many samples a run carries.
          if (truth && appState.simRunning) {
            const samples = appState.odometrySamples
              || (appState.odometrySamples = []);
            const t = Math.round(appState.simClockMs || 0);
            const last = samples.length ? samples[samples.length - 1] : null;
            if (!last || t - last.t_ms >= 200) {
              const headingErr = ((Number(headingDeg) - truth.headingDeg + 540) % 360) - 180;
              samples.push({
                t_ms: t,
                err_cm: +((appState.ghostErrorMeters) * 100).toFixed(1),
                heading_err_deg: +headingErr.toFixed(1),
                sim_x_cm: +(truth.x * 100).toFixed(1),
                sim_y_cm: +(truth.y * 100).toFixed(1),
                fw_x_cm: +(x * 100).toFixed(1),
                fw_y_cm: +(y * 100).toFixed(1),
              });
            }
          }
        }
      }
    }
  }

  function handleLogMessage(msg) {
    const text = msg?.message ?? msg?.text;
    if (typeof text !== "string" || !text.length) return;
    log(text);
  }

  function handleMessage(event) {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "control") {
        handleControl(msg);
      } else if (msg.type === "log") {
        handleLogMessage(msg);
      } else if (msg.type === "lockstep") {
        // The controller owns time now: the world only advances when it asks.
        simulator?.setLockStep?.(!!msg.on);
      } else if (msg.type === "advance") {
        const stepMs = Number(msg.stepMs) || 1000 / 60;
        simulator?.stepTo?.(appState.simClockMs + stepMs, stepMs);
        // Acknowledge only after the physics has advanced: the controller feeds
        // the firmware on this reply, so a lost or early ack would let it run
        // ahead of the world.
        safeSend({ type: "stepped", simMs: appState.simClockMs });
      }
    } catch (err) {
      log(`WS parse error: ${err.message || err}`);
    }
  }

  function teardown(reconnect) {
    setConnected(false);
    lastEncoderPose = null;
    headingZeroAbs = null;
    appState.ghostPose = null;
    appState.ghostSpeed = 0;
    appState.ghostSteeringRadius = null;
    setLatestTelemetryDistances(null);
    while (timers.length) {
      clearInterval(timers.pop());
    }
    if (ws) {
      ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
      try {
        ws.close();
      } catch (_) {
        /* ignore */
      }
    }
    ws = null;
    if (reconnect && !stopRequested && !reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, 1500);
    }
  }

  function connect() {
    if (ws || stopRequested) return;
    const target = endpoint();
    try {
      ws = new WebSocket(target);
    } catch (err) {
      log(`WS connect error: ${err.message || err}`);
      teardown(true);
      return;
    }

    ws.onopen = () => {
      setConnected(true);
      offlineNotified = false;
      if (simulator?.resetToParking) simulator.resetToParking();
      log(`WS connected to ${target}`);
      startTimers();
    };
    ws.onclose = (evt) => {
      if (!offlineNotified) {
        offlineNotified = true;
        log(`WS disconnected (code ${evt?.code ?? "?"} ${evt?.reason || ""})`);
      }
      teardown(true);
    };
    ws.onerror = (evt) => {
      const err = evt?.message || "unknown error";
      // Only surface the first error while offline to avoid log spam during reconnects.
      if (!offlineNotified || ws?.readyState === WebSocket.OPEN) {
        offlineNotified = true;
        log(`WS error: ${err}`);
      }
    };
    ws.onmessage = handleMessage;
  }

  function start() {
    stopRequested = false;
    connect();
  }

  function stop() {
    stopRequested = true;
    if (reconnectTimer) clearTimeout(reconnectTimer);
    reconnectTimer = null;
    teardown(false);
  }

  function send(payload) {
    safeSend(payload);
  }

  // Called when a new round starts: the firmware boots with its IMU at zero, so
  // the world heading it is fed must be relative to the *new* pose, and the
  // encoder delta from the previous run must not leak into the new one.
  function resetOrigin() {
    headingZeroAbs = null;
    lastEncoderPose = null;
    ghostHeadingOffset = null;
    appState.ghostPose = null;
    appState.ghostErrorMeters = null;
    // The simulated clock restarts at zero on «Repetir pista» / «Nova pista», but
    // the stream due-times are expressed in simulated milliseconds and survive
    // it. Left alone they sit at ~150000 from the finished round, every stream
    // is "not due yet" for the whole next round, and the firmware starts with no
    // sonars at all (-1 on all four) - which is how a repeat ended up building
    // its start geometry out of nothing and driving into the bay wall.
    startTimers();
  }

  function setScale() {
    // The cadence is in simulated time now, so a new multiplier only needs the
    // due times reset.
    startTimers();
  }

  return { start, stop, send, resetOrigin, setScale, pump };
}


