// WRO Future Engineers 2026 - Obstacle Challenge scoring.
//
// Transcribed from the official game rules 2026, section 10 (max 62 points,
// 77 with the Surprise Rule):
//
//   1.1  1 pt  section driven in the challenge direction         max 24
//   1.2  1 pt  full lap (8 sections, counted on leaving the last corner)  max 3
//   1.3  3 pts stopped in the finish section after three laps    max 3
//   1.4  2 pts one or more signs moved   (only if < 3 laps, needs >= 1 lap)
//   1.5  4 pts no sign moved             (only if < 3 laps, needs >= 1 lap)
//   1.6  8 pts one or more signs moved   (after 3 laps)
//   1.7 10 pts no sign moved             (after 3 laps)
//   1.8.1 7 pts started inside the parking lot and completed >= 1 lap
//   1.8.2 15 pts parked inside and parallel
//   1.8.3  7 pts parked partly or not parallel
//
// Contact rules that the world must enforce:
//   9.18 moving a wall zeroes the round (touching is allowed);
//   touching a magenta parking element ends the round (coach's call after the
//   2026 clarification: the car may not lean on the parking lot);
//   9.20 touching/moving a sign INSIDE its 85 mm seat circle is free (13.14);
//   9.24.6 pushing a sign out of that circle ends the round (and costs 1.4/1.6);
//   9.22 signs only count during the three official laps.
//
// Max round time is 180 s and ties are broken by time.

import { config } from "./config.js";
import { appState } from "./appState.js";
import {
  trafficSignCollision,
  getParkingSpot,
  bodyCentre,
  getRobotHalfDims,
  robotOutline,
  resetSignDisplacements,
} from "./geometry.js";

const RING_CCW = ["bottom", "c_br", "right", "c_tr", "top", "c_tl", "left", "c_bl"];
const RING_CW = ["bottom", "c_bl", "left", "c_tl", "top", "c_tr", "right", "c_br"];

const SECTION_LABELS = {
  bottom: "straight bottom",
  right: "straight right",
  top: "straight top",
  left: "straight left",
  c_bl: "corner bottom-left",
  c_br: "corner bottom-right",
  c_tr: "corner top-right",
  c_tl: "corner top-left",
};

export function sectionAt(x, y) {
  const cx = config.arenaWidthMeters / 2;
  const cy = config.arenaHeightMeters / 2;
  const half = config.internalSquareSizeMeters / 2;
  const inX = x > cx - half && x < cx + half;
  const inY = y > cy - half && y < cy + half;
  if (inX && y <= cy - half) return "bottom";
  if (inX && y >= cy + half) return "top";
  if (inY && x >= cx + half) return "right";
  if (inY && x <= cx - half) return "left";
  if (x < cx - half && y < cy - half) return "c_bl";
  if (x > cx + half && y < cy - half) return "c_br";
  if (x > cx + half && y > cy + half) return "c_tr";
  if (x < cx - half && y > cy + half) return "c_tl";
  return null;
}

function robotCorners(pose) {
  const { halfL, halfW } = getRobotHalfDims();
  // The pose is the odometry origin, which on this car sits under the rear axle
  // and not at the body centre. Building the box around the pose itself put it
  // `rearAxleToCentreMeters` behind where the physics, the drawing and the
  // collision tests put it: with the car parked 1 cm from the wall (the real
  // start) the scoring box stuck out through the wall and the round never
  // earned rule 1.8.1, even though the car had started inside the bay.
  const centre = bodyCentre(pose);
  const rad = (pose.headingDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  return [
    { x: centre.x + cos * halfL - sin * halfW, y: centre.y + sin * halfL + cos * halfW },
    { x: centre.x + cos * halfL + sin * halfW, y: centre.y + sin * halfL - cos * halfW },
    { x: centre.x - cos * halfL + sin * halfW, y: centre.y - sin * halfL - cos * halfW },
    { x: centre.x - cos * halfL - sin * halfW, y: centre.y - sin * halfL + cos * halfW },
  ];
}

function insideRect(pose, rect, tol = 0) {
  // The whole footprint, nose stick included: it is part of the measured length
  // the parking rules talk about.
  return robotOutline(pose).every(
    (c) =>
      c.x >= rect.leftX - tol &&
      c.x <= rect.rightX + tol &&
      c.y >= rect.y0 - tol &&
      c.y <= rect.y1 + tol
  );
}

function centreInside(pose, rect) {
  const c = bodyCentre(pose);
  return c.x > rect.leftX && c.x < rect.rightX && c.y > rect.y0 && c.y < rect.y1;
}

function angleDiffDeg(a, b) {
  let d = ((a - b) % 360 + 540) % 360 - 180;
  return d;
}

// "Parallel" per the rules is about the wheels, not the heading: the two wheels
// on one side sit `wheelbaseM` apart along the car's axis, so their distances to
// the wall differ by wheelbase * |sin(heading)| and the rule allows at most 2 cm
// (appendix A.6). With the fixed 70 mm wheelbase that is up to ~16,6 deg.
function wheelWallDiff(headingDeg) {
  const wheelbase = config.race.wheelbaseM ?? 0.07;
  return Math.abs(Math.sin((angleDiffDeg(headingDeg, 0) * Math.PI) / 180)) * wheelbase;
}

export function createRace({ log = () => {}, onUpdate = () => {}, onFinish = () => {} } = {}) {
  const cfg = config.race;

  const state = {
    phase: "idle", // idle | armed | running | finished
    direction: null, // "ccw" | "cw"
    section: null,
    sectionsDone: 0,
    laps: 0,
    lapTimesMs: [],
    startedAt: null,
    elapsedMs: 0,
    distanceMeters: 0,
    signsMoved: false,
    wallMoved: false,
    signWrongSide: false, // a sign taken on the wrong side ends the round
    surprisePasses: 0, // Surprise Rule: successful modified-sign passages (max 3)
    wallHits: 0,
    startInParking: false,
    finishedInSection: false,
    parking: null, // "1.8.2" | "1.8.3" | null
    timedOut: false,
    score: 0,
    breakdown: [],
    lastStillAt: null,
    bayPose: null, // last pose the car held inside the bay after the last lap
    baySince: null, // last time it was seen settled there
    bayMs: 0, // cumulative time settled inside the bay, after the last lap
    topSpeed: 0,
  };

  let timer = null;
  let lastPose = null;
  let lastCollisionAt = 0;

  function ring() {
    return state.direction === "cw" ? RING_CW : RING_CCW;
  }

  function scoreNow() {
    const breakdown = [];
    const add = (code, label, points) => {
      if (points) breakdown.push({ code, label, points });
    };

    add("1.1", `sections in the right direction (${state.sectionsDone})`, Math.min(24, state.sectionsDone));
    add("1.2", `full laps (${state.laps})`, Math.min(cfg.laps, state.laps));
    if (state.finishedInSection && state.laps >= cfg.laps) {
      add("1.3", "stopped in the finish section after 3 laps", 3);
    }
    if (state.laps >= cfg.laps) {
      add(
        state.signsMoved ? "1.6" : "1.7",
        state.signsMoved ? "signs moved (after 3 laps)" : "signs not moved (after 3 laps)",
        state.signsMoved ? 8 : 10
      );
    } else if (state.laps >= 1) {
      add(
        state.signsMoved ? "1.4" : "1.5",
        state.signsMoved ? "signs moved (without 3 laps)" : "signs not moved (without 3 laps)",
        state.signsMoved ? 2 : 4
      );
    }
    if (state.startInParking && state.laps >= 1) {
      add("1.8.1", "started inside the parking lot", 7);
    }
    if (state.parking === "1.8.2") {
      add("1.8.2", "parked complete and parallel", 15);
    } else if (state.parking === "1.8.3") {
      add("1.8.3", "parked partly or not parallel", 7);
    }
    add("SR", `modified sign passages (${state.surprisePasses}/3)`, Math.min(3, state.surprisePasses) * 5);

    state.breakdown = breakdown;
    const raw = breakdown.reduce((sum, item) => sum + item.points, 0);
    state.score = state.wallMoved ? 0 : raw;
    return state.score;
  }

  function emit() {
    // Score as it stands, not only at the final whistle: the panel is useless
    // during a run if the points only appear when it ends.
    scoreNow();
    onUpdate({
      phase: state.phase,
      direction: state.direction,
      section: state.section,
      sectionLabel: state.section ? SECTION_LABELS[state.section] : "-",
      sectionsDone: state.sectionsDone,
      laps: state.laps,
      elapsedMs: state.elapsedMs,
      score: state.score,
      signsMoved: state.signsMoved,
      signWrongSide: state.signWrongSide,
      wallMoved: state.wallMoved,
      parking: state.parking,
      startInParking: state.startInParking,
      surprisePasses: state.surprisePasses,
      ghostError: appState.ghostErrorMeters,
      timedOut: state.timedOut,
    });
  }

  function reset({ log_it = true } = {}) {
    state.phase = "idle";
    state.direction = null;
    state.section = null;
    state.sectionsDone = 0;
    state.laps = 0;
    state.lapTimesMs = [];
    state.startedAt = null;
    state.elapsedMs = 0;
    state.distanceMeters = 0;
    state.signsMoved = false;
    state.wallMoved = false;
    state.signWrongSide = false;
    state.surprisePasses = 0;
    state.lastSignPassageCount = 0;
    state.wallHits = 0;
    state.startInParking = false;
    state.finishedInSection = false;
    state.parking = null;
    state.timedOut = false;
    state.score = 0;
    state.breakdown = [];
    state.lastStillAt = null;
    state.bayPose = null;
    state.baySince = null;
    state.bayMs = 0;
    state.lastTickAt = undefined;
    // The camera/sign record belongs to the round, not to the session.
    appState.odometrySamples = [];
    appState.nanRangeWarned = false;
    appState.signSightings.length = 0;
    appState.signPassages.length = 0;
    appState.signHits.length = 0;
    state.lastSignHitAt = {};
    appState.signPassedAt = {};
    appState.signPassedVisit = {};
    appState.signPending = {};
    appState.laneVisits = {};
    appState.laneInside = {};
    appState.signSightEpoch = (appState.signSightEpoch || 0) + 1;
    resetSignDisplacements();
    state.topSpeed = 0;
    lastPose = null;
    if (log_it) log("Round restarted - waiting for movement.");
    emit();
  }

  function start(pose) {
    reset({ log_it: false });
    state.phase = "armed";
    const spot = getParkingSpot();
    if (spot && pose && insideRect(pose, spot, 0.01)) {
      state.startInParking = true;
    }
    log(
      `Round armed (${state.startInParking ? "inside the parking lot" : "outside the parking lot"}); the timer starts on the first movement.`
    );
    emit();
  }

  function ensureDirection(section) {
    if (state.direction || !section) return;
    if (section === "c_br" || section === "right") {
      state.direction = "ccw";
    } else if (section === "c_bl" || section === "left") {
      state.direction = "cw";
    }
    if (state.direction) {
      log(`Direction detected: ${state.direction === "ccw" ? "counter-clockwise" : "clockwise"}.`);
    }
  }

  function onSectionChange(next) {
    const prev = state.section;
    state.section = next;
    if (!next) return;
    ensureDirection(next);
    if (!state.direction || !prev) return;

    const order = ring();
    const prevIndex = order.indexOf(prev);
    const nextIndex = order.indexOf(next);
    if (prevIndex < 0 || nextIndex < 0) return;

    const expected = (prevIndex + 1) % order.length;
    if (nextIndex === expected) {
      state.sectionsDone += 1;
      // A lap completes when the car leaves the last corner section, which is
      // exactly the moment it re-enters the starting straight.
      if (nextIndex === 0 && state.sectionsDone >= cfg.sectionsPerLap * (state.laps + 1)) {
        state.laps += 1;
        state.lapTimesMs.push(Math.round(state.elapsedMs));
        log(
          state.laps >= cfg.laps
            ? `Lap ${state.laps}/${cfg.laps} - three full laps, heading to the parking lot.`
            : `Lap ${state.laps}/${cfg.laps} complete.`
        );
      }
    } else if (nextIndex === prevIndex) {
      // no-op
    } else {
      log(`Section out of order: ${prev} -> ${next} (does not count).`);
    }
  }

  function finalize(reason) {
    if (state.phase === "finished") return;
    state.phase = "finished";
    const pose = state.bayPose || appState.latestPose;
    if (pose) {
      state.finishedInSection = sectionAt(pose.x, pose.y) === "bottom";
      const spot = getParkingSpot();
      if (spot) {
        const parallel = wheelWallDiff(pose.headingDeg) <= 0.02;
        const fully = insideRect(pose, spot, 0.02);
        if (fully && parallel) state.parking = "1.8.2";
        else if (fully || centreInside(pose, spot)) state.parking = "1.8.3";
      }
    }
    scoreNow();
    if (timer) {
      clearInterval(timer);
      timer = null;
    }
    log(
      `Round over (${reason}): ${state.score} points, ${(state.elapsedMs / 1000).toFixed(1)} s, ${state.laps} laps.`
    );
    emit();
    onFinish({ ...state, result: state.score });
  }

  function tick() {
    const pose = appState.latestPose;
    if (!pose) return;

    // Simulated milliseconds, not wall clock: at 4x the round must last 180 s of
    // race, not 180 s of watching.
    const now = appState.simClockMs || 0;
    const tickMs = state.lastTickAt === undefined ? 0 : Math.max(0, now - state.lastTickAt);
    state.lastTickAt = now;
    const speed = Math.abs(Number(appState.robotSpeed) || 0);
    state.topSpeed = Math.max(state.topSpeed, speed);

    if (state.phase === "armed" && speed > 0.01) {
      state.phase = "running";
      state.startedAt = now;
      log("Timer running.");
    }

    if (state.phase === "running") {
      // The simulated clock restarts at zero whenever the world is restarted
      // (the repeat button does it with the car still rolling), and a start
      // instant captured before that restart makes the round clock run
      // backwards: elapsed showed -92 s at 4x on a repeat.
      if (state.startedAt === null || state.startedAt > now) state.startedAt = now;
      state.elapsedMs = now - state.startedAt;
      if (lastPose) {
        state.distanceMeters += Math.hypot(pose.x - lastPose.x, pose.y - lastPose.y);
      }

      const section = sectionAt(pose.x, pose.y);
      if (section !== state.section) onSectionChange(section);

      // Signs only count during the three official laps (rule 9.22), but
      // pushing one out of its circle ends the round at any time (9.24.6).
      const touched = trafficSignCollision(pose);
      if (touched) {
        // While pushed, the sign rides with the car and keeps the displacement
        // (rule 13.14: a sign off its seat but inside the 85 mm circle is fine).
        if (lastPose) {
          touched.dx = (touched.dx || 0) + (pose.x - lastPose.x);
          touched.dy = (touched.dy || 0) + (pose.y - lastPose.y);
        }

        const now2 = appState.simClockMs || 0;
        const key = `${touched.lane}|${touched.x}|${touched.y}`;
        const lastHit = state.lastSignHitAt || {};
        // One entry per contact, not per frame: the body stays on the sign for
        // a few hundred ms while it pushes past it.
        if (lastHit[key] === undefined || now2 - lastHit[key] > 1000) {
          state.lastSignHitAt = lastHit;
          lastHit[key] = now2;
          appState.signHits.push({
            t_ms: Math.round(now2),
            lane: touched.lane,
            colour: touched.state === 1 ? "green" : "red",
            x_cm: +(touched.x * 100).toFixed(1),
            y_cm: +(touched.y * 100).toFixed(1),
            car_x_cm: +(pose.x * 100).toFixed(1),
            car_y_cm: +(pose.y * 100).toFixed(1),
          });
        }

        // "Moved" is leaving the seat circle (13.14/13.15), not touching. A
        // touch inside the circle costs nothing (9.20).
        const radius = config.signSeatCircleDiameterMeters / 2;
        if (Math.hypot(touched.dx, touched.dy) > radius) {
          state.signsMoved = true;
          log("Sign moved out of the circle - round over.");
          finalize("sign moved out of the circle");
          return;
        }
      }

      // Taking a traffic sign on the wrong side ends the round. The firmware
      // documents its own line (green on the left, red on the right) and its
      // camera decides which one it drives; a pass against that line ends the
      // attempt instead of being a note in a log.
      const passages = appState.signPassages || [];
      const seenPassages = state.lastSignPassageCount || 0;
      if (passages.length > seenPassages) {
        for (let i = seenPassages; i < passages.length; i += 1) {
          const passage = passages[i];
          // Surprise Rule: +5 per successful passage of the modified sign (max 3).
          if (passage.modified && passage.on_intended_side === true && state.surprisePasses < 3) {
            state.surprisePasses += 1;
            log(`Modified sign passed on the correct side (+5, ${state.surprisePasses}/3).`);
          }
          if (passage.on_intended_side !== false) continue;
          state.lastSignPassageCount = passages.length;
          state.signWrongSide = true;
          log(
            `Sign ${passage.colour} in ${passage.lane} passed on the wrong side ` +
              `(${passage.side}, the line is ${passage.intended_side}) - round over.`
          );
          finalize("sign passed on the wrong side");
          return;
        }
        state.lastSignPassageCount = passages.length;
      }

      // Wall contact: blocking is the user's choice, but a hit with speed is
      // what would have moved a wall on the real field (rule 9.18).
      const hit = appState.collisionEvent;
      if (hit && hit.at > lastCollisionAt) {
        lastCollisionAt = hit.at;
        if (hit.type.includes("wall")) {
          state.wallHits += 1;
          // The magenta elements end the round on contact, at any speed: the
          // car may not lean on the parking lot at all. The black walls keep
          // the old behaviour - rubbing them costs the hit, not the round.
          if (hit.type === "parking wall") {
            log(
              `Touch on the magenta wall at ${(hit.speed * 100).toFixed(0)} cm/s - round over.`,
            );
            finalize("touch on the magenta wall");
            return;
          }
          if (hit.speed > cfg.wallMovedSpeedMetersPerSec) {
            state.wallMoved = true;
            log("Wall moved: round score zeroed (rule 9.18).");
            finalize("wall moved");
            return;
          }
          log(`Contact with ${hit.type} at ${(hit.speed * 100).toFixed(0)} cm/s (allowed).`);
        }
      }

      if (state.elapsedMs / 1000 >= cfg.maxSeconds) {
        state.timedOut = true;
        finalize("time limit");
        return;
      }

      // Remember where the car actually settled inside the bay. Its parking
      // manoeuvre oscillates: it enters, reverses, nudges, and never holds still
      // for the full 5.5 s, so the round used to reach the time limit with the
      // car back out on the field and no parking score at all.
      if (state.laps >= cfg.laps) {
        const baySpot = getParkingSpot();
        // centreInside, not insideRect: the car really does park at a small angle
        // with a corner over the line, and demanding all four corners inside for
        // 5.5 s meant the round never saw it as parked at all. "Fully inside" is
        // still tested at the end, for the 1.8.2 / 1.8.3 split.
        // "Stopped" means stopped: the same 1,2 cm/s threshold the finish-area
        // rule uses, not 10 cm/s. Creeping at 10 cm/s is the parking shuffle, not
        // a park.
        const stopped = speed < cfg.stopSpeedMetersPerSec;
        const settled = baySpot && stopped && centreInside(pose, baySpot);
        if (settled) {
          state.bayPose = { x: pose.x, y: pose.y, headingDeg: pose.headingDeg };
          // Only a car that is actually parked - the whole footprint inside the
          // rectangle and squared up with the wall - may end the round from here.
          const fully = insideRect(pose, baySpot, 0.02);
          if (fully && wheelWallDiff(pose.headingDeg) <= 0.02) {
            if (state.baySince === null) state.baySince = now;
          } else {
            state.baySince = null;
          }
        } else {
          state.baySince = null;
        }
        // A running hold, not an accumulator. The parallel-parking shuffle stops
        // for ~0,5 s between every step, and summing those pauses declared the
        // car parked mid-manoeuvre (round-...-100414: "parked" at 110.3 s
        // with the car 90 deg off parallel and still doing 8 cm/s). Only a car
        // that stays put for parkHoldSeconds ends the round; the pose it settled
        // in is still remembered above, so a timeout scores the best attempt.
        state.bayMs = state.baySince === null ? 0 : now - state.baySince;
        if (state.bayMs >= cfg.parkHoldSeconds * 1000) {
          finalize("parked");
          return;
        }
      }

      // A stop only ends the round inside the parking bay. The rule is about the
      // car finishing in the finish area, not about any pause: a firmware that
      // waits five seconds at a waypoint, reverses to line up, or stops to take a
      // decision was getting its run cut short in the middle of the field.
      if (speed < cfg.stopSpeedMetersPerSec && state.distanceMeters > 0.6) {
        const spot = getParkingSpot();
        const inBay =
          pose.x >= spot.leftX - 0.05 &&
          pose.x <= spot.rightX + 0.05 &&
          pose.y >= spot.y0 - 0.05 &&
          pose.y <= spot.y1 + 0.10;
        if (!inBay) {
          state.lastStillAt = null;
        } else if (state.lastStillAt === null) {
          state.lastStillAt = now;
        } else if (now - state.lastStillAt >= cfg.stopHoldSeconds * 1000) {
          finalize("stopped");
          return;
        }
      } else {
        state.lastStillAt = null;
      }
    }

    lastPose = { x: pose.x, y: pose.y };
    emit();
  }

  function begin(pose = appState.latestPose) {
    start(pose);
    if (!timer) timer = setInterval(tick, 50);
  }

  function stop() {
    if (timer) {
      clearInterval(timer);
      timer = null;
    }
  }

  function result() {
    return {
      score: state.score,
      breakdown: state.breakdown,
      elapsedMs: state.elapsedMs,
      laps: state.laps,
      lapTimesMs: state.lapTimesMs.slice(),
      sectionsDone: state.sectionsDone,
      direction: state.direction,
      signsMoved: state.signsMoved,
      signWrongSide: state.signWrongSide,
      wallMoved: state.wallMoved,
      wallHits: state.wallHits,
      parking: state.parking,
      startInParking: state.startInParking,
      surprisePasses: state.surprisePasses,
      timedOut: state.timedOut,
      distanceMeters: Number(state.distanceMeters.toFixed(2)),
      ghostError: appState.ghostErrorMeters,
    };
  }

  return { reset, begin, stop, tick, result, getState: () => state };
}
