import { trafficSigns } from "./traffic.js";
import { getSignHalfSize } from "./geometry.js";
import { collidesWithAnything } from "./geometry.js";
import { appState } from "./appState.js";
import { setPoseFromUser, formatSignedDeg } from "./pose.js";
import { rngNext } from "./rng.js";

export function createSignControls({ worldToScreen, screenToWorld, log }) {
  function toggleSignAtClick(e) {
    const rect = e.target.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const half = getSignHalfSize() * appState.camera.scale;
    let hitIndex = -1;
    for (let i = 0; i < trafficSigns.length; i++) {
      const p = worldToScreen(trafficSigns[i].x, trafficSigns[i].y);
      const dx = Math.abs(p.x - px);
      const dy = Math.abs(p.y - py);
      if (dx <= half && dy <= half) {
        hitIndex = i;
        break;
      }
    }
    if (hitIndex >= 0) {
      trafficSigns[hitIndex].state = (trafficSigns[hitIndex].state + 1) % 3;
      return true;
    }
    return false;
  }

  function handleTestLeftClick(e) {
    const toggled = toggleSignAtClick(e);
    if (toggled) return;

    const rect = e.target.getBoundingClientRect();
    const world = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
    const headingDeg = appState.latestPose?.headingDeg ?? 0;
    const candidate = { x: world.x, y: world.y, headingDeg };
    const collisionType = collidesWithAnything(candidate);
    if (collisionType) {
      log(`Cannot place robot here: collides with ${collisionType}.`);
      return;
    }
    setPoseFromUser(candidate);
    log(`Placed robot at ${(world.x * 100).toFixed(1)} cm, ${(world.y * 100).toFixed(1)} cm in Test mode.`);
  }

  function handleTestRightClick(e) {
    const rect = e.target.getBoundingClientRect();
    const world = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
    if (!appState.latestPose) {
      handleTestLeftClick(e);
      return;
    }
    const dx = world.x - appState.latestPose.x;
    const dy = world.y - appState.latestPose.y;
    if (Math.abs(dx) < 1e-5 && Math.abs(dy) < 1e-5) return;
    const headingDeg = ((Math.atan2(dy, dx) * 180) / Math.PI + 360) % 360;
    setPoseFromUser({ ...appState.latestPose, headingDeg });
    log(`Heading set to ${formatSignedDeg(headingDeg)} toward ${(world.x * 100).toFixed(1)} cm, ${(world.y * 100).toFixed(1)} cm.`);
  }

  return { toggleSignAtClick, handleTestLeftClick, handleTestRightClick };
}

export function randomizeSigns() {
  const rand = rngNext;
  for (const s of trafficSigns) {
    s.state = 0;
    s.modified = false;
    s.heightCm = 10;
  }
  const lanes = ["bottom", "top", "left", "right"];
  const minSeparation = 0.5;
  const singleLane = lanes[Math.floor(rand() * lanes.length)];

  for (const laneName of lanes) {
    const laneSigns = trafficSigns.filter((s) => s.lane === laneName);
    if (!laneSigns.length) continue;

    const centerSigns = getLaneCenterSigns(laneName, laneSigns);
    const nonCenter = laneSigns.filter((s) => !centerSigns.includes(s));

    let chosen = [];
    const needCount = laneName === singleLane ? 1 : 2;

    if (needCount === 1) {
      if (centerSigns.length && rand() < 0.6) {
        chosen = [centerSigns[Math.floor(rand() * centerSigns.length)]];
      } else if (nonCenter.length) {
        chosen = [nonCenter[Math.floor(rand() * nonCenter.length)]];
      } else if (centerSigns.length) {
        chosen = [centerSigns[0]];
      }
    } else {
      const pool = shuffle([...nonCenter], rand);
      outer: for (let i = 0; i < pool.length; i++) {
        for (let j = i + 1; j < pool.length; j++) {
          const a = pool[i];
          const b = pool[j];
          const dist = laneName === "bottom" || laneName === "top"
            ? Math.abs(a.x - b.x)
            : Math.abs(a.y - b.y);
          if (dist >= minSeparation) {
            chosen = [a, b];
            break outer;
          }
        }
      }
      if (chosen.length < 2) {
        if (nonCenter.length >= 2) {
          chosen = [nonCenter[0], nonCenter[1]];
        } else if (nonCenter.length === 1 && centerSigns.length) {
          chosen = [nonCenter[0], centerSigns[0]];
        } else if (centerSigns.length >= 2) {
          chosen = [centerSigns[0], centerSigns[1]];
        }
      }
    }

    for (const s of chosen) {
      s.state = rand() < 0.5 ? 1 : 2;
    }
  }

  // Surprise Rule: flag one active sign as "modified" - its VISIBLE colour is
  // the opposite of its ORIGINAL colour and its height is 5 or 15 cm (instead of
  // 10). The sign keeps its ORIGINAL colour here; the camera emulation inverts
  // it when it sends the frame to the firmware.
  if (appState.surpriseRule) {
    const active = trafficSigns.filter((s) => s.state === 1 || s.state === 2);
    if (active.length) {
      const picked = active[Math.floor(rand() * active.length)];
      picked.modified = true;
      picked.heightCm = rand() < 0.5 ? 5 : 15;
    }
  }
}

function getLaneCenterSigns(lane, signs) {
  if (!signs.length) return [];
  if (lane === "bottom" || lane === "top") {
    const xs = signs.map((s) => s.x);
    const centerX = (Math.min(...xs) + Math.max(...xs)) / 2;
    return signs.filter((s) => s.x === centerX);
  }
  if (lane === "left" || lane === "right") {
    const ys = signs.map((s) => s.y);
    const centerY = (Math.min(...ys) + Math.max(...ys)) / 2;
    return signs.filter((s) => s.y === centerY);
  }
  return [];
}

function shuffle(arr, rand = Math.random) {
  for (let i = arr.length - 1; i > 0; i--) {
    const j = Math.floor(rand() * (i + 1));
    [arr[i], arr[j]] = [arr[j], arr[i]];
  }
  return arr;
}

