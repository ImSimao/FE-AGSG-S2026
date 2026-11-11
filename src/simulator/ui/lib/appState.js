import { config } from "./config.js";

export const Modes = {
  TELEMETRY: "telemetry",
  PLAY: "play",
  TEST: "test",
};

export const appState = {
  latestPose: null,
  prevPose: null,
  lastMouseWorld: null,
  latestTelemetryDistances: null,
  wsControl: null,
  wsControlUpdatedAt: 0,
  wsConnected: false,
  lastCollisionMessageAt: 0,
  lastSignWarningAt: {},
  followHeading: false,
  followRotationRad: 0,
  showDistances: false,
  realWorldMode: false,
  showSignHud: false,
  simRunning: false,
  simTimer: null,
  simTime: 0,
  simPose: null,
  simVel: 0,
  steerActualDeg: 0, // angle the wheels actually have (slack + delay)
  steerContact: 0, // game side in contact: +1, -1 or 0 (within slack)
  simDt: 1 / 60,
  keyState: { w: false, a: false, s: false, d: false },
  camera: {
    scale: config.pixelsPerMeter,
    offsetX: 0,
    offsetY: 0,
    dragging: false,
    dragMoved: false,
    lastX: 0,
    lastY: 0,
  },
  robotDrag: {
    active: false,
    offsetX: 0,
    offsetY: 0,
    lastPx: 0,
    lastPy: 0,
    moved: false,
  },
  robotRotate: {
    active: false,
  },
  viewWidth: 0,
  viewHeight: 0,
  backgroundImage: null,
  defaultBgLoaded: false,
  currentMode: Modes.TELEMETRY,
  ghostPose: null,
  ghostSteeringRadius: null,
  ghostSpeed: 0,
  ghostTimestampMs: 0,
  // --- race support
  parkingSide: "right", // which end of the start section holds the bay
  raceStartPose: null, // ground-truth pose the firmware frame is anchored to
  raceStartHeading: 0,
  robotSpeed: 0, // measured world speed, m/s (signed)
  simScale: 1, // 1 = real time, 2 = double speed
  ghostErrorMeters: null,
  firmwareDistances: null, // what the firmware decoded, in cm
  firmwareCamera: null, // blobs the firmware parsed and zones it sees
  // --- race record: what the camera saw, and every sign the car actually passed.
  // A snapshot of `detectedSigns` is empty the moment the round ends (nothing is
  // in front of the camera any more), so an exported round cannot rely on it.
  signSightings: [], // one entry per sign entering the camera's field of view
  signPassages: [], // every sign driven past, with the side it was on
  signPassedAt: {}, // sign index -> sim ms of the last recorded passage
  signHits: [], // every contact with a sign: that is the rule, the side is not
  signSightEpoch: 0, // bumped on every round reset
};

export function resetPoseHistory() {
  appState.prevPose = null;
  appState.latestPose = null;
}

