export const defaultConfig = {
  // Websocket endpoint of the Python simulator. Override with ?ws=ws://host:port
  wsUrl: "ws://localhost:8780",
  arenaWidthMeters: 3,
  arenaHeightMeters: 3,
  pixelsPerMeter: 200,
  origin: "center", // bottom-left | top-left | center
  robotWidthMeters: 0.061, // 61 mm (AGSG, README)
  // The measured length the rules use for the parking lot (1,5 x). It is the
  // functional footprint within 10 cm of the mat: the 127 mm chassis plus a
  // thin 73 x 20 mm nose stick, so 200 mm and a 300 mm bay (10 cm of play in
  // the bay: 0,5 x the measured length). The body itself
  // stays 127 mm - see bodyLengthMeters, which is what carries the wheels.
  robotLengthMeters: 0.2,
  bodyLengthMeters: 0.127, // the box that carries the wheels and the sensors
  noseLengthMeters: 0.073, // stick ahead of the body, 20 mm wide, no sensors
  noseWidthMeters: 0.02,
  gridStepMeters: 0.25,
  internalSquareSizeMeters: 1, // draw a centered square of this size
  wallThicknessMeters: 0.01, // 1 cm wall thickness
  parkingLengthMeters: 0.2, // rules p.8: the lot is always 20 cm wide, out from the wall
  // Length along the wall = 1,5 x the measured length (rules p.8), and the 2026
  // Q&A measures only the part within 10 cm of the mat. With the 200 mm
  // footprint (body + nose stick) the bay is 300 mm.
  parkingGapMeters: 0.3,
  parkingGapFactor: 0, // legacy knob, unused when parkingGapMeters is set
  parkingWallThicknessMeters: 0.02, // magenta elements are 200 x 20 x 100 mm (rule 13.25)
  // Where the car sits in the bay at the start.
  //
  // The real car is parked with its rear face 1 cm from the magenta wall behind
  // it, so that is the default: the bench has to reproduce the pose the robot
  // really starts from, even when the firmware copes badly with it.
  //
  //   "wall"   - rear face 1 cm from the wall behind it (the real start)
  //   "nose"   - nose 1.75 cm from the wall ahead
  //   "centre" - middle of the bay
  //   "above"  - the middle zone over the bay (training start, loses 1.8.1)
  startMode: "wall",
  // "auto" draws the direction (rules), "cw" or "ccw" force it (?dir=ccw).
  // Default is "ccw" (car facing east in the bottom straight) because the
  // firmware's clockwise boot sequence diverges geometrically in this world.
  // Use ?dir=cw to reproduce it.
  startDirection: "cw", // anti-clockwise is not ready yet: clockwise only for now
  seed: null, // ?seed=1234 fixes the track for comparable runs
  startPose: null, // ?pose=x_cm,y_cm,heading overrides the start pose (test hook)
  // Hardware conventions pushed to the controller on the next round start:
  // ?imu=1&servo=-1 tries the mirrored pairing without restarting the server.
  imuSignParam: null,
  servoSignParam: null,
  // Sensor geometry, straight from the firmware (get_odom_side_sonar):
  // front_sonar_offset 8.97 cm, rear_sonar_offset 1.25 cm, side (7.97, 2.55) cm,
  // all measured from the odometry origin, which on the real car is under the
  // rear axle (not the body centre). rearAxleToCentreMeters converts them to the
  // simulator's pose, which is the centre of the 127 mm chassis.
  rearAxleToCentreMeters: 0.038, // rear axle is 2.55 cm from the rear face; chassis is 127 mm
  sonar: {
    frontMeters: 0.0897,
    rearMeters: 0.0125,
    sideForwardMeters: 0.0797,
    sideLateralMeters: 0.0255,
  },
  signSizeMeters: 0.05, // 5 cm square: WRO traffic sign.
  signSeatCircleDiameterMeters: 0.085, // rule 13.15: the 'moved' circle is 85 mm across
  // The camera sits at the FRONT of the car. Values must agree with the firmware:
  // camera.py places it 13.5 cm ahead of the odometry origin (the rear axle) in
  // get_traffic_lane_inside_pov(). Using the body half-length (6.35 cm) here put
  // the virtual camera 7 cm behind the real one, so the bearings, the distances
  // and the POV membership were all computed from the wrong point.
  cameraOffsetFromAxleMeters: 0.135,
  // OpenMV Cam H7 with the standard 2.8 mm lens: 70.8 deg horizontal, 55.6 deg
  // vertical. The firmware's camera_angle = 70 is a rounded copy of the
  // horizontal one. Only the horizontal figure constrains detection here: the
  // world is 2D, so a sign has no elevation, and the vertical FOV only decides
  // the blob's row in the frame.
  signVisionHalfFovDeg: 35.4,
  signVisionHalfVFovDeg: 27.8,
  sensorHalfFovDeg: 15, // ultrasonic half-angle (deg)
  maxPerpDeg: 15, // allowable surface angle to accept hits (deg)
  simFps: 120,
  simSpeedMetersPerSec: 0.25, // keyboard driving
  maxControlSpeedMetersPerSec: 1.0, // ceiling for the firmware's speed command
  simTurnRadiusMeters: 0.1, // keyboard driving only; firmware steering is geometric
  // Derived from the AGSG cruise controller: max_accel 0.425 PWM/s and
  // max_decel 2.25 PWM/s at ~150 cm/s per unit duty -> 0.64 and 3.4 m/s^2.
  simAccelMetersPerSec2: 0.64,
  simBrakeMetersPerSec2: 3.4,
  imuSign: -1, // firmware heading = imuSign * world heading (ghost overlay)
  telemetryNoise: {
    enabled: true, // scored runs are noisy by default; the UI has a toggle
    headingDegSigma: 1.0,
    // A real gyro does not only jitter: it has a fixed bias for the round plus a
    // slow random walk. Bias is drawn once per round, drift accumulates as
    // sigma * sqrt(seconds), which is how a MEMS rate sensor behaves.
    headingBiasDegSigma: 0.35,
    headingDriftDegPerSqrtS: 0.05,
    // The encoder is no longer given white noise per frame: that random-walks
    // (~12 cm a minute) which no real encoder does. Quantisation and slip are
    // modelled on the Python side, see ENCODER_SLIP_* in simulator/hal.py.
    encoderMetersSigma: 0,
    // Ultrasonic: fixed per-sensor offset (mounting, cone) plus per-reading noise.
    distanceMetersSigma: 0.025,
    sonarBiasMetersSigma: 0.01,
    signDistanceMetersSigma: 0.02,
    signAngleDegSigma: 2.0,
  },
  // Mechanical realities the geometric bicycle model leaves out. Everything here
  // is a physical effect of the real car, not a tuning knob for the firmware.
  realism: {
    // "Folga" na direccao: the servo and linkage take up ~1.5 deg before the
    // wheels move at all, so small corrections are lost.
    steerBacklashDeg: 1.5,
    // Servo travel plus tyre sidewall: the wheels reach the commanded angle with
    // a first-order lag instead of instantly.
    steerLagS: 0.06,
    // Understeer: at lateral acceleration a_lat the car turns less than the
    // geometry says, because the front tyres slip. omega *= 1/(1 + k*a_lat).
    gripLossK: 0.05,
    // The body also slides outward: the velocity direction lags the heading by
    // slipLateralK * a_lat radians.
    slipLateralK: 0.04,
    wheelbaseM: 0.07,
    enabled: true,
  },
  race: {
    maxSeconds: 180, // official round limit, 3 minutes
    laps: 3,
    sectionsPerLap: 8,
    parallelToleranceDeg: 9,
    stopSpeedMetersPerSec: 0.012,
    // The firmware legitimately stands still for ~3 s while it searches for the
    // lane colour (obstacle_challenge waits up to 3 s on can_exit_lane), so a
    // shorter window would end the round on a normal pause.
    // Inside the bay, this long is enough to call it parked: the firmware's
    // approach keeps nudging at 5-10 cm/s and never holds dead still.
    parkHoldSeconds: 5, // a park only counts after the car has stood still 5 s
    stopHoldSeconds: 5, // same five seconds for the plain stop in the bay
    wallMovedSpeedMetersPerSec: 0.08,
  },
};

export let config = { ...defaultConfig };

export function resetConfig() {
  config = { ...defaultConfig };
  // The websocket lives on the same host as the page, one port down: opening the
  // UI from another machine (http://100.120.30.84:8781) must not point at that
  // machine's localhost. ?ws=ws://host:port overrides it.
  try {
    const params = new URLSearchParams(window.location.search);
    const ws = params.get("ws");
    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    config.wsUrl = ws || `${proto}://${window.location.hostname}:8780`;
    const start = params.get("start");
    if (start) config.startMode = start;
    const dir = params.get("dir");
    if (dir) config.startDirection = dir;
    const speed = params.get("speed");
    if (speed) config.initialSimScale = Math.max(1, Number(speed) || 1);
    const seed = params.get("seed");
    if (seed) config.seed = Number(seed);
    const pose = params.get("pose");
    if (pose) {
      const [px, py, ph] = pose.split(",").map(Number);
      if ([px, py, ph].every(Number.isFinite)) {
        config.startPose = { x: px / 100, y: py / 100, headingDeg: ph };
      }
    }
    const imu = params.get("imu");
    if (imu) config.imuSignParam = Number(imu);
    const servo = params.get("servo");
    if (servo) config.servoSignParam = Number(servo);
    // ?signs=150,60,1;60,100,2 -> exact sign layout, applied on every start so a
    // reload keeps the field. Same shape as window.__race.field.set: x_cm, y_cm,
    // state with 1=green and 2=red. Written down in that compact form because a
    // JSON array does not survive being typed into an address bar.
    // ?realism=off removes the mechanical effects (backlash, lag, grip loss,
    // lateral slip) and ?noise=off the sensor errors, so a change can be
    // attributed instead of guessed at.
    const realism = params.get("realism");
    if (realism === "off" || realism === "0") {
      config.realism.enabled = false;
      config.realism.steerBacklashDeg = 0;
      config.realism.steerLagS = 0;
      config.realism.gripLossK = 0;
      config.realism.slipLateralK = 0;
    }
    const noise = params.get("noise");
    if (noise === "off" || noise === "0") config.telemetryNoise.enabled = false;
    if (noise === "on" || noise === "1") config.telemetryNoise.enabled = true;
    const signs = params.get("signs");
    if (signs) {
      config.fieldSigns = signs
        .split(";")
        .map((chunk) => chunk.split(",").map(Number))
        .filter((n) => n.length === 3 && n.every(Number.isFinite))
        .map(([x, y, state]) => ({ x_cm: x, y_cm: y, state }));
    }
  } catch (_) {
    /* not in a browser context */
  }
  return config;
}

