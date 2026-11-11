import { appState, Modes } from "./appState.js";
import { recenterArenaView } from "./view.js";
import { randomizeSigns } from "./signsControl.js";
import { updateLatestPose } from "./pose.js";

export function createModeController({ dom, log, ensureSimulatorForPlay, recenterOnRobot }) {
  const isPlayTestMode = (mode) => mode === Modes.PLAY || mode === Modes.TEST;

  function updateModeButtons(mode) {
    const entries = [
      [dom.modeTelemetryBtn, Modes.TELEMETRY],
      [dom.modePlayBtn, Modes.PLAY],
    ];
    entries.forEach(([btn, value]) => {
      if (!btn) return;
      const pressed =
        (mode === Modes.TELEMETRY && value === Modes.TELEMETRY) ||
        (isPlayTestMode(mode) && isPlayTestMode(value));
      btn.setAttribute("aria-pressed", String(pressed));
    });
  }

  function setMode(mode, { force = false } = {}) {
    if (!Object.values(Modes).includes(mode)) return;
    if (mode === appState.currentMode && !force) return;

    // Play and Test are merged; treat either selection as Play.
    if (mode === Modes.TEST) mode = Modes.PLAY;

    if (mode !== Modes.PLAY && appState.simRunning) {
      ensureSimulatorForPlay(true);
    }

    appState.currentMode = mode;
    dom.simToggleBtn.disabled = mode !== Modes.PLAY;
    updateModeButtons(mode);

    switch (mode) {
      case Modes.TELEMETRY:
        log("Mode: Telemetry (sending sensor-derived streams; receiving control + odometry).");
        break;
      case Modes.PLAY:
        log("Mode: Play/Test (W/A/S/D to drive; click/drag to place; right click to set heading).");
        ensureSimulatorForPlay(false);
        break;
    }
  }

  function setFollowHeading(on) {
    appState.followHeading = on;
    dom.followHeadingBtn.textContent = `Follow Heading: ${on ? "On" : "Off"}`;
    dom.followHeadingBtn.setAttribute("aria-pressed", String(on));
    if (on) {
      recenterOnRobot();
    } else {
      appState.followRotationRad = 0;
      recenterArenaView();
    }
  }

  function toggleFollowHeading() {
    setFollowHeading(!appState.followHeading);
  }

  function setShowDistances(on) {
    appState.showDistances = on;
    dom.debugDistancesBtn.textContent = `Distances: ${on ? "On" : "Off"}`;
    dom.debugDistancesBtn.setAttribute("aria-pressed", String(on));
  }

  function toggleShowDistances() {
    setShowDistances(!appState.showDistances);
  }

  function setShowSignHud(on) {
    appState.showSignHud = on;
    dom.signHudBtn.textContent = `Sign HUD: ${on ? "On" : "Off"}`;
    dom.signHudBtn.setAttribute("aria-pressed", String(on));
  }

  function toggleShowSignHud() {
    setShowSignHud(!appState.showSignHud);
  }

  function setRealWorldMode(on) {
    appState.realWorldMode = on;
    dom.realWorldBtn.textContent = `Real World: ${on ? "On" : "Off"}`;
    dom.realWorldBtn.setAttribute("aria-pressed", String(on));
    log(on ? "Real world sensor cone + telemetry noise enabled." : "Ideal sensor rays; telemetry noise off unless forced in config.");
  }

  function toggleRealWorldMode() {
    setRealWorldMode(!appState.realWorldMode);
  }

  function resetPoseHistory() {
    appState.prevPose = null;
    appState.latestPose = null;
  }

  function initModeDefaults() {
    resetPoseHistory();
    setFollowHeading(false);
    setShowDistances(false);
    setShowSignHud(false);
    setRealWorldMode(false);
    randomizeSigns();
    appState.latestPose && updateLatestPose({ ...appState.latestPose });
  }

  return {
    setMode,
    toggleFollowHeading,
    toggleShowDistances,
    toggleShowSignHud,
    toggleRealWorldMode,
    initModeDefaults,
  };
}

