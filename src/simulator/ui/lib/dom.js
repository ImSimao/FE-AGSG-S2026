export function getDomRefs() {
  const canvas = document.getElementById("arenaCanvas");
  return {
    canvas,
    ctx: canvas.getContext("2d"),
    logPanel: document.getElementById("logPanel"),
    logArea: document.getElementById("logArea"),
    logToggleBtn: document.getElementById("logToggleBtn"),
    statusBadge: document.getElementById("statusBadge"),
    connectBtn: document.getElementById("connectBtn"),
    disconnectBtn: document.getElementById("disconnectBtn"),
    simToggleBtn: document.getElementById("simToggleBtn"),
    randomizeSignsBtn: document.getElementById("randomizeSignsBtn"),
    surpriseRuleBtn: document.getElementById("surpriseRuleBtn"),
    modeTelemetryBtn: document.getElementById("modeTelemetryBtn"),
    modePlayBtn: document.getElementById("modePlayBtn"),
    followHeadingBtn: document.getElementById("followHeadingBtn"),
    debugDistancesBtn: document.getElementById("debugDistancesBtn"),
    realWorldBtn: document.getElementById("realWorldBtn"),
    signHudBtn: document.getElementById("signHudBtn"),
    clearLogBtn: document.getElementById("clearLogBtn"),
    baudInput: document.getElementById("baudRate"),
    cursorPosEl: document.getElementById("cursorPos"),
    bgInput: document.getElementById("bgInput"),
    toastContainer: document.getElementById("toastContainer"),
    // race panel
    racePhase: document.getElementById("racePhase"),
    raceTime: document.getElementById("raceTime"),
    raceLaps: document.getElementById("raceLaps"),
    raceSections: document.getElementById("raceSections"),
    raceScore: document.getElementById("raceScore"),
    raceDirection: document.getElementById("raceDirection"),
    raceSection: document.getElementById("raceSection"),
    raceGhost: document.getElementById("raceGhost"),
    raceFlags: document.getElementById("raceFlags"),
    raceBreakdown: document.getElementById("raceBreakdown"),
    raceSeed: document.getElementById("raceSeed"),
    newTrackBtn: document.getElementById("newTrackBtn"),
    repeatTrackBtn: document.getElementById("repeatTrackBtn"),
    noiseBtn: document.getElementById("noiseBtn"),
    speedBtn: document.getElementById("speedBtn"),
  replayBar: document.getElementById("replayBar"),
  replayEnter: document.getElementById("replayEnter"),
  replayBack: document.getElementById("replayBack"),
  replayPlay: document.getElementById("replayPlay"),
  replaySpeed: document.getElementById("replaySpeed"),
  replayFwd: document.getElementById("replayFwd"),
  replaySlider: document.getElementById("replaySlider"),
  replayTime: document.getElementById("replayTime"),
  signReadout: document.getElementById("signReadout"),
  exportJsonBtn: document.getElementById("exportJsonBtn"),
  exportCsvBtn: document.getElementById("exportCsvBtn"),
  };
}

export function createUi(dom) {
  function notify(message, { variant = "info", duration = 3500 } = {}) {
    if (!dom.toastContainer || !message) return;
    const toast = document.createElement("div");
    toast.className = `toast toast-${variant}`;
    toast.textContent = message;
    toast.setAttribute("role", "status");
    dom.toastContainer.appendChild(toast);
    // Allow pointer events on the toast but not the container so clicks pass through.
    const remove = () => {
      toast.classList.add("toast-hide");
      setTimeout(() => toast.remove(), 180);
    };
    toast.addEventListener("click", remove);
    setTimeout(remove, duration);
  }

  function log(message) {
    const text = String(message ?? "");
    const ts = new Date().toLocaleTimeString();
    dom.logArea.textContent += `[${ts}] ${text}\n`;
    dom.logArea.scrollTop = dom.logArea.scrollHeight;
    if (/^\s*warning\b/i.test(text)) {
      notify(text, { variant: "warning" });
    }
  }

  function setStatus(text, color = "#1f2937") {
    dom.statusBadge.textContent = text;
    dom.statusBadge.style.background = color;
  }

  return { log, setStatus, notify };
}

