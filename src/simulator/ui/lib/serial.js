import { appState, Modes } from "./appState.js";
import { setLatestTelemetryDistances, updateLatestPose } from "./pose.js";

export function createSerialController(deps) {
  let serialPort = null;
  let reader = null;
  let reading = false;

  async function connectSerial() {
    if (!("serial" in navigator)) {
      deps.log("Web Serial not supported. Use Chrome/Edge on localhost/HTTPS.");
      deps.setStatus("No Web Serial", "#7f1d1d");
      return;
    }
    try {
      const baudRate = Number(deps.baudInput.value) || 115200;
      serialPort = await navigator.serial.requestPort();
      await serialPort.open({ baudRate });
      deps.setStatus("Connected", "#14532d");
      deps.connectBtn.disabled = true;
      deps.disconnectBtn.disabled = false;
      deps.log(`Serial opened @ ${baudRate} baud.`);
      startReading(serialPort);
    } catch (err) {
      deps.log(`Connect error: ${err.message || err}`);
      deps.setStatus("Error", "#7f1d1d");
    }
  }

  async function disconnectSerial() {
    reading = false;
    if (reader) {
      try {
        await reader.cancel();
        reader.releaseLock();
      } catch (_) {
        /* ignore */
      }
    }
    if (serialPort) {
      try {
        await serialPort.close();
      } catch (_) {
        /* ignore */
      }
    }
    serialPort = null;
    reader = null;
    setLatestTelemetryDistances(null);
    deps.connectBtn.disabled = false;
    deps.disconnectBtn.disabled = true;
    deps.setStatus("Disconnected", "#1f2937");
    deps.log("Serial disconnected.");
  }

  async function startReading(port) {
    reading = true;
    const textDecoder = new TextDecoderStream();
    const readableStreamClosed = port.readable.pipeTo(textDecoder.writable).catch(() => {});
    const lineStream = textDecoder.readable.pipeThrough(new TransformStream(new LineBreakTransformer()));
    reader = lineStream.getReader();

    try {
      while (reading) {
        const { value, done } = await reader.read();
        if (done) break;
        if (typeof value === "string" && value.trim().length) {
          const parsed = parsePoseLine(value);
          if (parsed) {
            setLatestTelemetryDistances(parsed.distances || null);
          }
          if (parsed?.pose && appState.currentMode === Modes.TELEMETRY) {
            updateLatestPose(parsed.pose);
          }
        }
      }
    } catch (err) {
      deps.log(`Read error: ${err.message || err}`);
      deps.setStatus("Error", "#7f1d1d");
    } finally {
      reader?.releaseLock();
      await readableStreamClosed.catch(() => {});
    }
  }

  function parsePoseLine(line) {
    const { payload, hasChecksum, checksum } = splitPayloadAndChecksum(line);
    if (!payload) {
      deps.log(`Skip line (empty payload): ${line}`);
      return null;
    }
    if (hasChecksum) {
      if (Number.isNaN(checksum)) {
        deps.log(`Skip line (bad checksum field): ${line}`);
        return null;
      }
      const computed = computeXorChecksum(payload);
      if (computed !== checksum) {
        deps.log(
          `Skip line (checksum mismatch exp ${toHex(checksum)} got ${toHex(
            computed,
          )}): ${line}`,
        );
        return null;
      }
    }

    const parts = payload.split(",").map((p) => p.trim());
    if (parts.length < 3) {
      deps.log(`Skip line (need 3 fields): ${line}`);
      return null;
    }
    const nums = parts.map((p) => Number(p));
    const [x, y, headingDeg] = nums;
    if ([x, y, headingDeg].some((n) => Number.isNaN(n))) {
      deps.log(`Skip line (not a number): ${line}`);
      return null;
    }
    const labels = ["Front", "Rear", "Left", "Right"];
    const distances = nums
      .slice(3)
      .slice(0, labels.length)
      .reduce((acc, val, idx) => {
        if (!Number.isFinite(val)) return acc;
        acc[labels[idx]] = val;
        return acc;
      }, {});
    return {
      pose: { x, y, headingDeg },
      distances: Object.keys(distances).length ? distances : null,
    };
  }

  return {
    connectSerial,
    disconnectSerial,
  };
}

function computeXorChecksum(str) {
  let acc = 0;
  for (let i = 0; i < str.length; i += 1) {
    acc ^= str.charCodeAt(i) & 0xff;
  }
  return acc;
}

function toHex(val) {
  return Number(val & 0xff)
    .toString(16)
    .toUpperCase()
    .padStart(2, "0");
}

function splitPayloadAndChecksum(rawLine) {
  if (typeof rawLine !== "string") {
    return { payload: "", hasChecksum: false, checksum: null };
  }
  const trimmed = rawLine.trim();
  if (!trimmed.length) return { payload: "", hasChecksum: false, checksum: null };

  const starIdx = trimmed.lastIndexOf("*");
  if (starIdx === -1) {
    return { payload: trimmed, hasChecksum: false, checksum: null };
  }

  const payload = trimmed.slice(0, starIdx).trim();
  const checksumText = trimmed.slice(starIdx + 1).trim();
  if (!checksumText.length) {
    return { payload: trimmed, hasChecksum: false, checksum: null };
  }

  const parsed = Number.parseInt(checksumText, 16);
  if (Number.isNaN(parsed)) {
    return { payload, hasChecksum: true, checksum: NaN };
  }
  return { payload, hasChecksum: true, checksum: parsed & 0xff };
}

class LineBreakTransformer {
  constructor() {
    this.container = "";
  }
  transform(chunk, controller) {
    this.container += chunk;
    const lines = this.container.split(/\r?\n/);
    this.container = lines.pop();
    for (const line of lines) controller.enqueue(line);
  }
  flush(controller) {
    if (this.container) controller.enqueue(this.container);
  }
}

