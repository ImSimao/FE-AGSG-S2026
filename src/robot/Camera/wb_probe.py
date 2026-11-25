# OpenMV bench probe: measure the white balance once, then pin it for good.
#
# main.py fixes exposure, gain and white balance to numbers so the image is the
# same whatever the camera was pointing at when it was switched on. Exposure and
# gain are chosen by hand; the white balance cannot be, because the camera has no
# idea what "white" looks like - it has to average a scene that contains one.
#
# Run this script (from the OpenMV IDE) with the camera where the car will run,
# ideally with a sheet of white paper filling the frame:
#
#   1. it holds the same exposure and gain main.py uses,
#   2. it lets auto white balance settle,
#   3. it prints the tuple in paste-ready form.
#
# Copy that line over FIXED_RGB_GAINS in main.py. From then on every boot gives
# the same image, including boots against a black wall.

import sensor, time

# Must match main.py, otherwise the numbers describe a different image.
EXPOSURE_US = 14000
FIXED_GAIN_DB = 0.0

SETTLE_MS = 2500


def luma(img):
    value = img.get_statistics().l_mean
    return value() if callable(value) else value


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# The same "first write after reset is swallowed" trick main.py works around: set,
# read back, retry. Without this the exposure stays at whatever the last auto
# frame chose and the measurement describes the wrong image.
for _attempt in range(3):
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    sensor.skip_frames(time=100)
    try:
        if abs(sensor.get_exposure_us() - EXPOSURE_US) <= 300:
            break
    except Exception:
        break

sensor.set_auto_gain(False, gain_db=FIXED_GAIN_DB)
sensor.set_auto_whitebal(True)

print("measuring for %d ms, keep the scene still" % SETTLE_MS)
sensor.skip_frames(time=SETTLE_MS)

try:
    red, green, blue = sensor.get_rgb_gain_db()
except Exception as err:
    print("get_rgb_gain_db failed: %s" % err)
    red = green = blue = None

frame = sensor.snapshot()
print("exposure=%s us  gain=%s dB  luma=%.1f"
      % (sensor.get_exposure_us(), sensor.get_gain_db(), luma(frame)))

# Is the statistics object actually describing the frame? A covered lens has to
# read near zero, so print several fields and a small centre ROI. On the v5
# firmware l_mean came back as a constant 45.0 with the lens capped and with the
# field in front of it, which no sensor can produce - that is a broken
# instrument, not a broken camera, and it must not be used to tune exposure.
try:
    stats = frame.get_statistics()
    parts = []
    for name in ("l_mean", "l_max", "l_median", "l_std_dev"):
        value = getattr(stats, name, None)
        if callable(value):
            value = value()
        parts.append("%s=%s" % (name, value))
    print("frame stats:", " ".join(parts))
    centre = frame.get_statistics(roi=(120, 90, 80, 60))
    value = centre.l_mean
    if callable(value):
        value = value()
    print("centre ROI l_mean=%.1f" % value)
except Exception as err:
    print("statistics failed: %s" % err)

if red is not None:
    print("")
    print("paste this into main.py:")
    print("FIXED_RGB_GAINS = (%.2f, %.2f, %.2f)" % (red, green, blue))
    print("")
    print("then set AUTO_WHITEBAL = False so nothing drifts afterwards.")

# Two more samples a second apart: if the tuple keeps moving the scene is not
# giving the auto loop a stable reference (a coloured wall, or a hand in front of
# the lens), and that number is not worth pinning.
for _ in range(2):
    sensor.skip_frames(time=1000)
    try:
        red, green, blue = sensor.get_rgb_gain_db()
        print("again: (%.2f, %.2f, %.2f)" % (red, green, blue))
    except Exception:
        break

print("done - copy the line back into the log")
