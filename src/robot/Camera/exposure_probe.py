# OpenMV bench probe: what does THIS firmware do with the exposure API?
#
# Run from the OpenMV IDE (do not overwrite main.py with it). It asks for six
# exposures, reads each one back, and samples the mean brightness of the frame,
# so you can see whether the number you ask for is the number the sensor uses.
#
#   exposure.wanted   what we asked for        (us)
#   exposure.readback what the sensor reports  (us)
#   diff              readback - wanted        (0 = the lock took)
#   luma              mean brightness of the frame (%)
#
# How to read it:
#   - every diff is ~0            -> the API works, and main.py's number is real
#   - every diff is large and the
#     readback never changes      -> exposure_us is ignored by this build: the
#                                    sensor keeps whatever auto exposure left,
#                                    which is why a black-wall boot blows out the
#                                    field. The fix is to drive the sensor's own
#                                    AEC registers (or a firmware update), not to
#                                    keep tweaking EXPOSURE_US.
#   - the readback follows but is
#     clamped at some value       -> that is the sensor's minimum exposure for
#                                    this frame rate; go below it with framerate
#                                    or the value never applies.
#
# Point the camera at the same scene for the whole run, otherwise the luma column
# mixes "exposure changed" with "the scene changed".

import sensor, time

WANTED_US = (2000, 5000, 10000, 15000, 20000, 25000)


def luma(img):
    value = img.get_statistics().l_mean
    return value() if callable(value) else value


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=500)

print("framesize QVGA, pixformat RGB565")
try:
    print("framerate:", sensor.get_framerate())
except Exception:
    pass

for wanted in WANTED_US:
    sensor.set_auto_exposure(False, exposure_us=wanted)
    sensor.skip_frames(time=200)
    try:
        readback = sensor.get_exposure_us()
    except Exception as err:
        readback = "ERR %s" % err

    if isinstance(readback, int):
        diff = readback - wanted
    else:
        diff = "?"

    value = luma(sensor.snapshot())
    print("wanted=%6d  readback=%s  diff=%s  luma=%.1f"
          % (wanted, readback, diff, value))

# Same question for the gain, since it is the other half of the brightness.
for gain in (0.0, 3.0, 6.0, 12.0):
    sensor.set_auto_gain(False, gain_db=gain)
    sensor.skip_frames(time=200)
    try:
        readback = sensor.get_gain_db()
    except Exception as err:
        readback = "ERR %s" % err
    print("gain wanted=%.1f  readback=%s  luma=%.1f"
          % (gain, readback, luma(sensor.snapshot())))

print("done - copy this output back into the log")
