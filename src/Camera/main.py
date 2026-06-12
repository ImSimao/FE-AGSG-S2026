import sensor, image, time, pyb
DEBUG = False

GREEN_THRESHOLDS = [
    (0, 100, -128, -9, 0, 127)
]
RED_THRESHOLDS = [
    (0, 100, 17, 127, 24, 127)
]
THRESHOLDS = RED_THRESHOLDS + GREEN_THRESHOLDS

RES = [160, 120]
START_Y = int(0.37*RES[1])
HEIGHT = RES [1] - START_Y
ROI = (0, START_Y, RES[0], HEIGHT)



sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
clock = time.clock()

p0 = pyb.Pin("P0", pyb.Pin.OUT_PP)
p1 = pyb.Pin("P1", pyb.Pin.OUT_PP)
p2 = pyb.Pin("P2", pyb.Pin.OUT_PP)
p3 = pyb.Pin("P3", pyb.Pin.OUT_PP)
while True:
    clock.tick()
    img = sensor.snapshot()
    best_blob = None
    best_area = 0
    best_color = None
    blobs = img.find_blobs(
        THRESHOLDS,
        roi=ROI,
        merge=False,
        pixels_threshold=80,
        area_threshold=80
    )

    valid_blobs = []

    for blob in blobs:
        w = blob.w()
        h = blob.h()
        if w == 0 or h == 0:
            continue
        ratio = h / w
        if ratio > 2.5:
            continue

        if ratio < 1:
            continue


        area = blob.pixels()
        code = blob.code()
        if DEBUG:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(
                blob.x(),
                blob.y() - 10,
                str(code),
                color=(255, 255, 255)
            )
        if code == 1:
            color = "RED"
        elif code == 2:
            color = "GREEN"
        elif code == 3:
            color = "RED"
        else:
            continue
        if area > best_area:
            best_area = area
            best_blob = blob
            best_color = color
        
        valid_blobs.append([blob.cx(), blob.y()+blob.h(), color])
    
    if best_blob:
        if DEBUG:
            img.draw_rectangle(best_blob.rect(), color=(0, 255, 255))
            img.draw_string(5, 5, best_color, color=(255, 255, 255))

    if DEBUG:
        img.draw_string(5, 20, "FPS: " + str(clock.fps()), color=(255, 255, 255))
