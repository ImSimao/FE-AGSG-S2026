import sensor, image, time, pyb
DEBUG = True
GREEN_THRESHOLDS = [
	(0, 100, -128, -9, 0, 127)
]
RED_THRESHOLDS = [
	(0, 100, 17, 127, 24, 127)
]
THRESHOLDS = RED_THRESHOLDS + GREEN_THRESHOLDS
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA if not DEBUG else sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
clock = time.clock()
ROI = (0, 40, 320, 200) if DEBUG else (0, 30, 160, 120)
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
	for blob in blobs:
		w = blob.w()
		h = blob.h()
		if w == 0 or h == 0:
			continue
		ratio = max(w, h) / min(w, h)
		if ratio > 2.5:
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
	if best_blob:
		if DEBUG:
			img.draw_rectangle(best_blob.rect(), color=(0, 255, 255))
			img.draw_string(5, 5, best_color, color=(255, 255, 255))
		if best_color == "RED":
			p0.high()
		else:
			p0.low()
		cx = best_blob.cx()
		left = img.width() // 3
		right = (img.width() * 2) // 3
		if cx < left:
			p1.low(); p2.low(); p3.high()
		elif cx > right:
			p1.high(); p2.low(); p3.low()
		else:
			p1.low(); p2.high(); p3.low()
	else:
		p0.low()
		p1.low()
		p2.low()
		p3.low()
	if DEBUG:
		img.draw_string(5, 20, "FPS: " + str(clock.fps()), color=(255, 255, 255))