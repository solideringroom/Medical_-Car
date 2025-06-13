# Untitled - By: qigud - Tue Jun 10 2025

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

ROI = (0,0,100,100)

while True:
    clock.tick()
    img = sensor.snapshot()
    statistics = img.get_statistics(roi = ROI)
    color_l = statistics.l_mean()
    color_a = statistics.a_mean()
    color_b = statistics.b_mean()
    img.draw_rectangle(ROI)
    print(clock.fps(),color_l,color_a,color_b)











