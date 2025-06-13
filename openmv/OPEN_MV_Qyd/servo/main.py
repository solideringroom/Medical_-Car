import sensor, image, time

from pid import PID
from pyb import Servo
from pyb import UART

uart = UART(1,115200)
compo_angle = 2

pan_servo=Servo(1)      #打开控制伺服的定时器的通道，相当于初始化
tilt_servo=Servo(2)

pan_servo.calibration(500,2500,500)#这个500可能是初始化归零
tilt_servo.calibration(500,2500,500)

red_threshold  = (34, 56, 44, 92, -11, 59)

pan_pid = PID(p=0.08, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.08, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_vflip(True)  #垂直反转一下，要不有点别扭
clock = time.clock() # Tracks FPS.

def find_max(blobs):       #返回方框最大的物体
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob


while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    #uart.write("@.%d.%d.\r\n"%(2, 3))
    blobs = img.find_blobs([red_threshold],area_threshold = 10)
    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        # print("pan_error: ", pan_error)

        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        uart.write("@%d.%d.\r\n"%(max_blob.cx(), max_blob.cy()))
        print(max_blob.cx(), max_blob.cy())

        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        # print("pan_output",pan_output)
        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()+tilt_output)
        #print(max_blob.cx()* max_blob.cy());
    # tilt_servo.angle(180+compo_angle)
