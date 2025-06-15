# Untitled - By: qigud - Thu Jun 12 2025
#引用，变量
import sensor
import time
from pyb import Servo
from pyb import UART

uart = UART(1,115200)
pay_angle = 2  #记得在舵机底盘上补充上这个角度


#外设初始化
pan_servo=Servo(1)      #打开控制伺服的定时器的通道，相当于初始化
tilt_servo=Servo(2)
pan_servo.calibration(500,2500,500) #这个500可能是初始化归零
tilt_servo.calibration(500,2500,500)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_vflip(True)  #垂直反转一下，要不有点别扭

clock = time.clock()
#BSP函数
servo_init_flag = 0
def Servo_Init():       #调整舵机初始角度到合适巡线的角度
    global servo_init_flag
    if(servo_init_flag == 0):
        pan_servo.angle(90+pay_angle)
        tilt_servo.angle(90)
    servo_init_flag = 1


while True:
    clock.tick()
    img = sensor.snapshot()

    print(clock.fps())
