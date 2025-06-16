THRESHOLD = (12, 59, 15, 48, -1, 34)#(11, 56, 13, 77, -10, 42) # Grayscale threshold for dark things...#阈值
import sensor, time
from pyb import LED
from pyb import UART
from pyb import Servo
uart = UART(1,115200)
compo_angle = 2
pan_servo=Servo(1)      #打开控制伺服的定时器的通道，相当于初始化
tilt_servo=Servo(2)
pan_servo.calibration(500,2500,500)#这个500可能是初始化归零
tilt_servo.calibration(500,2500,500)

pan_servo_flag = 0
TASK_NUM = -1
LED(1).on()#补光
LED(2).on()#补光
LED(3).on()#补光

Roi_left = (0,0,10,20)
Roi_right = (70,0,10,20)
Roi_down = (0,20,80,40)
Roi_middle = (25,0,30,20)
left_detected = 0
right_detected = 0
middle_detected = 0
crossing_detect_threshold = 100;
meet_crossing_flag = 0;#路口标志位：0为无，1为右手丁字，2为左手丁字，3为十字

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(False)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.
TASK_NUM = 1#调试时打开
#车向左偏，theta是180减去相对于中线的偏角
#所以在theta>90,判定为小车向左偏，直接把theta-180得到角度。
#垂直向前是0度，小车右偏是小于90度的正数，不用管它
#综上：
#小车左偏：大于90度，减去180度 小车右偏，小于90度，不做处理
while(True):
    clock.tick()

    if uart.any():
        recv_task = uart.read(1)
        if recv_task in b'0123456789':  #如果串口给了个非数字也不会报错
            TASK_NUM = int(recv_task)
            uart.write("^%d.\r\n"%(int(recv_task)))
            print("OK")   #用于调试
    if TASK_NUM == 1:
        #设置舵机角度，只执行一次
        if pan_servo_flag == 0:
            pan_servo.angle(90+compo_angle)
            tilt_servo.angle(180)
            pan_servo_flag = 1;
        #拍照，二值化，回归拟合，得到距离和角度误差，如果可信度比较大，传给32
        img = sensor.snapshot().binary([THRESHOLD])
        #识别路口
        left_blobs = img.find_blobs([(100, 255)], roi=Roi_left, merge=True)
        right_blobs = img.find_blobs([(100, 255)], roi=Roi_right, merge=True)
        middle_blobs = img.find_blobs([(100, 255)], roi = Roi_middle, merge = True)
        #检测左右roi内是否有足够多到色块
        if left_blobs:
            if(left_blobs[0].pixels() > crossing_detect_threshold):
                left_detected = 1;
            else:
                left_detected = 0;
        else:
            left_detected = 0;#再写一遍是因为如果没有色块，left_blobs就为空，
                              #此时进不到上面那个判断里面，left_detected就无法被置零，下同理
        if right_blobs:
            if(right_blobs[0].pixels() > crossing_detect_threshold):
                right_detected = 1;
            else:
                right_detected = 0;
        else:
            right_detected = 0;
        if middle_blobs:
            if(middle_blobs[0].pixels() > crossing_detect_threshold):
                middle_detected = 1;
            else:
                middle_detected = 0;
        else:
            middle_detected = 0
        #根据左右roi是否检测到色块判断路口类型
        if(left_detected == 0 and right_detected == 1 and middle_detected == 1):
            meet_crossing_flag = 1
        elif(left_detected == 1 and right_detected == 0 and middle_detected == 1):
            meet_crossing_flag = 2
        elif(left_detected == 1 and right_detected == 1 and middle_detected == 1):
            meet_crossing_flag = 3
        else:
            meet_crossing_flag = 0
        #print(left_detected,right_detected,meet_crossing_flag)
        #只对下半部分画面进行线性回归
        img.draw_rectangle(Roi_down, (255,0,0))
        img.draw_rectangle(Roi_left, (0,255,0))
        img.draw_rectangle(Roi_right, (0,255,0))
        img.draw_rectangle(Roi_middle, (255,165,0))

        line = img.get_regression([(100,100)], roi=Roi_down, robust = True)
        if (line):
            img.draw_line(line.line(), color = 127)
            rho_err = abs(line.rho())-img.width()/2
            if line.theta()>90:
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
            print(rho_err,line.magnitude(),theta_err)
            if line.magnitude()>8:
                uart.write("@%d.%d.%d.\r\n"%(rho_err,theta_err,meet_crossing_flag))
                print(rho_err,line.magnitude(),theta_err,meet_crossing_flag )   #一会用于测试
                #print(clock.fps())
        else:
            uart.write("@%d.%d.%d.\r\n"%(-1,-1,0))



