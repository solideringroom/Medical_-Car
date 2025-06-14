from pyb import millis #pyb是micropython的硬件调用库，millis返回系统运行时间，类似systick
from math import pi, isnan #isnan用来处理nan字符串返回值

class PID:
    _kp = _ki = _kd = _integrator = _imax = 0
    _last_error = _last_derivative = _last_t = 0
    _RC = 1/(2 * pi * 20) #这个20是截止频率。高于这个频率的噪声会被衰减
    def __init__(self, p=0, i=0, d=0, imax=0): #这个是初始化参数
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float('nan')
    #没懂这个格式。32是目标值，测量值。这个是误差和比例
    def get_pid(self, error, scaler):
        #首先对每一帧之间的时间进行计算。不同于之前32中的定时器pid，这里时间是需要每一帧计算的
        tnow = millis()
        dt = tnow - self._last_t        #python比较灵活，可以随时动态添加成员，不需要提前声明
        output = 0                      #其实感觉用不着，但是没啥问题
        if self._last_t == 0 or dt > 1000:  #刚开机或者卡顿，那就把积分项重置。为什么？
            dt = 0                          #如果系统卡顿，那么之前的I可能应对不了卡顿之后的情况（情况变化较大）
            self.reset_I()
        self._last_t = tnow
        delta_time = float(dt) / float(1000)        #换算成秒
        output += error * self._kp
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time
            derivative = self._last_derivative + \
                                     ((delta_time / (self._RC + delta_time)) * \
                                        (derivative - self._last_derivative))#这是滤波器，低通，平滑D项
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax: self._integrator = -self._imax
            elif self._integrator > self._imax: self._integrator = self._imax
            output += self._integrator
        return output
    def reset_I(self): #其实这里reset的是过去的影响
        self._integrator = 0
        self._last_derivative = float('nan')
