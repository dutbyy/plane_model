import math
from math import sin, cos, asin, atan, pi
from scipy.integrate import odeint
from numpy import cos, sin, array

def distance(x1, y1, x2, y2):
    return ((x1-x2)**2 + (y1-y2)**2) **.5

def dmove2(x_input, t, control):
    g = 9.81  # 重力加速度
    velocity, gamma, course = x_input
    nx, nz, phi = control

    velocity_ = g * (nx - sin(gamma))  # # 米每秒
    gamma_ = (g / velocity) * (nz * cos(phi) - cos(gamma))  # 米每秒
    course_ = g * nz * sin(phi) / (velocity * cos(gamma))

    return array([velocity_, gamma_, course_])

# 三自由度固定翼飞机仿真模型
class PlaneModel:
    def __init__(self, config):
        # 位置
        self.x          = 0.0       # 坐标x
        self.y          = 0.0       # 坐标y
        self.z          = 1000.0    # 高度z

        # 速度
        self.velocity   = 100.0     # 速率
        self.gamma      = 0.0       # 航迹角
        self.course     = 0.0       # 航向角

        # 控制量 
        self.nx         = 0.0       # 切向过载   
        self.nz         = 1.0       # 法向过载
        self.phi        = 0.0       # 滚转角

        self.traces = []
        self.target_course = None
        self.target_point = None
        self.target_points = None

    # 推演 dt 的时间
    def advance(self, dt_ms = 100): # ms
        self.control()
        dt = dt_ms / 1000
        st = odeint(dmove2, 
            (self.velocity, self.gamma, self.course), 
            [0,dt], 
            args=([self.nx, self.nz, self.phi],)
        )
        self.velocity, self.gamma, self.course = st[1, :]
        
        dx = self.velocity * cos(self.gamma) * sin(self.course) * dt
        dy = self.velocity * cos(self.gamma) * cos(self.course) * dt
        dz = self.velocity * sin(self.gamma) * dt

        self.x += dx
        self.y += dy
        self.z += dz

        return self.summary()

    def summary(self):
        data =  {
            "name": "plane",
            "position": [ self.x, self.y],
            "side": "red",
            "icon": "plane",
            "iconsize": 36, 
            'course': self.course / pi * 180,
            "cirsize": 0,
        }
        def distance(a, b):
            return ((a[0]-b[0])**2 + (a[1] - b[1])**2) ** .5
        if len(self.traces) <= 0 or distance(self.traces[-1]['position'], data['position']) > 500:
            trace = {
                "position": data['position'],
                "side": "red",
                "icon": "point",
                "iconsize": 8, 
            }
            self.traces.append(trace)
        if len(self.traces) > 180:
            self.traces = self.traces[-180:]
        return data, self.traces
    
    def control(self):
        if self.target_course is not None:
            dcourse = ( self.target_course +  2 * pi - self.course ) % (2 * pi)
            if dcourse < 1e-4:
                self.target_course = None
            elif dcourse <= pi:
                self.phi = 3.141592653 / 12
            else :
                self.phi = - 3.141592653 / 12
        elif self.target_point is not None:
            dx = self.target_point[0] - self.x
            dy = self.target_point[1]  - self.y 
            tcourse = (atan(dx/dy) + 2 * pi) % ( 2 * pi)
            if dy < 0: 
                tcourse = tcourse +  pi 
            tcourse = (tcourse + 2 * pi) % (2 * pi)
            dcourse = ( tcourse +  2 * pi - self.course ) % (2 * pi)
            if distance(self.x, self.y, self.target_point[0], self.target_point[1]) < 10:
                self.target_point = None
            elif dcourse <= pi:
                self.phi = 3.141592653 / 12
            else :
                self.phi = - 3.141592653 / 12
        else:
            self.phi = 0

        if self.target_points and not self.target_point:
            self.target_point = self.target_points[0]

            if len(self.target_points) == 1:
                self.target_points = None
            else:
                self.target_points = self.target_points[1:]
        

        self.nz = 1 / cos(self.phi) 



    def receive_command(self, command):
        self.command = command 



        