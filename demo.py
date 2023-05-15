# -*- coding: utf-8 -*-

"""
@author     : zuti
@software   : PyCharm
@file       : 3_run_func.py
@time       : 2023/3/23 19:46
@desc       ：

"""
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint


plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams.update({'font.size': 12})

# 初始姿态
init_velocity, init_gamma, init_varphi = 260., 3.14 / 10., 0.
init_x, init_y, init_z = 0., 0., 1000.  # 初始位置

# todo 三个控制量
Nx = 3.0
Nz = 2.
mu = 3.141592653 / 12

state = [init_x, init_y, init_z, init_velocity, init_gamma, init_varphi]
control = [Nx, Nz, mu]

time = 1  # 秒 总时间
n = 10  # 仿真步数
t = np.linspace(0, time, n)  # 仿真步长


def dmove2(x_input, t, control):
    g = 9.81  # 重力加速度
    velocity, gamma, fai = x_input
    nx, nz, gunzhuan = control

    velocity_ = g * (nx - np.sin(gamma))  # # 米每秒
    gamma_ = (g / velocity) * (nz * np.cos(gunzhuan) - np.cos(gamma))  # 米每秒
    fai_ = g * nz * np.sin(gunzhuan) / (velocity * np.cos(gamma))

    return np.array([velocity_, gamma_, fai_])


def update_position(state, control, time=10, n=100):
    """

    :param state:  初始状态
    :param control: 控制量
    :param time:   仿真时长
    :param n: 仿真步数
    :return:
    """
    t = np.linspace(0, time, n)  # 仿真步长
    dt = t[1] - t[0]
    state_list = np.zeros((n, 6))  # 轨迹长度
    state_list[0] = state  # 轨迹列表第一个元素为初始状态
    x,y,z,velocity, gamma, varphi = state_list[0]

    for k in range(1, n):
        tspan = [t[k - 1], t[k]]

        st = odeint(dmove2, (velocity, gamma, varphi), tspan, args=([control[0], control[1], control[2]],))
        import math
        velocity, gamma, varphi = st[1, :]
        print(control[2]/3.1415926*180, varphi/math.pi * 180)
        dx = velocity * np.cos(gamma) * np.sin(varphi) * dt
        dy = velocity * np.cos(gamma) * np.cos(varphi) * dt
        dz = velocity * np.sin(gamma) * dt

        x = x + dx
        state_list[k, 0] = x
        y = y + dy
        state_list[k, 1] = y
        z = z + dz
        state_list[k, 2] = z

        state_list[k, 3] = velocity
        state_list[k, 4] = gamma
        state_list[k, 5] = varphi

    return state_list

#测试运动学方程
state_list = update_position(state,control)
print(f'轨迹 {state_list}')
#绘制图像
fig = plt.figure()
ax1 = fig.add_subplot(221,projection='3d')
ax1.plot(state_list[:, 0], state_list[:, 1], state_list[:, 2])
ax1.set_title('trajectory 轨迹')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

ax2 = fig.add_subplot(222)
ax2.plot(state_list[:,3])
ax2.set_title('velocity 速度')

ax3 = fig.add_subplot(223)
ax3.plot(state_list[:,4])
ax3.set_title('gamma 航迹倾角')

ax3 = fig.add_subplot(224)
ax3.plot(state_list[:,5])
ax3.set_title('varphi  航向角')

#plt.savefig('test.jpg')
plt.show()
