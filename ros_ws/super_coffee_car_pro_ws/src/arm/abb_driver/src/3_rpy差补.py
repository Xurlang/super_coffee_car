#!/usr/bin/env python3 
import sys
import numpy as np
import time
import os
import json
from datetime import datetime
import triad_openvr
import math
from scipy.spatial.transform import Rotation as R
import random

R_viveOinBase = np.array(
                [[-0.36569105, -0.03870039, -0.92993136],
                 [-0.92834085, -0.05647087,  0.3674157 ],
                 [-0.06673317,  0.99765391, -0.01527626]
                ])
# t_viveOinBase = np.array([-478.72913305, 171.26425215, 1157.82135641])       # 2026/2/9 原始数据
# t_viveOinBase = np.array([-469.66167436, 166.9760486, 1154.51525089])       # 2026/2/9 修正数据
t_viveOinBase = np.array([-567.13963065,  185.49465537, 1149.35589731])       # 2026/3/9 原始数据
t_viveOinBase = np.array([-557.94508662,  183.69988032, 1151.66627443])       # 2026/3/9 修正数据


x = -2.801829020422635
y = 3.441409949117306
z = 0.9515264551434677
t_viveOinBase[0] = t_viveOinBase[0]
t_viveOinBase[1] = t_viveOinBase[1]
t_viveOinBase[2] = t_viveOinBase[2]

v = triad_openvr.triad_openvr()

while True:
    original_tracker_pose = v.devices["tracker_1"].get_pose_euler()
    T = [original_tracker_pose[0]*1000,
        original_tracker_pose[1]*1000,
        original_tracker_pose[2]*1000]
    R_euler = [float(original_tracker_pose[3])*180/math.pi, 
                float(original_tracker_pose[4])*180/math.pi, 
                float(original_tracker_pose[5])*180/math.pi]
    for i in range(3):
        T[i] = (200 * R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()[:, 2])[i] + T[i]

    newR = R_viveOinBase @ R.from_euler('x', x, degrees=True).as_matrix()
    newR = newR @ R.from_euler('y', y, degrees=True).as_matrix()
    newR = newR @ R.from_euler('z', z, degrees=True).as_matrix()

    R_euler = R.from_matrix(newR @ R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()).as_euler('xyz', degrees=True)
    T = newR @ np.array(T) + t_viveOinBase
    pose = T.tolist()+R_euler.tolist()
    pose = [round(pose[i],3) for i in range(6)]
    print("计算值："+str(pose))

    diff = abs(R_euler[0]) + abs(R_euler[1]) + abs(R_euler[2])
    if diff > 0.1:
        power = 1
        x_rand = x + random.uniform(-power, power)
        y_rand = y + random.uniform(-power, power)
        z_rand = z + random.uniform(-power, power)

        newR = R_viveOinBase @ R.from_euler('x', x_rand, degrees=True).as_matrix()
        newR = newR @ R.from_euler('y', y_rand, degrees=True).as_matrix()
        newR = newR @ R.from_euler('z', z_rand, degrees=True).as_matrix()


        R_euler = [float(original_tracker_pose[3])*180/math.pi, 
                    float(original_tracker_pose[4])*180/math.pi, 
                    float(original_tracker_pose[5])*180/math.pi]
        R_euler = R.from_matrix(newR @ R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()).as_euler('xyz', degrees=True)

        if diff > abs(R_euler[0]) + abs(R_euler[1]) + abs(R_euler[2]) + 0.01:
            diff = abs(R_euler[0]) + abs(R_euler[1]) + abs(R_euler[2])
            x = x_rand
            y = y_rand
            z = z_rand
    else:
        print("xyz：" + str(x) + ", "  + str(y) + ", "  + str(z))
        break