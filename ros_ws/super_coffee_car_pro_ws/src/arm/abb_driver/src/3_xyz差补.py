#!/usr/bin/env python3 
import rospy
from moveit_commander import MoveGroupCommander
import sys
import numpy as np
import time
import os
import json
from datetime import datetime
import triad_openvr
import math
from scipy.spatial.transform import Rotation as R

R_viveOinBase = np.array(
                [[-0.36569105, -0.03870039, -0.92993136],
                 [-0.92834085, -0.05647087,  0.3674157 ],
                 [-0.06673317,  0.99765391, -0.01527626]
                ])
# t_viveOinBase = np.array([-478.72913305, 171.26425215, 1157.82135641])       # 2026/2/9 原始数据
# t_viveOinBase = np.array([-469.66167436, 166.9760486, 1154.51525089])       # 2026/2/9 修正数据
t_viveOinBase = np.array([-567.13963065,  185.49465537, 1149.35589731])       # 2026/3/9 原始数据
t_viveOinBase = np.array([-557.94508662,  183.69988032, 1151.66627443])       # 2026/3/9 修正数据



tracker_poses = [
[21.07057340273002, 51.91642607336999, 178.57804641371, 87.35011424894596, -1.0768024192967958, -3.57688588593818],
[48.40983454377999, 109.21972798302, 393.97740127504005, 114.52230353048724, 47.476675135272785, -3.219109013108197],
[-49.43429565186, 94.38099514833998, 382.7404760816801, 109.99846650292899, 48.00382273009482, 45.59455882654771],
[123.29805686999009, 14.698293324809981, 387.65606605876997, 116.59828788213714, 48.773027385488014, -62.657455408785076],
[96.10863043950008, 274.38834487486, 370.60374303168, 86.40176577022156, 43.31926551149202, -31.06134623761162],
[3.369513532580072, -184.78431690009998, 451.3672993235001, 179.17097375830446, 23.34615001488108, 31.70697003998366],
[151.17046575921995, -263.80582599388003, 487.14277268420005, -167.73787633222636, 10.724112964558874, 70.78337844708179],
[-109.82310052039998, -40.584691009520014, 564.5212499446, 176.7042104884853, -28.186423521113802, -87.45469507258251],
[-59.02072632949995, -104.11583150972004, 558.6258312997201, -172.34490710097685, 33.4479904668208, 1.955703160667349],
[20.415488223109946, 19.691127540450026, 637.20488513219, 168.7225283705214, -4.334730840991581, -52.72766359129833],
]
base_poses = [
[0.01, 54.0, 175.462, 90.011, -0.0, -0.011],
[31.797, 112.076, 392.266, 115.108, 47.102, -0.438],
[-64.594, 90.313, 385.127, 115.142, 47.082, 51.786],
[108.705, 24.216, 385.227, 115.068, 47.062, -61.34],
[64.923, 277.047, 373.403, 87.715, 43.484, -26.866],
[4.762, -185.344, 447.586, -179.48, 24.543, 33.077],
[159.665, -253.326, 474.951, -165.14, 11.457, 72.057],
[-112.321, -45.474, 565.543, 177.19, -27.409, -86.45],
[-56.301, -105.326, 556.932, -173.517, 35.104, 2.659],
[16.973, 20.77, 632.817, 168.838, -3.224, -51.626],
]


def Quaternion2Euler(x,y,z,w):
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

rospy.init_node('get_end_effector_pose_py')
planning_group = "arm"

v = triad_openvr.triad_openvr()


print('\n'+'='*20+"输入差补的数据数"+'='*20+'\n')
i=0
max = 5
while True:
    try:
        collect = input("请输入采集次数（整数）：")
        max = int(collect)
        break
    except:
        print("输入的不是整数，程序退出")
        exit()

print(f"接下来进行{max}次采集（每次回车继续，输入q退出）")
while i<max:
    if str(input()) == 'q':
        break
    i+=1
    print(f"第{i}次：")

    original_tracker_pose = v.devices["tracker_1"].get_pose_euler()
    T = [round(original_tracker_pose[0]*1000, 3),
        round(original_tracker_pose[1]*1000, 3),
        round(original_tracker_pose[2]*1000, 3)]
    R_euler = [float(original_tracker_pose[3])*180/math.pi, 
                float(original_tracker_pose[4])*180/math.pi, 
                float(original_tracker_pose[5])*180/math.pi]
    R_euler = R.from_matrix(R_viveOinBase @ R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()).as_euler('xyz', degrees=True)
    T = R_viveOinBase @ np.array(T) + t_viveOinBase
    print("计算值："+str(T.tolist()+R_euler.tolist()))
    tracker_poses.append(T.tolist()+R_euler.tolist())

    move_group = MoveGroupCommander(planning_group)
    current_pose = move_group.get_current_pose()
    # rospy.loginfo(f"参考坐标系: {current_pose.header.frame_id}")
    x, y, z = current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z
    x, y, z = round(x*1000, 3), round(y*1000, 3), round(z*1000, 3)
    rx, ry, rz = Quaternion2Euler(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)
    rx, ry, rz = round(rx*180/math.pi, 3), round(ry*180/math.pi, 3), round(rz*180/math.pi, 3)
    
    print("真实值："+str([x, y, z, rx, ry, rz]))
    base_poses.append([x, y, z, rx, ry, rz])

print(f"\n共{i}次")
print('\n'+'='*20+"计算误差并矫正T"+'='*20+'\n')

if type(tracker_poses) == list:

    print("tracker_poses = [")
    for i in range(len(tracker_poses)):
        print(str(tracker_poses[i])+',',sep='')
    print("]")

    print("base_poses = [")
    for i in range(len(base_poses)):
        print(str(base_poses[i])+',',sep='')
    print("]")

    diff = [[] for i in range(len(tracker_poses))]
    for index, pose in enumerate(tracker_poses):
        for i in range(0, 3):
            diff[index].append(pose[i]-base_poses[index][i])
        print(f"第{index}组diff:", diff[index])

    print("原T:", t_viveOinBase)
    for i in range(0, 3):
        for d in diff:
            t_viveOinBase[i] = t_viveOinBase[i] + d[i] / len(diff)
    print("矫正后T:", t_viveOinBase)