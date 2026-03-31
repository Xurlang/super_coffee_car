# #!/usr/bin/env python3 
# import rospy
# from moveit_commander import MoveGroupCommander
# import sys
# import numpy as np
# import time
# import os
# import json
# from datetime import datetime
# import triad_openvr
# import math
# from scipy.spatial.transform import Rotation as R
# from playsound import playsound

# def Quaternion2Euler(x,y,z,w):
#     # Roll (x-axis rotation)
#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = math.atan2(sinr_cosp, cosr_cosp)

#     # Pitch (y-axis rotation)
#     sinp = 2 * (w * y - z * x)
#     if abs(sinp) >= 1:
#         pitch = math.copysign(math.pi / 2, sinp) # Use 90 degrees if out of range
#     else:
#         pitch = math.asin(sinp)

#     # Yaw (z-axis rotation)
#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = math.atan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

# rospy.init_node('get_end_effector_pose_py')
# planning_group = "arm"
# move_group = MoveGroupCommander(planning_group)
# rate = rospy.Rate(1) 

# v = triad_openvr.triad_openvr()

# def get_vive_pose():
#     T = [v.devices["tracker_1"].get_pose_euler()[0]*1000,
#             v.devices["tracker_1"].get_pose_euler()[1]*1000,
#             v.devices["tracker_1"].get_pose_euler()[2]*1000]
#     R_euler = [float(v.devices["tracker_1"].get_pose_euler()[3])*180/math.pi, 
#                 float(v.devices["tracker_1"].get_pose_euler()[4])*180/math.pi, 
#                 float(v.devices["tracker_1"].get_pose_euler()[5])*180/math.pi]
#     for i in range(3):
#         T[i] += (200 * R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()[:, 2])[i]
#     R_euler = R.from_matrix(R_viveOinBase @ R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()).as_euler('xyz', degrees=True)
#     T = R_viveOinBase @ np.array(T) + t_viveOinBase
#     return T.tolist()+R_euler.tolist()

# def moveL_robot_to_pose(pose):
#     try:
#         nRet = robot.HRIF_MoveL(0,0,pose,[0, 0, 0, 0, 0, 0],"TCP_needle_test","Base",30,35,0,0,0,0,"1")
#         if nRet != 0:
#             print(f"移动命令失败，错误码: {nRet}")
#         robot.waitMoveDone(0, 0)
#         print("移动完成")
#         time.sleep(1)
#     except Exception as e:
#         print(f"移动机器人失败: {e}")

# R_viveOinBase = np.array([
#                 [ 0.49793258, -0.01303898, -0.86711772],
#                 [-0.86698482, -0.03055769, -0.49739676],
#                 [-0.02001157,  0.99944795, -0.02652027]
#                 ])
# # t_viveOinBase = np.array([-366.41914416, -689.52656168, 1023.12539028])       # 2026/1/28 5个位置矫正1次
# # t_viveOinBase = np.array([-391.41914416, -714.52656168, 1023.12539028])       # 2026/1/29 x-25,y-25
# # t_viveOinBase = np.array([-379.41914416, -709.52656168, 1019.12539028])       # 2026/1/30 x-13,y-20
# t_viveOinBase = np.array([-388.41914416, -714.52656168, 1023.12539028])       # 2026/1/30
# t_viveOinBase[0] += 0
# t_viveOinBase[1] += 0

# time.sleep(3)
# playsound("audio/yy_setStart_1.mp3")
# # time.sleep(5)
# # start = get_vive_pose()

# time.sleep(2)
# playsound("audio/ding.mp3")
# time.sleep(1)
# start = []
# for i in range(20):
#     start.append(get_vive_pose())
#     time.sleep(0.1)
# start_avg = []
# for i in range(6):
#     sum = 0
#     for j in range(20):
#         sum += start[j][i]
#     start_avg.append(round(sum/20,3))
# start = start_avg

# playsound("audio/yy_setStartOK_1.mp3")
# time.sleep(3)
# playsound("audio/yy_setEnd_1.mp3")
# time.sleep(2)
# playsound("audio/ding.mp3")
# time.sleep(3)
# end = get_vive_pose()
# playsound("audio/yy_setEndOK_1.mp3")
# time.sleep(3)

# result=[]
# nRet = robot.HRIF_ReadActPos(0,0,result)
# z = float(result[8])
# start[2], start[3], start[4], start[5] = z, 180, -45, 0
# end[2], end[3], end[4], end[5] = z, start[3], start[4], start[5]
# playsound("audio/yy_startTracking_1.mp3")
# time.sleep(1)
# moveL_robot_to_pose(start)
# moveL_robot_to_pose(end)

