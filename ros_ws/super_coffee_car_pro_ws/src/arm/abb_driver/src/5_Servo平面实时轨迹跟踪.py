import sys
import numpy as np
import time
import os
import json
from datetime import datetime
import triad_openvr
from CPS import CPSClient
import math
from scipy.spatial.transform import Rotation as R

try:
    robot_ip = '192.168.1.10'
    robot = CPSClient()
    robot.HRIF_Connect(0,robot_ip,10003)
    print(f"成功连接到机器人 {robot_ip}")
except Exception as e:
    raise RuntimeError(f"机器人初始化失败: {e}")

v = triad_openvr.triad_openvr()

def get_vive_pose():
    T = [v.devices["tracker_1"].get_pose_euler()[0]*1000,
        v.devices["tracker_1"].get_pose_euler()[1]*1000,
        v.devices["tracker_1"].get_pose_euler()[2]*1000]
    R_euler = [float(v.devices["tracker_1"].get_pose_euler()[3])*180/math.pi, 
                float(v.devices["tracker_1"].get_pose_euler()[4])*180/math.pi, 
                float(v.devices["tracker_1"].get_pose_euler()[5])*180/math.pi]
    for i in range(3):
        T[i] = (200 * R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()[:, 2])[i] + T[i]
    R_euler = R.from_matrix(R_viveOinBase @ R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()).as_euler('xyz', degrees=True)
    T = R_viveOinBase @ np.array(T) + t_viveOinBase
    return T.tolist()+R_euler.tolist()

R_viveOinBase = np.array([
                [ 0.49793258, -0.01303898, -0.86711772],
                [-0.86698482, -0.03055769, -0.49739676],
                [-0.02001157,  0.99944795, -0.02652027]
                ])
# t_viveOinBase = np.array([-366.41914416, -689.52656168, 1023.12539028])       # 2026/1/28 5个位置矫正1次
t_viveOinBase = np.array([-396.71914416, -705.52656168, 1023.12539028])       # 2026/1/29 x-25,y-25


# 2. 关键：设置安全速度/加速度（按实际需求调整）
# 2.1 全局速度比（优先设低，测试后再调高）
robot.HRIF_SetOverride(0, 0, 0.1)  # 10%原始速度
# 2.2 关节速度上限（6轴分别设置，避免单轴过快）
robot.HRIF_SetJointMaxVel(0, 0, [20, 20, 20, 20, 20, 20])
# 2.3 关节加速度上限（降低加速度减少冲击）
robot.HRIF_SetJointMaxAcc(0, 0, [30, 30, 30, 30, 30, 30])
# 2.4 TCP直线速度上限（ServoP模式生效）
robot.HRIF_SetLinearMaxVel(0, 0, 20)
# 2.5 TCP直线加速度上限
robot.HRIF_SetLinearMaxAcc(0, 0, 30)



# 周期 
dServoTime = 0.02
# dServoTime = 1
# 前瞻时间 
dLookaheadTime = 0.2
# 启动机器人在线控制 
nRet = robot.HRIF_StartServo(0,0,dServoTime, dLookaheadTime)
# 定义工具坐标变量 
result=[]
nRet = robot.HRIF_ReadActPos(0,0,result)
# z = float(result[8])
z = 200
z = 222.260
dTcp = [float(result[i]) for i in range(12,18)]  
# dTcp = [0 for i in range(12,18)]  
# 定义用户坐标变量 
dUcs = [0, 0, 0, 0, 0, 0]


try:
    # 5. 循环推送实时目标位姿（target为外部实时输入）
    while True:
        target = get_vive_pose()
        target[0] -= 80
        target[2], target[3], target[4], target[5] = z, 180, -45, 0
        print("目标位姿：", target)
        print("tcp位姿：", dTcp)
        nRet = robot.HRIF_PushServoP(0,0,target, dTcp, dUcs)
        if nRet != 0:
            print(f"位姿推送失败，错误码：{nRet}")
            break
        # 按更新周期延时
        time.sleep(dServoTime)
except KeyboardInterrupt:
    print("遥操作被用户中断")