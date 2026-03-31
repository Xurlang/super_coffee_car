#!/usr/bin/env python3
import rospy, sys
from moveit_commander import MoveGroupCommander
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_msgs.msg import ObjectInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from geometry_msgs.msg import Point, Quaternion
import math

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
move_group = MoveGroupCommander(planning_group)
rate = rospy.Rate(1) 

current_pose = move_group.get_current_pose()
x, y, z = current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z
x, y, z = round(x*1000,3), round(y*1000,3), round(z*1000,3)
rx, ry, rz = Quaternion2Euler(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)
print("(np.array("+str([x, y, z])+"),np.array("+str([rx, ry, rz])+")),",sep='')