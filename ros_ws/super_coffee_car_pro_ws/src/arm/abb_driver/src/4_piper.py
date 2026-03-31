#!/usr/bin/env python3 
from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, PositionIKRequest
import numpy as np
from scipy.spatial.transform import Rotation as R
import triad_openvr
import math
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler

def convert_endpose(endpose):
    if len(endpose) == 6:
        x, y, z, roll, pitch, yaw = endpose
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        return [x, y, z, qx, qy, qz, qw]

    elif len(endpose) == 7:
        return endpose  # 直接返回四元数

    else:
        raise ValueError("Invalid endpose format! Must be 6 (Euler) or 7 (Quaternion) values.")
    
