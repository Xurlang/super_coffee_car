#!/usr/bin/env python3 
import rospy
from moveit_commander import MoveGroupCommander
import triad_openvr
import math
import time

# 全局变量
move_group = None
ik_service = None

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

def movej_type(joint_positions, speed=0.5):

    """关节空间运动"""
    global move_group
    move_group.set_goal_joint_tolerance(0.001)
    move_group.set_max_acceleration_scaling_factor(speed)
    move_group.set_max_velocity_scaling_factor(speed)
    move_group.set_joint_value_target(joint_positions)
    success = move_group.go(wait=True)
    move_group.stop()
    rospy.sleep(0.5)
    return success

rospy.init_node('get_end_effector_pose_py')

# client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
# if not client.wait_for_server(rospy.Duration(10)):
#     rospy.logerr("MoveGroup action server not available!")
#     exit(1)

planning_group = "arm"

print("11111111111111111111111")

v = triad_openvr.triad_openvr()

i=0
pose_tracker = []
pose_arm = []

joints = [
    [-30,-24,32,0,42,0],
    [29,-24,33,-1,42,0],
    [28,-3,32,38,42,1],
    [-39,-4,32,-25,37,1],
    [-57,-45,55,-48,44,1],
    [2,-34,72,-48,44,1],
    [88,-36,99,-49,35,1],
    [-10,-66,99,-48,35,2],
    [0,-33,62,-7,40,2],
    [0,-78,62,-7,40,3],
    [0,-2,62,-6,40,3],
    [0,-2,62,-7,-42,3],
    [-42,-2,62,106,-45,0],
    [-1,0,62,106,-44,1],
    [0,0,30,106,-44,1],
    [0,0,0,0,0,0]
]
for joint in joints:
    for i in range(6):
        joint[i] *= 0.01745329252

# while (True):
#     print("1")
i = 0

while (True):
    if input() == 'q':
        break

    move_group = MoveGroupCommander(planning_group)
    movej_type(joints[i])
    time.sleep(3)

    i+=1
    if i >= len(joints):
        break
    print(f"第{i}次：")

    original_tracker_pose = v.devices["tracker_1"].get_pose_euler()
    T = [original_tracker_pose[0]*1000,
        original_tracker_pose[1]*1000,
        original_tracker_pose[2]*1000]
    R_euler = [float(original_tracker_pose[3])*180/math.pi, 
                float(original_tracker_pose[4])*180/math.pi, 
                float(original_tracker_pose[5])*180/math.pi]
    txt = "(np.array(["
    txt +="%.3f, " % T[0]
    txt +="%.3f, " % T[1]
    txt +="%.3f" % T[2]
    txt += "]),np.array(["
    txt +="%.3f, " % R_euler[0]
    txt +="%.3f, " % R_euler[1]
    txt +="%.3f" % R_euler[2]
    txt += "])),"
    print(txt)
    pose_tracker.append(txt)

    current_pose = move_group.get_current_pose()
    # rospy.loginfo(f"参考坐标系: {current_pose.header.frame_id}")
    x, y, z = current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z
    rx, ry, rz = Quaternion2Euler(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)
    x, y, z = round(x*1000, 3), round(y*1000, 3), round(z*1000, 3)
    rx, ry, rz = round(rx*180/math.pi, 3), round(ry*180/math.pi, 3), round(rz*180/math.pi, 3)
    print("(np.array("+str([x, y, z])+"),np.array("+str([rx, ry, rz])+")),",sep='')
    pose_arm.append("(np.array("+str([x, y, z])+"),np.array("+str([rx, ry, rz])+")),")    


print(f"共{i}次")
for j in range(i):
    print(pose_tracker[j])
print()
for j in range(i):
    print(pose_arm[j])


rospy.loginfo("节点已退出。")