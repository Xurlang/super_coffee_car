#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from sensor_msgs.msg import Joy

def joy_callback(msg):
    # A 键 (Index 0)
    if msg.buttons[0] == 1:
        rospy.loginfo("Planning to GRASP...")
        arm_group.set_named_target("grasp")
        arm_group.go(wait=True)
    # B 键 (Index 1)
    elif msg.buttons[1] == 1:
        rospy.loginfo("Planning to HOME...")
        arm_group.set_named_target("home")
        arm_group.go(wait=True)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pose_executor_node')
    
    # 这里可能会报 joint1 not found，但没关系，它只在自己这个进程报
    try:
        arm_group = moveit_commander.MoveGroupCommander("robot_total")
    except:
        rospy.logerr("MoveGroup 还没准备好，请检查 URDF 关节名！")
        sys.exit(1)

    # 订阅分发后的手柄指令
    rospy.Subscriber('/arm_joy_cmd', Joy, joy_callback)
    rospy.spin()