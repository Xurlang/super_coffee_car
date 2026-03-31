#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

class TeleopModeSwitch:
    def __init__(self):
        # 节点名也起个独一无二的
        rospy.init_node('teleop_mode_switch_node')
        
        # 初始化默认模式：底盘 (X)
        self.mode = 'chassis'
        
        # 转发出去的两个全新话题名
        self.pub_chassis = rospy.Publisher('/chassis_joy_cmd', Joy, queue_size=10)
        self.pub_arm = rospy.Publisher('/arm_joy_cmd', Joy, queue_size=10)
        
        # 接收手柄硬件发出的最原始指令
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        rospy.loginfo("🚥 遥控模式切换节点已启动！")
        rospy.loginfo("默认模式: [底盘] (按 X 键底盘，按 Y 键机械臂)")

    def joy_callback(self, msg):
        if len(msg.buttons) > 3:
            if msg.buttons[2] == 1 and self.mode != 'chassis': 
                self.mode = 'chassis'
                rospy.logwarn(">>> 模式已切换：[底盘控制] <<<")
            elif msg.buttons[3] == 1 and self.mode != 'arm': 
                self.mode = 'arm'
                rospy.logwarn(">>> 模式已切换：[机械臂控制] <<<")

        # 转发逻辑
        if self.mode == 'chassis':
            self.pub_chassis.publish(msg)
        elif self.mode == 'arm':
            self.pub_arm.publish(msg)

if __name__ == '__main__':
    try:
        TeleopModeSwitch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass