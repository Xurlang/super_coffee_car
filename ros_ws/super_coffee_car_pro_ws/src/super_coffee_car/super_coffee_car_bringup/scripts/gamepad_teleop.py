#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class GamepadTeleop:
    def __init__(self):
        rospy.init_node('gamepad_teleop')
        
        # 1. 从参数服务器获取速度配置，预留外部修改窗口 (默认值分别为 0.2, 0.5, 60.0)
        self.max_linear_speed = rospy.get_param('~linear_speed', 0.2)
        self.max_angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.stepper_rpm = rospy.get_param('~stepper_rpm', 60.0)
        
        self.target_twist = Twist()
        self.target_step_rpm = 0.0
        
        # 2. 初始化发布者与手柄订阅者
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.step_pub = rospy.Publisher('/step_motor_rpm', Float64, queue_size=10)
        rospy.Subscriber('/chassis_joy_cmd', Joy, self.joy_callback)
        
        # 3. 设置 20Hz 定时器，确保持续向底层高频下发控制指令
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        
        rospy.loginfo("手柄节点启动成功 (按住 LB 开启使能控制)。")

    def joy_callback(self, msg):
        # 4. LB 使能键安全锁，松开时目标速度强制归零
        if len(msg.buttons) > 4 and msg.buttons[4] == 0:
            self.target_twist = Twist()
            self.target_step_rpm = 0.0
            return

        # 5. 解析摇杆输入并映射到底盘 Twist 消息
        self.target_twist.linear.x = msg.axes[1] * self.max_linear_speed
        self.target_twist.angular.z = msg.axes[3] * self.max_angular_speed
        
        # 6. 解析扳机状态，控制步进电机升降 (-0.5为防抖阈值)
        if msg.axes[2] < -0.5:
            self.target_step_rpm = -self.stepper_rpm
        elif msg.axes[5] < -0.5:
            self.target_step_rpm = self.stepper_rpm
        else:
            self.target_step_rpm = 0.0

    def timer_callback(self, event):
        # 7. 定时器回调函数，持续发布缓存中的最新状态
        self.cmd_pub.publish(self.target_twist)
        self.step_pub.publish(Float64(self.target_step_rpm))

if __name__ == '__main__':
    try:
        GamepadTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass