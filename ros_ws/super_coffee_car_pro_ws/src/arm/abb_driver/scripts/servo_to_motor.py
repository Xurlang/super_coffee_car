#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class ServoToMotor:
    def __init__(self):
        rospy.init_node('servo_to_motor', anonymous=True)
        
        # 状态缓存
        self.arm_vel = [0.0, 0.0, 0.0, 0.0, 0.0]  # 前 5 轴 (MoveIt)
        self.gripper_vel = 0.0                    # 第 6 轴 (夹爪)

        # 发布：给真实电机的 6 轴数组
        self.pub_hardware = rospy.Publisher('/abb/cmd_servo_vel', Float64MultiArray, queue_size=1)

        # 订阅 1：MoveIt 的 5 轴输出
        rospy.Subscriber('/abb/cmd_servo_vel_raw', Float64MultiArray, self.arm_cb)
        
        # 订阅 2：手柄的原始输出 (修正这里：把 joy_callback 改成 joy_cb)
        rospy.Subscriber('/arm_joy_cmd', Joy, self.joy_cb)

        rospy.loginfo(">>> 硬件指令聚合节点 (Servo -> Motor) 启动成功 <<<")

    def arm_cb(self, msg):
        if len(msg.data) >= 5:
            self.arm_vel = list(msg.data[:5])
            self.send_to_hardware()

    def joy_cb(self, msg):
        # 检查十字键/摇杆索引，确保不会越界
        if len(msg.axes) > 6:
            # 假设你的夹爪由第 6 号轴控制
            self.gripper_vel = msg.axes[6] * 0.4
            self.send_to_hardware()

    def send_to_hardware(self):
        # 强行把 5 (机械臂) + 1 (夹爪) 轴拼接成一个 6 轴数组发给 C++ 驱动
        final_msg = Float64MultiArray()
        final_msg.data = self.arm_vel + [self.gripper_vel]
        self.pub_hardware.publish(final_msg)

if __name__ == '__main__':
    try:
        node = ServoToMotor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass