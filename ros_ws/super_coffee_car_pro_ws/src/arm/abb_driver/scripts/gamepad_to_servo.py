#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped

class GamepadAdapter:
    def __init__(self):
        rospy.init_node('gamepad_adapter', anonymous=True)
        
        # 1. 订阅：来自 teleop_twist_joy 的原始指令
        self.sub = rospy.Subscriber('/raw_cmd_vel', Twist, self.callback)
        
        # 2. 发布：给 MoveIt Servo 的指令
        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        
        # 3. 参考系：推荐用 base_link，这样推摇杆永远是相对于底座前后左右
        self.control_frame = "tcp_link"

        rospy.loginfo(f">>> 手柄适配器启动 | 参考系: {self.control_frame} <<<")

    def callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = self.control_frame

        # --- 1. 线性速度 ---
        stamped.twist.linear.x = msg.linear.x 
        stamped.twist.linear.y = msg.linear.y
        stamped.twist.linear.z = msg.linear.z

        # --- 2. 角速度 ---
        stamped.twist.angular.x = msg.angular.x
        stamped.twist.angular.y = msg.angular.y 
        stamped.twist.angular.z = msg.angular.z 
        self.pub.publish(stamped)

if __name__ == '__main__':
    try:
        node = GamepadAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# # !/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist, TwistStamped
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import Joy

# class GamepadAdapter:
#     def __init__(self):
#         rospy.init_node('gamepad_adapter', anonymous=True)
#         self.control_frame = "base_link"

#         # --- 状态缓存区 ---
#         self.arm_vel_ready = [0.0, 0.0, 0.0, 0.0, 0.0]  # 存 MoveIt 算出的 5 轴速度
#         self.gripper_vel = 0.0                          # 存十字键按出的 6 轴速度

#         # 1. 最终输出：发给真实的达妙底层驱动
#         self.pub_final = rospy.Publisher('/abb/cmd_servo_vel', Float64MultiArray, queue_size=1)

#         # 2. 中转输出：把摇杆的 XYZ 速度发给 MoveIt
#         self.pub_twist = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)

#         # 3. 监听：MoveIt 算好的前 5 轴速度 (对应你 YAML 里新改的话题)
#         self.sub_arm_servo = rospy.Subscriber('/abb/arm_servo_vel_raw', Float64MultiArray, self.arm_servo_cb)

#         # 4. 监听：手柄原始输入 (处理十字键)
#         self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)

#         # 5. 监听：手柄推摇杆产生的 Twist 速度
#         self.sub = rospy.Subscriber('/raw_cmd_vel', Twist, self.callback)

#         rospy.loginfo(f">>> 终极聚合适配器启动 | 参考系: {self.control_frame} <<<")

#     # --- 摇杆控制逻辑 (喂给 MoveIt) ---
#     def callback(self, msg):
#         stamped = TwistStamped()
#         stamped.header.stamp = rospy.Time.now()
#         stamped.header.frame_id = self.control_frame
#         stamped.twist = msg  
#         self.pub_twist.publish(stamped)

#     # --- 十字键控制逻辑 (直通底层) ---
#     def joy_callback(self, msg):
#         if len(msg.axes) > 6:
#             # 读取十字键，0.2 是安全速度
#             self.gripper_vel = msg.axes[6] * 0.2
#             # 按下十字键立刻触发发送，不管 MoveIt 活没活着
#             self.send_to_hardware()

#     # --- MoveIt 5轴接收逻辑 ---
#     def arm_servo_cb(self, arm_msg):
#         # 只要 MoveIt 发出前 5 轴速度，就更新缓存并触发发送
#         if len(arm_msg.data) >= 5:
#             self.arm_vel_ready = list(arm_msg.data[:5])
#             self.send_to_hardware()

#     # --- 核心：强制合并 5+1 发送给电机 ---
#     def send_to_hardware(self):
#         final_msg = Float64MultiArray()
#         # 将 5轴手臂 和 1轴夹爪 拼成一个 6 位数组
#         final_msg.data = self.arm_vel_ready + [self.gripper_vel]
#         self.pub_final.publish(final_msg)

# if __name__ == '__main__':
#     try:
#         node = GamepadAdapter()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass