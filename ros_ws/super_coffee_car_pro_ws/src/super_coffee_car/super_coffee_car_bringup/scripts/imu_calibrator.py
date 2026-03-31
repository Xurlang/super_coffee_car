#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import numpy as np

class ImuCalibrator:
    def __init__(self):
        rospy.init_node('imu_calibrator')
        
        self.imu_sub = rospy.Subscriber('/livox/imu', Imu, self.imu_callback)
        self.imu_pub = rospy.Publisher('/imu/data_calibrated', Imu, queue_size=10)
        
        self.buffer_size = 200  # 采样前200帧来计算零偏
        self.buffer = []
        self.bias_z = 0.0
        self.is_calibrated = False
        
        rospy.loginfo("IMU Calibrator: Starting calibration... KEEP ROBOT STILL!")

    def imu_callback(self, msg):
        # 1. 校准阶段：收集数据
        if not self.is_calibrated:
            self.buffer.append(msg.angular_velocity.z)
            if len(self.buffer) >= self.buffer_size:
                self.bias_z = np.mean(self.buffer)
                self.is_calibrated = True
                rospy.loginfo(f"IMU Calibration Done! Z-Axis Bias: {self.bias_z:.6f} rad/s")
                rospy.loginfo("Now publishing calibrated data...")
            return

        # 2. 工作阶段：扣除零偏
        calibrated_msg = msg
        # 关键一步：原始数据 - 零偏
        calibrated_msg.angular_velocity.z -= self.bias_z
        
        # 重新发布
        self.imu_pub.publish(calibrated_msg)

if __name__ == '__main__':
    try:
        ImuCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass