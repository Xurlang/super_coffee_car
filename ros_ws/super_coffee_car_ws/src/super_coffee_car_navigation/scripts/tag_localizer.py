#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import atexit
import os
import numpy as np


class TagLocalizer:
    def __init__(self):
        rospy.init_node('tag_map_locator', anonymous=True)

        self.tag_data = {}
        self.min_samples = 5

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # === 关键设置 ===
        self.target_frame = "map"
        self.force_source_frame = "camera_color_optical_frame"

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        atexit.register(self.save_results)

        rospy.loginfo("二维码定位节点启动！（已启用：离群点剔除 + 加权平均）")

    def tag_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_id = detection.id[0]

            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header

            if not pose_stamped.header.frame_id:
                pose_stamped.header.frame_id = "d435i_depth_optical_frame"

            pose_stamped.pose = detection.pose.pose.pose

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    pose_stamped.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.5)
                )

                pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

                x = pose_in_map.pose.position.x
                y = pose_in_map.pose.position.y
                z = pose_in_map.pose.position.z

                if tag_id not in self.tag_data:
                    self.tag_data[tag_id] = {'x': [], 'y': [], 'z': []}

                self.tag_data[tag_id]['x'].append(x)
                self.tag_data[tag_id]['y'].append(y)
                self.tag_data[tag_id]['z'].append(z)

                if len(self.tag_data[tag_id]['x']) % 20 == 0:
                    rospy.loginfo(
                        f"ID: {tag_id} 正在采集... 当前 Map 坐标: [{x:.2f}, {y:.2f}, {z:.2f}]")

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

    # ================== 新增：离群点剔除 + 加权平均 ==================
    def remove_outliers_and_weighted_avg(self, data_list):
        """
        使用 MAD（中位数绝对偏差）剔除离群点 + 加权平均
        """
        data = np.array(data_list)

        if len(data) < self.min_samples:
            return None, None

        median = np.median(data)
        abs_dev = np.abs(data - median)
        mad = np.median(abs_dev)

        if mad == 0:
            return np.mean(data), 0

        threshold = 2.5 * mad

        # 剔除离群点
        mask = abs_dev < threshold
        filtered = data[mask]

        if len(filtered) < self.min_samples:
            # 兜底：使用原始均值
            return np.mean(data), np.std(data)

        # 加权（离中位数越近权重越大）
        weights = 1 / (np.abs(filtered - median) + 1e-6)
        weighted_avg = np.sum(weights * filtered) / np.sum(weights)

        return weighted_avg, np.std(filtered)

    def save_results(self):
        output_file = os.path.join(os.path.expanduser("~"), "tag_results.txt")
        rospy.loginfo(f"--- 正在保存结果到: {output_file} ---")

        results = []

        for tag_id, data in self.tag_data.items():
            count = len(data['x'])
            if count < self.min_samples:
                continue

            final_x, std_x = self.remove_outliers_and_weighted_avg(data['x'])
            final_y, std_y = self.remove_outliers_and_weighted_avg(data['y'])
            final_z, std_z = self.remove_outliers_and_weighted_avg(data['z'])

            if final_x is None:
                continue

            results.append({
                'id': tag_id,
                'x': final_x,
                'y': final_y,
                'z': final_z,
                'count': count,
                'std_x': std_x
            })

        with open(output_file, 'w') as f:
            f.write("TagID, X, Y, Z, Samples, Stability\n")
            for r in results:
                f.write(
                    f"{r['id']},{r['x']:.4f},{r['y']:.4f},{r['z']:.4f},{r['count']},{r['std_x']:.4f}\n"
                )
                rospy.loginfo(
                    f"FINAL | ID: {r['id']} | X: {r['x']:.4f} | Y: {r['y']:.4f} | Z: {r['z']:.4f}")

if __name__ == '__main__':
    try:
        TagLocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass