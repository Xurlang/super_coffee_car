#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# MODIFICATION 1: Add Twist import for velocity control
from geometry_msgs.msg import PoseStamped, Twist 
from std_msgs.msg import Empty

# 引入 math 库用于计算 Pi
import math

class ClickAndRun:
    def __init__(self):
        rospy.init_node('click_and_run_node')
        
        self.waypoints = [] # 存储
        self.is_running = False
        
        # 1. 连接 Move Base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在连接导航服务器...")
        self.client.wait_for_server()
        rospy.loginfo("导航服务器已连接！")
        rospy.loginfo(">>> 请在 RViz 的 Tool Properties 中，将 '2D Nav Goal' 的 Topic 改为 '/waypoint'")
        rospy.loginfo(">>> 然后使用红色箭头在地图上标点（设定位置和朝向）。")

        # 2. 订阅 RViz 的红色箭头 (注意话题名)
        rospy.Subscriber('/waypoint', PoseStamped, self.waypoint_callback)
        
        # 3. 订阅“开始”指令
        rospy.Subscriber('/start_patrol', Empty, self.start_callback)
        
        # MODIFICATION 2: Add velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        
    def waypoint_callback(self, msg):
        """每在 RViz 拖出一个红色箭头，就会触发这个函数"""
        if self.is_running:
            rospy.logwarn("正在巡逻中，无法添加新点！")
            return
            
        # 直接把整个 Pose (包含 x, y 和 orientation 四元数) 存下来
        self.waypoints.append(msg.pose)
        
        # 打印一下，确认收到
        rospy.loginfo(f"已添加第 {len(self.waypoints)} 个目标点。")
        rospy.loginfo("继续添加，或者发送 /start_patrol 开始巡航")

    def start_callback(self, msg):
        if not self.waypoints:
            rospy.logwarn("还没设置目标点呢！请先在 RViz 里用红色箭头标点。")
            return
            
        if self.is_running:
            return

        self.is_running = True
        rospy.loginfo(f"=== 收到指令！开始执行 {len(self.waypoints)} 个目标的巡航任务 ===")
        
        for i, pose in enumerate(self.waypoints):
            success = self.move_to_goal(pose, i+1)
            if not success:
                rospy.logwarn("任务中断。")
                break
                
        rospy.loginfo("=== 所有任务已完成！等待新的标记... ===")
        self.waypoints = [] 
        self.is_running = False

    def move_to_goal(self, target_pose, index):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 直接使用录制的 Pose (这就包含了你刚才拖动箭头时的方向！)
        goal.target_pose.pose = target_pose

        rospy.loginfo(f"正在前往第 {index} 个点...")
        self.client.send_goal(goal)
        
        # 设置超时时间为 60 秒
        wait = self.client.wait_for_result(rospy.Duration(60))
        
        if not wait:
            # 如果超时，取消当前目标，并继续下一个 (返回 True 意味着继续)
            self.client.cancel_goal()
            rospy.logerr("超时！取消当前目标并继续。")
            return True 
        else:
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"到达第 {index} 个点！朝向已对齐。原地休息 0.2 秒...")
                rospy.sleep(0.2)
                # MODIFICATION 4: Call rotation
                self.rotate_360() 
                return True
            else:
                rospy.logwarn("未能到达目标。")
                return False

    # MODIFICATION 3: Add the rotation function
    def rotate_360(self):
        rospy.loginfo("开始原地旋转 360 度...")
        twist = Twist()
        
        # 旋转速度 (用户指定 0.6 rad/s)
        angular_speed = 0.6  # <--- 修改点 1: 速度改为 0.6 rad/s
        
        # 旋转时间 = 2*Pi / 速度 + 修正时间 (1.75秒)
        # 理论时长现在是 2*Pi / 0.6 约 10.47 秒。修正时间 1.75 秒保留。
        duration = (2 * math.pi) / angular_speed # <--- 修改点 2: 重新计算时长

        twist.angular.z = angular_speed
        
        rate = rospy.Rate(10) # 10Hz
        start_time = rospy.Time.now()
        
        # 在指定时间内发布速度指令，直到时间耗尽或 ROS 关闭
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止旋转
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("原地旋转完成。")

if __name__ == '__main__':
    try:
        node = ClickAndRun()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass