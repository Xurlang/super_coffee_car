#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# [修改] 移除了 Twist 导入
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Empty

class ClickAndRun:
    def __init__(self):
        # [可选修改] 节点名字改一下，方便区分日志，不改也没事
        rospy.init_node('click_and_run_node_no_spin')
        
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
        
        # [修改] 移除了 self.cmd_vel_pub (不需要发速度)
        
    def waypoint_callback(self, msg):
        """每在 RViz 拖出一个红色箭头，就会触发这个函数"""
        if self.is_running:
            rospy.logwarn("正在巡逻中，无法添加新点！")
            return
            
        self.waypoints.append(msg.pose)
        
        rospy.loginfo(f"已添加第 {len(self.waypoints)} 个目标点。")
        rospy.loginfo("继续添加，或者发送 /start_patrol 开始巡航")

    def start_callback(self, msg):
        if not self.waypoints:
            rospy.logwarn("还没设置目标点呢！请先在 RViz 里用红色箭头标点。")
            return
            
        if self.is_running:
            return

        self.is_running = True
        rospy.loginfo(f"=== 收到指令！开始执行 {len(self.waypoints)} 个目标的巡航任务(无旋转版) ===")
        
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
        goal.target_pose.pose = target_pose

        rospy.loginfo(f"正在前往第 {index} 个点...")
        self.client.send_goal(goal)
        
        # 设置超时时间为 60 秒
        wait = self.client.wait_for_result(rospy.Duration(60))
        
        if not wait:
            self.client.cancel_goal()
            rospy.logerr("超时！取消当前目标并继续。")
            return True 
        else:
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"到达第 {index} 个点！原地休息 0.2 秒...")
                rospy.sleep(0.2)
                # [修改] 直接返回 True，不调用旋转
                return True
            else:
                rospy.logwarn("未能到达目标。")
                return False

    # [修改] 移除了 rotate_360 函数

if __name__ == '__main__':
    try:
        node = ClickAndRun()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass