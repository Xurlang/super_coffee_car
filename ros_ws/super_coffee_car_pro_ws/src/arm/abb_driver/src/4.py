#!/usr/bin/env python3 
from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, PositionIKRequest
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_msgs.msg import ObjectInfo
import math
from moveit_commander import MoveGroupCommander

# 全局变量
arm = None
ik_service = None

# ===================== 逆运动学函数 =====================
def compute_ik_from_pose(target_pose, timeout=5.0):

    global arm, ik_service
    
    try:
        # 等待IK服务
        if ik_service is None:
            rospy.loginfo("等待IK服务 /compute_ik ...")
            rospy.wait_for_service('/compute_ik', timeout=5.0)
            ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
            rospy.loginfo("IK服务已连接")
        
        # 构建IK请求
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.robot_state = RobotState()
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout = rospy.Duration(timeout)
        
        # 设置目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = target_pose
        ik_request.ik_request.pose_stamped = pose_stamped
        
        # 调用IK服务
        response = ik_service(ik_request)
        
        if response.error_code.val == response.error_code.SUCCESS:
            joint_names = response.solution.joint_state.name
            joint_values = list(response.solution.joint_state.position)
            
            # 只取arm组的关节
            arm_joint_names = arm.get_active_joints()
            arm_joint_values = []
            for name in arm_joint_names:
                if name in joint_names:
                    idx = joint_names.index(name)
                    arm_joint_values.append(joint_values[idx])
            
            rospy.loginfo("IK计算成功!")
            rospy.loginfo("关节角: %s" % arm_joint_values)
            return arm_joint_values
        else:
            rospy.logwarn("IK计算失败，错误码: %d" % response.error_code.val)
            return None
            
    except rospy.ServiceException as e:
        rospy.logerr("IK服务调用失败: %s" % e)
        return None
    except rospy.ROSException as e:
        rospy.logerr("等待IK服务超时: %s" % e)
        return None
    

def normalize_quaternion(qx, qy, qz, qw):
    """归一化四元数"""
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx/norm, qy/norm, qz/norm, qw/norm


def compute_ik_from_xyz_quat(x, y, z, qx, qy, qz, qw):
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw
    return compute_ik_from_pose(target_pose)


def compute_ik_from_pose_stamped(pose_stamped):
    
    return compute_ik_from_xyz_quat(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z,
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w
    )

# ===================== 运动控制 =====================
def movej_type(joint_positions, speed=0.5):

    """关节空间运动"""
    global arm
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    arm.set_joint_value_target(joint_positions)
    success = arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    return success


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_end_effector_pose_py')
    planning_group = "arm"

    input("Press Enter to continue...")
    arm = MoveGroupCommander(planning_group)
    
    current_pose = arm.get_current_pose()
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("当前末端位姿:")
    rospy.loginfo("  位置: x=%.4f, y=%.4f, z=%.4f" % (
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    ))
    rospy.loginfo("  姿态: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f" % (
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    ))
    rospy.loginfo("=" * 60)
    
    # 方法1: 使用新增的compute_ik_from_pose_stamped函数
    rospy.loginfo("方法1: 使用 compute_ik_from_pose_stamped")
    joints = compute_ik_from_pose_stamped(current_pose)
    
    if joints is not None:
        rospy.loginfo("计算得到的关节角:")
        print(joints)
        
        # 对比当前实际关节角
        current_joints = arm.get_current_joint_values()
        rospy.loginfo("当前实际关节角:")
        print(current_joints)
    else:
        rospy.logerr("IK计算失败!")
    
    joints[2] = joints[2] + 0.2
    print(joints)

    success = movej_type(joints)
    
    # 关闭MoveIt
    moveit_commander.roscpp_shutdown()
