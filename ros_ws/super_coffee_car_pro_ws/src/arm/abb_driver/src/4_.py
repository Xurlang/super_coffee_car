#!/usr/bin/env python3 
from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, PositionIKRequest
import numpy as np
from scipy.spatial.transform import Rotation as R
import triad_openvr
import math
from moveit_commander import MoveGroupCommander

# 全局变量
arm = None
ik_service = None

# ===================== 逆运动学函数 =====================
def compute_ik_from_pose(target_pose, timeout=1.0):

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
        ik_request.ik_request.robot_state.joint_state.name = arm.get_active_joints()
        ik_request.ik_request.robot_state.joint_state.position = arm.get_current_joint_values()     # 6电机关节角
        # rospy.loginfo(ik_request.ik_request)
        # rospy.loginfo("111111111111111")
        # rospy.loginfo(ik_request.ik_request.robot_state)
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
        pose_stamped[0],
        pose_stamped[1],
        pose_stamped[2],
        pose_stamped[3],
        pose_stamped[4],
        pose_stamped[5],
        pose_stamped[6]
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

def Quaternion2Euler(x,y,z,w):
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def Euler2Quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return [qx, qy, qz, qw]

if __name__ == '__main__':
    R_viveOinBase = np.array([
                [-0.28370253, -0.03145289, -0.95839636],
                [-0.95839888, -0.02340242,  0.2844713 ],
                [-0.03137624,  0.99923123, -0.02350509]
                ])
    x = -2.801829020422635
    y = 3.441409949117306
    z = 0.9515264551434677
    R_viveOinBase = R_viveOinBase @ R.from_euler('x', x, degrees=True).as_matrix()
    R_viveOinBase = R_viveOinBase @ R.from_euler('y', y, degrees=True).as_matrix()
    R_viveOinBase = R_viveOinBase @ R.from_euler('z', z, degrees=True).as_matrix()
    # t_viveOinBase = np.array([-478.72913305, 171.26425215, 1157.82135641])       # 2026/2/9 原始数据
    t_viveOinBase = np.array([-0.46966167436, 0.1669760486, 1.15451525089])       # 2026/2/9 修正数据
    t_viveOinBase = np.array([-0.46766167436, 0.1669760486, 1.15451525089])       # 2026/2/10 修正数据
    t_viveOinBase = np.array([-0.47036167436, 0.1159760486, 1.18451525089])       # 2026/2/27 修正数据

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_end_effector_pose_py')
    planning_group = "arm"

    v = triad_openvr.triad_openvr()

    while True:
        if str(input("Press Enter to continue...")) == 'q':
            break

        original_tracker_pose = v.devices["tracker_1"].get_pose_euler()
        T = [original_tracker_pose[0],
            original_tracker_pose[1],
            original_tracker_pose[2]]
        R_euler = [float(original_tracker_pose[3])*180/math.pi, 
                    float(original_tracker_pose[4])*180/math.pi, 
                    float(original_tracker_pose[5])*180/math.pi]
        for i in range(3):
            T[i] += (0.200 * R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()[:, 2])[i]
        R_euler = R.from_matrix(R_viveOinBase @ R.from_euler('xyz', np.array(R_euler), degrees=True).as_matrix()).as_euler('xyz', degrees=True)
        T = R_viveOinBase @ np.array(T) + t_viveOinBase
        R_q = Euler2Quaternion(*R_euler)


        arm = MoveGroupCommander(planning_group)

        arm_pose = arm.get_current_pose()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("当前末端位姿:")
        rospy.loginfo("  位置: x=%.4f, y=%.4f, z=%.4f" % (
            arm_pose.pose.position.x,
            arm_pose.pose.position.y,
            arm_pose.pose.position.z
        ))
        rospy.loginfo("  姿态: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f" % (
            arm_pose.pose.orientation.x,
            arm_pose.pose.orientation.y,
            arm_pose.pose.orientation.z,
            arm_pose.pose.orientation.w
        ))
        rospy.loginfo("=" * 60)

        rospy.loginfo("Tracker位姿 (位置 + 欧拉角):")
        rospy.loginfo("  位置: x=%.4f, y=%.4f, z=%.4f" % (
            T[0],
            T[1],
            T[2]
        ))
        rospy.loginfo("  姿态: roll=%.4f, pitch=%.4f, yaw=%.4f" % (
            R_euler[0],
            R_euler[1],
            R_euler[2]
        ))
        rospy.loginfo("=" * 60)

        T[1] -= 0
        # current_pose = T + list(R_q)
        current_pose = T.tolist() + [float(arm_pose.pose.orientation.x), float(arm_pose.pose.orientation.y), float(arm_pose.pose.orientation.z), float(arm_pose.pose.orientation.w)]

        rospy.loginfo("=" * 60)
        
        # 方法1: 使用新增的compute_ik_from_pose_stamped函数
        rospy.loginfo("方法1: 使用 compute_ik_from_pose_stamped")
        joints = compute_ik_from_pose_stamped(current_pose)

        rospy.loginfo("从 x=%.4f, y=%.4f, z=%.4f, Qx=%.4f, Qy=%.4f, Qz=%.4f, Qw=%.4f" % (
            arm_pose.pose.position.x,
            arm_pose.pose.position.y,
            arm_pose.pose.position.z,
            arm_pose.pose.orientation.x,
            arm_pose.pose.orientation.y,
            arm_pose.pose.orientation.z,
            arm_pose.pose.orientation.w)
        )
        rospy.loginfo("到 x=%.4f, y=%.4f, z=%.4f, Qx=%.4f, Qy=%.4f, Qz=%.4f, Qw=%.4f" % (
            T[0], 
            T[1], 
            T[2],
            arm_pose.pose.orientation.x,
            arm_pose.pose.orientation.y,
            arm_pose.pose.orientation.z,
            arm_pose.pose.orientation.w)
        )
        
        if joints is not None:
            rospy.loginfo("计算得到的关节角:")
            print(joints)
            # if 4th joint rad2deg > 90 degrees, continue
            if (joints[3]*180/math.pi > 90):
                rospy.logerr("4th joint > 90 degrees, continue")
                continue

            # 对比当前实际关节角
            current_joints = arm.get_current_joint_values()
            rospy.loginfo("当前实际关节角:")
            print(current_joints)
            if str(input("Press Enter to execute...")) == 'q':
                break
            print(joints)
            # success = movej_type(joints)
        else:
            rospy.logerr("IK计算失败!")
    

    
    # 关闭MoveIt
    moveit_commander.roscpp_shutdown()
