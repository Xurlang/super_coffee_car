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

def _dh_transform(a, alpha, d, theta):
    """标准 DH 变换矩阵（a, alpha, d, theta）"""
    ca = math.cos(alpha); sa = math.sin(alpha)
    ct = math.cos(theta); st = math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,   sa,     ca,    d],
        [0.0,  0.0,    0.0,  1.0]
    ])

def _fk_from_dh(dh_params, thetas):
    """通过 DH 顺序计算 4x4 变换矩阵链。dh_params: list of (a,alpha,d,theta_offset)"""
    T = np.eye(4)
    for i, (a, alpha, d, theta_off) in enumerate(dh_params):
        theta = thetas[i] + theta_off
        T = T @ _dh_transform(a, alpha, d, theta)
    return T

# def compute_ik_dh_from_xyz_quat(x, y, z, qx, qy, qz, qw, dh_params, wrist_link_index=5):
#     """
#     使用 DH/解析法（带手腕中心分离）求逆解。
#     - dh_params: 长度6的列表，每项 (a, alpha, d, theta_offset)（单位：米、弧度）
#     - wrist_link_index: 最后一个用于计算 d6 的索引（通常为 5，表示第6关节）
#     返回: list of candidate joint-angle lists（rad），或 None
#     注意：需要根据你机器人手腕轴顺序调整最后一部的欧拉顺序。
#     """
#     # 目标位置与方向
#     p_end = np.array([x, y, z])
#     R_end = R.from_quat([qx, qy, qz, qw]).as_matrix()

#     # 从 dh_params 取 d6（最后一关节沿末端 z 的偏移，用于计算 wrist center）
#     a6, alpha6, d6, th_off6 = dh_params[wrist_link_index]

#     # 计算手腕中心
#     z_axis_end = R_end[:, 2]   # 末端 z 轴（DH 常规假设）
#     p_wc = p_end - d6 * z_axis_end

#     # 常用参数
#     a1, alpha1, d1, t1_off = dh_params[0]
#     a2, alpha2, d2, t2_off = dh_params[1]
#     a3, alpha3, d3, t3_off = dh_params[2]
#     # 注意：具体 a2, a3, d1 等名词与机器人 DH 约定有关，请按实际 DH 表替换

#     # 求 theta1（可能有两个解）
#     x_wc, y_wc, z_wc = p_wc
#     theta1_candidates = []
#     # 保守求解：两解
#     try:
#         theta1_1 = math.atan2(y_wc, x_wc)
#         theta1_2 = math.atan2(y_wc, x_wc) + math.pi
#         theta1_candidates = [theta1_1, theta1_2]
#     except Exception:
#         theta1_candidates = []

#     solutions = []
#     for th1 in theta1_candidates:
#         # 将手腕中心点转到第1关节坐标系（求平面长度）
#         # 计算在关节1后投影的 r 和 s
#         # 根据 DH 不同约定，下面计算可能需调整（这里采用常见的 a2 在 x2 方向，d1 为第一关节高度）
#         # 先把 p_wc 在基坐标系旋转到去掉 th1 的坐标系（第一关节旋转）
#         R1 = _dh_transform(0, 0, 0, th1)[:3, :3]
#         p1 = np.linalg.inv(R1) @ (p_wc - np.array([0.0, 0.0, d1]))
#         r = math.hypot(p1[0], p1[1])
#         s = p1[2]

#         # a2, a3 作为前臂长度（请根据你的 DH 填写）
#         L2 = a2
#         L3 = a3

#         # 余弦定理求 theta3
#         cos_phi = (r*r + s*s - L2*L2 - L3*L3) / (2 * L2 * L3)
#         if abs(cos_phi) > 1.0 + 1e-6:
#             # 不可达
#             continue
#         cos_phi = max(-1.0, min(1.0, cos_phi))
#         # 两组肘解
#         phi1 = math.acos(cos_phi)
#         for sign in [1, -1]:  # 肘上/肘下
#             theta3 = sign * phi1
#             # theta2
#             k1 = L2 + L3 * math.cos(theta3)
#             k2 = L3 * math.sin(theta3)
#             theta2 = math.atan2(s, r) - math.atan2(k2, k1)

#             # 组合前三关节（加上 DH 的 offset）
#             ths_123 = [np.array([th1 - t1_off, theta2 - t2_off, theta3 - t3_off])]

#             # 计算 R0_3
#             ths = [th1, theta2, theta3] + [0.0, 0.0, 0.0]
#             T0_3 = _fk_from_dh(dh_params[:3], ths[:3])
#             R0_3 = T0_3[:3, :3]

#             # 计算 R3_6
#             R3_6 = R0_3.T @ R_end

#             # 从 R3_6 提取 theta4,theta5,theta6（这里使用 'xyz' 顺序作为示例）
#             # 注意：若你的机器人手腕轴为不同序列，请修改 'xyz' 为正确序列，并可能调整符号
#             try:
#                 euler_345 = R.from_matrix(R3_6).as_euler('xyz')  # 返回 [theta4, theta5, theta6]
#                 theta4, theta5, theta6 = euler_345
#             except Exception:
#                 # 奇异或数值问题
#                 continue

#             # 合并并把 dh theta_offset 校正（若 dh 参数中的 theta_offset 是常量偏置）
#             joint_solution = [
#                 th1 - t1_off,
#                 theta2 - t2_off,
#                 theta3 - t3_off,
#                 theta4 - dh_params[3][3],
#                 theta5 - dh_params[4][3],
#                 theta6 - dh_params[5][3],
#             ]
#             # 归一化到 (-pi, pi]
#             joint_solution = [math.atan2(math.sin(j), math.cos(j)) for j in joint_solution]
#             solutions.append(joint_solution)

#     if len(solutions) == 0:
#         return None
#     return solutions  # 返回候选解列表，调用处可选第一个或按约束筛选

def compute_ik_dh_from_xyz_quat(x, y, z, qx, qy, qz, qw, dh_params, wrist_link_index=5):
    """
    使用 DH/解析法（带手腕中心分离）求逆解。
    - dh_params: 长度6的列表，每项 (a, alpha, d, theta_offset)（单位：米、弧度）
    - wrist_link_index: 最后一个用于计算 d6 的索引（通常为 5，表示第6关节）
    返回: list of candidate joint-angle lists（rad），或 None
    注意：需要根据你机器人手腕轴顺序调整最后一部的欧拉顺序。
    """
    solutions = [[0, 0, 0, 0, 0, 0] for i in range(4)]

    # 目标位置与方向
    p_end = np.array([x, y, z])
    R_end = R.from_quat([qx, qy, qz, qw]).as_matrix()

    # 常用参数
    # a1, alpha1, d1, t1_off = dh_params[0]
    # a2, alpha2, d2, t2_off = dh_params[1]
    # a3, alpha3, d3, t3_off = dh_params[2]
    # a4, alpha4, d4, t4_off = dh_params[3]
    # a5, alpha5, d5, t5_off = dh_params[4]
    # a6, alpha6, d6, t6_off = dh_params[5]
    theta1, alpha0, a0, d1 = dh_params[0]
    theta2, alpha1, a1, d2 = dh_params[1]
    theta3, alpha2, a2, d3 = dh_params[2]
    theta4, alpha3, a3, d4 = dh_params[3]
    theta5, alpha4, a4, d5 = dh_params[4]
    theta6, alpha5, a5, d6 = dh_params[5]
    # 计算手腕中心
    z_axis_end = R_end[:, 2]   # 末端 z 轴（DH 常规假设）
    p_wc = p_end - d6 * z_axis_end
    # 注意：具体 a2, a3, d1 等名词与机器人 DH 约定有关，请按实际 DH 表替换
    nx, ny, nz = R_end[:, 0]
    ax, ay, az = R_end[:, 2]
    px, py, pz = p_wc

    # 求 theta1（可能有两个解）
    # 保守求解：两解
    theta1_1 = math.atan2(py, px) - math.atan2(d2, math.sqrt(px**2 + py**2 - d2**2))
    theta1_2 = math.atan2(py, px) - math.atan2(d2, -math.sqrt(px**2 + py**2 - d2**2))
    # theta3
    k = (px**2 + py**2 + pz**2 - a2**2 - a3**2 - d2**2 - d4**2) / (2 * a2)
    theta3_1 = math.atan2(a3, d4) - math.atan2(k, math.sqrt(a3**2 + d4**2 - k**2))
    theta3_2 = math.atan2(a3, d4) - math.atan2(-k, math.sqrt(a3**2 + d4**2 - k**2))

    for index, solution in enumerate(solutions):
        if index < 2:
            solution[0] = theta1_1
        else:
            solution[0] = theta1_2
        if index % 2:
            solution[2] = theta3_1
        else:
            solution[2] = theta3_2
        s1 = math.sin(solution[0])
        c1 = math.cos(solution[0])
        s3 = math.sin(solution[2])
        c3 = math.cos(solution[2])
    # theta2
        solution[1] = (math.atan2(-(a3+a2*c3)*pz + (c1*px+s1*py)*(a2*s3-d4),
                                  (-d4+a2*s3)*pz + (c1*px+s1*py)*(a2*s3+a3))
                        -solution[2])
        
        s23 = math.sin(solution[1] + solution[2])
        c23 = math.cos(solution[1] + solution[2])
    # theta4
        solution[3] = math.atan2(-ax*s1+ay*c1, -ax*c1*c23-ay*s1*c23+az*s23)

        s4 = math.sin(solution[3])
        c4 = math.cos(solution[3])
    # theta5
        solution[4] = math.atan2(-ax*(c1*c23*c3+s1*s4)-ay*(s1*c23*c4-c1*s4)+az*(s23*c4),
                                 ax(-c1*s23)+ay(-s1*s23)+az(-c23))

        s5 = math.sin(solution[4])
        c5 = math.cos(solution[4])
    # theta6
        solution[5] = math.atan2(-nx*(c1*c23*s4-s1*c4)-ny*(s1*c23*s4+c1*c4)+nz*(s23*s4),
                                 nx*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+ny*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-nz*(s23*c4*c5+c23*s5))

    # 合并并把 dh theta_offset 校正（若 dh 参数中的 theta_offset 是常量偏置）
        solution = [
            solution[0] - t1_off,
            solution[1] - t2_off,
            solution[2] - t3_off,
            solution[3] - t4_off,
            solution[4] - t5_off,
            solution[5] - t6_off,
        ]

    if len(solutions) == 0:
        return None
    return solutions  # 返回候选解列表，调用处可选第一个或按约束筛选

def compute_ik_from_xyz_quat_dh(x, y, z, qx, qy, qz, qw):
    """
    示例包装函数：把 DH 参数填写为你机器人的实际值后直接调用。
    dh_params 格式示例（请替换为实际值）：
      dh_params = [
        (a1, alpha1, d1, theta1_offset),
        (a2, alpha2, d2, theta2_offset),
        ...
        (a6, alpha6, d6, theta6_offset),
      ]
    """
    # 下面的常量直接来自 URDF (/src/arm_dm_description/urdf/arm_dm_description.urdf)
    # 映射规则： a = origin.x, alpha = origin.rpy[0], d = origin.z, theta_offset = 0.0
    dh_params = [
        (0.0, 0.0, 0.06475, 0.0),     # joint1: origin xyz="0 0 0.06475" rpy="0 0 0"
        (0.00325, 0.0, 0.04575, 0.0), # joint2: origin xyz="0.00325 0 0.04575"
        (0.0, 0.0, 0.0, 0.0),         # joint3: origin xyz="0 -0.3 0"
        (-0.00325, 0.0, 0.065, 0.0),  # joint4: origin xyz="-0.00325 0.218 0.065"
        (-0.00275, 0.0, 0.0, 0.0),    # joint5: origin xyz="-0.00275 0.036 0"
        (0.00275, 0.0, 0.0, 0.0),     # joint6: origin xyz="0.00275 0.082 0"
    ]

    sols = compute_ik_dh_from_xyz_quat(x, y, z, qx, qy, qz, qw, dh_params)
    if sols is None:
        return None
    
    # 如果只有一个解，直接返回
    if len(sols) == 1:
        return sols[0]

    # 获取当前关节角作为参考（若 arm 未初始化则使用 0 向量）
    try:
        if arm is not None:
            current = arm.get_current_joint_values()
        else:
            current = [0.0] * len(sols[0])
    except Exception:
        current = [0.0] * len(sols[0])

    # 角度差归一化到 [-pi, pi]
    def angle_diff(a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    # 计算每个候选解到 current 的距离（平方和）
    best = None
    best_score = None
    for cand in sols:
        # 保证长度一致
        cand_vec = cand[:len(current)]
        # 计算平方和距离
        s = 0.0
        for ai, bi in zip(cand_vec, current):
            d = angle_diff(ai, bi)
            s += d * d
        if best_score is None or s < best_score:
            best_score = s
            best = cand_vec

    return best

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
    x = -2.54
    y = -4.8
    z = 1.747
    R_viveOinBase = R_viveOinBase @ R.from_euler('x', x, degrees=True).as_matrix()
    R_viveOinBase = R_viveOinBase @ R.from_euler('y', y, degrees=True).as_matrix()
    R_viveOinBase = R_viveOinBase @ R.from_euler('z', z, degrees=True).as_matrix()
    # t_viveOinBase = np.array([-478.72913305, 171.26425215, 1157.82135641])       # 2026/2/9 原始数据
    # t_viveOinBase = np.array([-0.46966167436, 0.1669760486, 1.15451525089])       # 2026/2/9 修正数据
    # t_viveOinBase = np.array([-0.46766167436, 0.1669760486, 1.15451525089])       # 2026/2/10 修正数据
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
        
        # # 方法1: 使用新增的compute_ik_from_pose_stamped函数
        # rospy.loginfo("方法1: 使用 compute_ik_from_pose_stamped")
        # joints = compute_ik_from_pose_stamped(current_pose)

        # 方法2：使用 compute_ik_from_xyz_quat_dh 函数（基于 DH 解析法）
        rospy.loginfo("方法2: 使用 compute_ik_from_xyz_quat_dh")
        joints = compute_ik_from_xyz_quat_dh(T[0], T[1], T[2], arm_pose.pose.orientation.x, arm_pose.pose.orientation.y, arm_pose.pose.orientation.z, arm_pose.pose.orientation.w)
        
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
        else:
            rospy.logerr("IK计算失败!")

    # joints[2] = joints[2] + 0.2
    # print(joints)

    # success = movej_type(joints)
    
    # 关闭MoveIt
    moveit_commander.roscpp_shutdown()
