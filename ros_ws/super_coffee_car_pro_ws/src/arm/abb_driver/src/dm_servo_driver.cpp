/**
 * @file dm_servo_driver.cpp
 * @brief 达妙电机 MoveIt Servo 专用驱动节点 (并行开发版)
 * * 功能：
 * 1. 订阅 /servo_server/command_joint_state (接收 MoveIt 规划好的下一步位置)
 * 2. 结合重力补偿，使用 MIT 模式驱动电机
 * 3. 发布 /joint_states 给 MoveIt
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <array>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "dmbot_serial/protocol/damiao.h"

using namespace Eigen;

// ==========================================
// 1. 复用 teach.cpp 的参数 (确保手感一致)
// ==========================================
constexpr int DOF = 6;
constexpr double G = 9.8;

// 电机参数 (直接拷贝自你的 teach.cpp)
const double TAU_MAX[DOF] = {28.0, 28.0, 28.0, 10.0, 10.0, 10.0};
const int MOTOR_DIR[DOF] = {1, -1, 1, -1, -1, 1}; // 注意方向！
const double MASS[DOF] = {0.138, 1.379, 0.727, 0.370, 0.391, 0.179};
const Vector3d COM[DOF] = {
    Vector3d(0.0027728, 0.0, 0.017932),
    Vector3d(-0.0035914, -0.14996, 0.0),
    Vector3d(-0.00257, 0.09923, 0.051397),
    Vector3d(0.0034984, 0.028153, 0.0),
    Vector3d(-0.00068876, 0.076386, 0.0),
    Vector3d(0.0, 0.0065, 0.0)
};

// 遥操作时的 PD 参数 (可以根据响应速度微调，建议先用这套偏硬一点的)
const double KP[DOF] = {35.0, 60.0, 70.0, 45.0, 15.0, 15.0}; 
const double KD[DOF] = {1.5, 1.2, 1.5, 1.0, 1.0, 1.0};

// ==========================================
// 2. 全局变量
// ==========================================
std::shared_ptr<damiao::Motor_Control> g_motor_ctrl;
std::array<double, DOF> g_target_pos; // 目标位置
bool g_has_command = false;           // 是否收到过指令
ros::Publisher g_pub_js;

// ==========================================
// 3. 辅助函数 (矩阵运算、限幅、重力补偿)
// ==========================================
inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
Matrix3d rotX(double t) { Matrix3d R; double c=cos(t), s=sin(t); R << 1,0,0, 0,c,-s, 0,s,c; return R; }
Matrix3d rotY(double t) { Matrix3d R; double c=cos(t), s=sin(t); R << c,0,s, 0,1,0, -s,0,c; return R; }
Matrix3d rotZ(double t) { Matrix3d R; double c=cos(t), s=sin(t); R << c,-s,0, s,c,0, 0,0,1; return R; }

// 重力补偿核心算法 (和 teach.cpp 一模一样)
void computeGravityCompensation(const std::array<double, DOF>& q, std::array<double, DOF>& tau_g) {
    tau_g.fill(0.0);
    Matrix4d T[DOF + 1]; T[0] = Matrix4d::Identity();
    // J1
    Matrix4d T1 = Matrix4d::Identity(); T1.block<3,3>(0,0) = rotZ(q[0]); T1(2,3) = 0.06475; T[1] = T[0] * T1;
    // J2
    Matrix4d T2 = Matrix4d::Identity(); T2.block<3,3>(0,0) = rotX(q[1]); T2(0,3) = 0.00325; T2(2,3) = 0.04575; T[2] = T[1] * T2;
    // J3
    Matrix4d T3 = Matrix4d::Identity(); T3.block<3,3>(0,0) = rotX(q[2]); T3(1,3) = -0.3; T[3] = T[2] * T3;
    // J4
    Matrix4d T4 = Matrix4d::Identity(); T4.block<3,3>(0,0) = rotY(-q[3]); T4(0,3) = -0.00325; T4(1,3) = 0.218; T4(2,3) = 0.065; T[4] = T[3] * T4;
    // J5
    Matrix4d T5 = Matrix4d::Identity(); T5.block<3,3>(0,0) = rotX(q[4]); T5(0,3) = -0.00275; T5(1,3) = 0.036; T[5] = T[4] * T5;
    // J6
    Matrix4d T6 = Matrix4d::Identity(); T6.block<3,3>(0,0) = rotY(-q[5]); T6(0,3) = 0.00275; T6(1,3) = 0.082; T[6] = T[5] * T6;
    
    Vector3d axis[DOF] = {{0,0,1}, {1,0,0}, {1,0,0}, {0,-1,0}, {1,0,0}, {0,-1,0}};
    Vector3d g_world(0, 0, -G);
    for (int link = 0; link < DOF; link++) {
        Vector4d com_local(COM[link](0), COM[link](1), COM[link](2), 1.0);
        Vector4d com_world = T[link + 1] * com_local;
        Vector3d p_com(com_world(0), com_world(1), com_world(2));
        Vector3d F = MASS[link] * g_world;
        for (int j = 0; j <= link; j++) {
            Vector3d p_joint(T[j+1](0,3), T[j+1](1,3), T[j+1](2,3));
            Vector3d r = p_com - p_joint;
            Vector3d torque = r.cross(F);
            Vector3d axis_world = T[j+1].block<3,3>(0,0) * axis[j];
            tau_g[j] += torque.dot(axis_world);
        }
    }
    for (int i = 0; i < DOF; i++) tau_g[i] = -tau_g[i];
}

// 电机底层操作
bool initMotors() {
    std::vector<damiao::DmActData> data;
    for (int i = 0; i < 3; ++i) data.push_back({damiao::DM4340, damiao::MIT_MODE, (uint16_t)(i+1), (uint16_t)(0x11+i)});
    for (int i = 3; i < 6; ++i) data.push_back({damiao::DM4310, damiao::MIT_MODE, (uint16_t)(i+1), (uint16_t)(0x11+i)});
    try {
        g_motor_ctrl = std::make_shared<damiao::Motor_Control>(1000000, 5000000, "14AA044B241402B10DDBDAFE448040BB", &data);
        ros::Duration(0.5).sleep();
        for (int i = 0; i < DOF; ++i) {
            g_motor_ctrl->switchControlMode(*g_motor_ctrl->getMotor(i+1), damiao::MIT);
            usleep(5000);
        }
        return true;
    } catch (...) { return false; }
}

void readMotors(std::array<double, DOF>& pos, std::array<double, DOF>& vel) {
    for (int i = 0; i < DOF; ++i) g_motor_ctrl->refresh_motor_status(*g_motor_ctrl->getMotor(i+1));
    usleep(500); // 稍微给点时间
    for (int i = 0; i < DOF; ++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) { pos[i] = m->Get_Position() * MOTOR_DIR[i]; vel[i] = m->Get_Velocity() * MOTOR_DIR[i]; }
    }
}

void sendMIT(int joint_idx, double kp, double kd, double pos, double vel, double tau) {
    auto m = g_motor_ctrl->getMotor(joint_idx + 1);
    if (m) {
        g_motor_ctrl->control_mit(*m, kp, kd, pos * MOTOR_DIR[joint_idx], vel * MOTOR_DIR[joint_idx], clamp(tau * MOTOR_DIR[joint_idx], -TAU_MAX[joint_idx], TAU_MAX[joint_idx]));
    }
}

void sendZero() {
    for (int i = 0; i < DOF; ++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) g_motor_ctrl->control_mit(*m, 0, 0, 0, 0, 0);
    }
}

// ==========================================
// 4. 回调函数：接收 Servo 指令
// ==========================================
void commandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != DOF) {
        ROS_WARN_THROTTLE(1.0, "Received invalid command size: %lu", msg->data.size());
        return;
    }
    // 更新全局目标
    for (int i = 0; i < DOF; ++i) {
        g_target_pos[i] = msg->data[i];
    }
    g_has_command = true;
}

// ==========================================
// 5. 主函数
// ==========================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "dm_servo_driver_node");
    ros::NodeHandle nh;

    if (!initMotors()) {
        ROS_ERROR("Failed to init motors!");
        return -1;
    }
    ROS_INFO("Motors initialized. Waiting for Servo commands...");

    // 订阅 MoveIt Servo 的输出
    ros::Subscriber sub_cmd = nh.subscribe("/servo_server/command_joint_state", 1, commandCallback);
    
    // 发布关节状态 (给 MoveIt 用)
    g_pub_js = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // 初始化目标位置为当前位置
    std::array<double, DOF> current_pos, current_vel;
    readMotors(current_pos, current_vel);
    g_target_pos = current_pos;

    ros::Rate rate(100); // 500Hz 控制闭环

    while (ros::ok()) {
        // --- 1. 读取状态 ---
        readMotors(current_pos, current_vel);

        // --- 2. 发布 JointState ---
        sensor_msgs::JointState js_msg;
        js_msg.header.stamp = ros::Time::now();
        // 这里的名字必须和你的 URDF 对应！通常是 joint1 ~ joint6
        js_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}; 
        js_msg.position.resize(DOF);
        js_msg.velocity.resize(DOF);
        for(int i=0; i<DOF; ++i) {
            js_msg.position[i] = current_pos[i];
            js_msg.velocity[i] = current_vel[i];
        }
        g_pub_js.publish(js_msg);

        // --- 3. 计算重力补偿 ---
        std::array<double, DOF> tau_g;
        computeGravityCompensation(current_pos, tau_g);

        // --- 4. 发送控制 ---
        if (g_has_command) {
            // 有指令：位置跟踪 + 重力补偿
            for (int i = 0; i < DOF; ++i) {
                sendMIT(i, KP[i], KD[i], g_target_pos[i], 0.0, tau_g[i]);
            }
        } else {
            // 无指令：位置保持 + 重力补偿 (相当于“锁住”在当前位置)
            for (int i = 0; i < DOF; ++i) {
                // 用稍软一点的刚度锁住
                sendMIT(i, 20.0, 1.0, current_pos[i], 0.0, tau_g[i]);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    sendZero();
    return 0;
}