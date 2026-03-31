#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h> 
#include <nav_msgs/Odometry.h>         // <--- 新增: Odom 消息头文件
#include <tf/transform_broadcaster.h>  // <--- 新增: TF 广播头文件

serial::Serial ser;
ros::Publisher actual_twist_pub;



// --- 新增全局变量 ---
ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster; // 使用指针（为了在main中初始化）
double x = 0.0;     // 机器人X坐标
double y = 0.0;     // 机器人Y坐标
double th = 0.0;    // 机器人朝向 (弧度)
ros::Time last_time; // 上次计算的时间
// --------------------




void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (!ser.isOpen())
    {
        ROS_ERROR("Serial port not open");
        return;
    }

    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    uint8_t send_buf[10] = {0};

    send_buf[0] = 0xAA;
    memcpy(&send_buf[1], &linear_x, sizeof(float));
    memcpy(&send_buf[5], &angular_z, sizeof(float));
    send_buf[9] = 0x55;

    ser.write(send_buf, 10);

    ROS_INFO("Sent cmd_vel: linear_x=%.2f, angular_z=%.2f", linear_x, angular_z);
}

// 解析串口数据的函数（优化缓冲区溢出处理）
void parseSerialData(const uint8_t* data, size_t len) {
    static uint8_t recv_buffer[200];  // 静态缓冲区存储不完整帧
    static size_t buffer_len = 0;     // 缓冲区当前数据长度

    // 计算需要保留的旧数据长度：确保新数据能存入缓冲区
    // 若旧数据+新数据超过缓冲区大小，只保留末尾部分旧数据（优先保留最新的残留数据）
    if (buffer_len + len > sizeof(recv_buffer))
    {
        size_t keep_len = sizeof(recv_buffer) - len; // 为新数据预留空间后，能保留的旧数据长度
        if (keep_len > 0)
        {
            // 将旧数据的末尾keep_len字节前移到缓冲区头部
            memmove(recv_buffer, recv_buffer + (buffer_len - keep_len), keep_len);
        }
        buffer_len = keep_len; // 更新缓冲区长度为保留的旧数据长度
        ROS_INFO("Receive buffer nearly full, reserved %zu bytes old data", keep_len);
    }

    // 将新接收的数据存入缓冲区（此时已确保不会溢出）
    memcpy(recv_buffer + buffer_len, data, len);
    buffer_len += len;

    // 循环查找完整帧（帧头0xAA + 4字节线速度 + 4字节角速度 + 帧尾0x55，共10字节）
    while (buffer_len >= 10) {

        // ROS_INFO("enter paras loop\n");

        // 查找帧头0xAA和帧尾0x55的位置（确保完整帧）
        size_t frame_start = 0;
        while (frame_start <= buffer_len - 10) {
            if (recv_buffer[frame_start] == 0xAA && 
                recv_buffer[frame_start + 9] == 0x55) {
                break;
            }
            frame_start++;
        }

        // 未找到完整帧：将剩余数据前移，等待下一次接收
        if (frame_start > buffer_len - 10) {
            memmove(recv_buffer, recv_buffer + frame_start, buffer_len - frame_start);
            buffer_len -= frame_start;
            return;
        }

        // 解析完整帧数据
        float actual_linear_x;    // 线速度（m/s）
        float actual_angular_z;   // 角速度（rad/s）

        memcpy(&actual_linear_x, recv_buffer + frame_start + 1, 4);
        memcpy(&actual_angular_z, recv_buffer + frame_start + 5, 4);

        // 发布数据
        geometry_msgs::Twist actual_twist;
        actual_twist.linear.x = actual_linear_x;
        actual_twist.angular.z = actual_angular_z;
        actual_twist_pub.publish(actual_twist);





        // =======================================================
        // <--- 新增: 计算并发布 ODOM 和 TF ---
        // =======================================================

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        // 运动学积分 (使用STM32算好的速度)
        double delta_x = (actual_linear_x * cos(th)) * dt;
        double delta_y = (actual_linear_x * sin(th)) * dt;
        double delta_th = actual_angular_z * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // 1. 创建 TF 广播 (odom -> base_footprint)
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        odom_trans.transform.rotation = odom_quat;

        // 发送 TF
        // odom_broadcaster->sendTransform(odom_trans);

        // 2. 创建 Odometry 消息 (发布到 /odom 话题)
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        // 填充 Pose (位置)
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        // 填充 Twist (速度)
        odom_msg.twist.twist.linear.x = actual_linear_x;
        odom_msg.twist.twist.linear.y = 0.0; 
        odom_msg.twist.twist.angular.z = actual_angular_z;

        // 发布 Odom 消息
        odom_pub.publish(odom_msg);

        last_time = current_time;
        // =======================================================
        // <--- 新增结束 ---
        // =======================================================





        ROS_INFO("Received actual: linear=%f m/s, angular=%f rad/s\n", 
                 actual_linear_x, actual_angular_z);

        // 移除已解析的帧，保留剩余数据
        buffer_len -= (frame_start + 10);
        memmove(recv_buffer, recv_buffer + frame_start + 10, buffer_len);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_node");
    ros::NodeHandle nh;

    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 100, twistCallback);
    actual_twist_pub = nh.advertise<geometry_msgs::Twist>("/actual_twist", 100);  // 实际速度话题
    



    // <--- 新增 ---
    // 初始化 Odom 发布器
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom/wheel", 50); // 标准 /odom 话题
    // 初始化 TF 广播器
    // odom_broadcaster = new tf::TransformBroadcaster();
    // -----------------





    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException & e)
    {
        ROS_ERROR_STREAM("Unable to open serial port ");
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    uint8_t recv_buf[100];
    uint8_t recv_len = 0;
    ros::Rate loop_rate(100);




    // <--- 新增 ---
    last_time = ros::Time::now(); // 初始化时间戳
    // -----------------



    while (ros::ok())
    {
        if (ser.available()) 
        {
            size_t len = ser.read(recv_buf, sizeof(recv_buf));
            //OS_INFO("begin parse\n");
            parseSerialData(recv_buf, len);  // 解析接收的数据
            //ROS_INFO("end parse\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    if (ser.isOpen())
    {
        ser.close();
    }




    // delete odom_broadcaster; // <--- 新增: 释放内存




    return 0;
}