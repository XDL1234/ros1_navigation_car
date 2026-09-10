#ifndef __CHASSIS_DRIVE_H_
#define __CHASSIS_DRIVE_H_

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

//================================================================================================================
// 宏定义 / Macro Definitions
//================================================================================================================

// --- 通信协议常量 / Communication Protocol Constants ---
#define FRAME_HEADER      0x7B  // 帧头 / Frame Header
#define FRAME_TAIL        0x7D  // 帧尾 / Frame Tail
#define RECEIVE_DATA_SIZE 24    // 接收数据帧长度 / Length of the received data frame
#define SEND_DATA_SIZE    11    // 发送数据帧长度 / Length of the sent data frame

// --- 校验模式 / Checksum Modes ---
#define SEND_DATA_CHECK   1     // 发送数据校验标志 / Flag for sending data checksum
#define READ_DATA_CHECK   0     // 接收数据校验标志 / Flag for receiving data checksum

// --- IMU 数据转换比率 / IMU Data Conversion Ratios ---
// 陀螺仪原始数据到弧度/秒的转换系数 (量程±500°/s)
// Gyroscope raw data to rad/s conversion factor (range ±500°/s)
#define GYROSCOPE_RATIO   0.00026644f

// 加速度计原始数据到米/秒^2的转换系数 (量程±2g)
// Accelerometer raw data to m/s^2 conversion factor (range ±2g)
#define ACCEl_RATIO       1671.84f

//================================================================================================================
// 全局变量与常量 / Global Variables and Constants
//================================================================================================================

// 外部IMU数据变量声明
// External declaration for IMU data variable
extern sensor_msgs::Imu Mpu6050;

// 里程计协方差矩阵 (用于robot_pose_ekf)
// Odometry covariance matrices (for robot_pose_ekf)
const double odom_pose_covariance[36]  = {1e-3,    0,    0,    0,    0,    0,
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3};
const double odom_pose_covariance2[36] = {1e-9,    0,    0,    0,    0,    0,
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9};
const double odom_twist_covariance[36] = {1e-3,    0,    0,    0,    0,    0,
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3};
const double odom_twist_covariance2[36]= {1e-9,    0,    0,    0,    0,    0,
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9};

//================================================================================================================
// 结构体定义 / Struct Definitions
//================================================================================================================

/**
 * @brief 存储机器人的速度和位置数据
 * @brief Structure to store robot's velocity and position data
 */
typedef struct __Vel_Pos_Data_
{
    float X; // X轴数据 / X-axis data
    float Y; // Y轴数据 / Y-axis data
    float Z; // Z轴数据 (位置时为角度，速度时为角速度) / Z-axis data (angle for position, angular velocity for velocity)
} Vel_Pos_Data;

/**
 * @brief 存储从下位机接收的原始IMU数据
 * @brief Structure to store raw IMU data received from the lower-level controller
 */
typedef struct __MPU6050_DATA_
{
    short accele_x_data; // X轴加速度 / X-axis acceleration
    short accele_y_data; // Y轴加速度 / Y-axis acceleration
    short accele_z_data; // Z轴加速度 / Z-axis acceleration
    short gyros_x_data;  // X轴角速度 / X-axis angular velocity
    short gyros_y_data;  // Y轴角速度 / Y-axis angular velocity
    short gyros_z_data;  // Z轴角速度 / Z-axis angular velocity
} MPU6050_DATA;

/**
 * @brief 发送给下位机的数据帧结构体
 * @brief Data frame structure for sending data to the lower-level controller
 */
typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE]; // 待发送的字节数组 / Byte array to be sent
} SEND_DATA;

/**
 * @brief 从下位机接收的数据帧结构体
 * @brief Data frame structure for receiving data from the lower-level controller
 */
typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE]; // 接收到的字节数组 / Received byte array
} RECEIVE_DATA;

//================================================================================================================
// 主控制类定义 / Main Control Class Definition
//================================================================================================================

/**
 * @class turn_on_robot
 * @brief 机器人底盘控制类
 * @details 负责与下位机串口通信、解析传感器数据、计算并发布里程计、IMU和电压等话题。
 * @class turn_on_robot
 * @brief Robot chassis control class
 * @details Responsible for serial communication with the lower-level controller, parsing sensor data,
 * and calculating and publishing topics such as odometry, IMU, and voltage.
 */
class turn_on_robot
{
public:
    /**
     * @brief 构造函数，用于初始化。
     * @brief Constructor for initialization.
     */
    turn_on_robot();

    /**
     * @brief 析构函数，用于资源释放。
     * @brief Destructor for resource release.
     */
    ~turn_on_robot();

    /**
     * @brief 主循环函数，持续处理数据。
     * @brief Main loop function for continuous data processing.
     */
    void Control();

    // 串口通信对象
    // Serial communication object
    serial::Serial Stm32_Serial;

private:
    // --- ROS 相关句柄和变量 / ROS Handles and Variables ---
    ros::NodeHandle n;
    ros::Time _Now, _Last_Time;
    float Sampling_Time;

    // --- ROS 订阅者与发布者 / ROS Subscribers and Publishers ---
    ros::Subscriber Cmd_Vel_Sub;
    ros::Publisher odom_publisher, imu_publisher, voltage_publisher;

    // --- 回调函数和话题发布函数 / Callback and Topic Publishing Functions ---
    void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);
    void Publish_Odom();
    void Publish_ImuSensor();
    void Publish_Voltage();

    // --- 数据处理函数 / Data Processing Functions ---
    bool Get_Sensor_Data();
    unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);

    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);
	short Bytes_To_Short(uint8_t Data_Low, uint8_t Data_High);

    // --- 参数和配置变量 / Parameters and Configuration Variables ---
    std::string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
    int serial_baud_rate;
    float odom_x_scale, odom_y_scale, odom_z_scale_positive, odom_z_scale_negative;

    // --- 数据存储结构体 / Data Storage Structs ---
    RECEIVE_DATA Receive_Data;
    SEND_DATA Send_Data;
    Vel_Pos_Data Robot_Pos;
    Vel_Pos_Data Robot_Vel;
    MPU6050_DATA Mpu6050_Data;
    float Power_voltage;
};

#endif // __CHASSIS_DRIVE_H_
