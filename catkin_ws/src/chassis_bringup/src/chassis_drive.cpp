#include "chassis_drive.h"
#include "Quaternion_Solution.h"


// Instantiate an IMU object for storing sensor data
// 实例化IMU对象，用于存储传感器数据
sensor_msgs::Imu Mpu6050;

/**
 * @brief 主函数
 * @details ROS初始化，创建Robot_Control对象并进入主循环。
 * @param argc 参数个数
 * @param argv 参数数组
 * @return int 退出状态码
 */
int main(int argc, char** argv)
{
    // Initialize the ROS node and name it "chassis_drive"
    // ROS初始化 并设置节点名称为 "chassis_drive"
    ros::init(argc, argv, "chassis_drive");

    // Instantiate the main control object
    // 实例化主控制对象
    turn_on_robot Robot_Control;

    // Enter the main control loop for data processing and topic publishing
    // 进入主控制循环，处理数据采集和话题发布
    Robot_Control.Control();

    return 0;
}

/**
 * @brief 将两个字节（小端模式）合并为一个short类型整数。
 * @param Data_Low 低八位数据
 * @param Data_High 高八位数据
 * @return short 合并后的16位短整型数
 */
short turn_on_robot::Bytes_To_Short(uint8_t Data_Low, uint8_t Data_High)
{
    short transition_16;
    transition_16 = (Data_High << 8) | Data_Low;
    return transition_16;
}

/**
 * @brief 将来自下位机的里程计数据（两个字节）转换为m/s单位的浮点数。
 * @details 原始数据单位为mm/s。
 * @param Data_Low 低八位数据
 * @param Data_High 高八位数据
 * @return float 转换后的速度，单位m/s
 */
float turn_on_robot::Odom_Trans(uint8_t Data_Low, uint8_t Data_High)
{
    // Combine two bytes to a short, then convert from mm/s to m/s
    // 将两个字节合并为一个short，然后从 mm/s 转换为 m/s
    return Bytes_To_Short(Data_Low, Data_High) / 1000.0f;
}

/**
 * @brief 速度指令话题的回调函数。
 * @details接收/cmd_vel话题的速度指令，打包并通过串口发送给下位机。
 * @param twist_aux 包含线速度和角速度的Twist消息指针
 */
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist& twist_aux)
{
    // Prepare the data frame to be sent
    // 准备要发送的数据帧
    Send_Data.tx[0] = FRAME_HEADER; // Frame header
    Send_Data.tx[1] = 0;            // Reserved
    Send_Data.tx[2] = 0;            // Reserved

    // Convert velocity from m/s to mm/s and pack into bytes (Little Endian)
    // 将速度从 m/s 转换为 mm/s 并打包成字节（小端模式）
    short linear_x_mm = static_cast<short>(twist_aux.linear.x * 1000);
    Send_Data.tx[3] = linear_x_mm & 0xFF;
    Send_Data.tx[4] = (linear_x_mm >> 8) & 0xFF;

    short linear_y_mm = static_cast<short>(twist_aux.linear.y * 1000);
    Send_Data.tx[5] = linear_y_mm & 0xFF;
    Send_Data.tx[6] = (linear_y_mm >> 8) & 0xFF;

    short angular_z_mm = static_cast<short>(twist_aux.angular.z * 1000);
    Send_Data.tx[7] = angular_z_mm & 0xFF;
    Send_Data.tx[8] = (angular_z_mm >> 8) & 0xFF;

    // Calculate checksum and set the frame tail
    // 计算校验和并设置帧尾
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
    Send_Data.tx[10] = FRAME_TAIL;

    try
    {
        // Write the data frame to the serial port
        // 通过串口将数据帧写入下位机
        Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to send data through serial port. " << e.what());
    }
}

/**
 * @brief 发布IMU传感器数据。
 * @details 将从下位机获取的IMU数据填充到sensor_msgs::Imu消息中并发布。
 */
void turn_on_robot::Publish_ImuSensor()
{
    sensor_msgs::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = ros::Time::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id;

    // Fill orientation data (quaternion)
    // 填充姿态数据（四元数）
    Imu_Data_Pub.orientation = Mpu6050.orientation;
    Imu_Data_Pub.orientation_covariance[0] = 1e6;
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;

    // Fill angular velocity data
    // 填充角速度数据
    Imu_Data_Pub.angular_velocity = Mpu6050.angular_velocity;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;

    // Fill linear acceleration data
    // 填充线性加速度数据
    Imu_Data_Pub.linear_acceleration = Mpu6050.linear_acceleration;

    imu_publisher.publish(Imu_Data_Pub);
}

/**
 * @brief 发布里程计数据。
 * @details 结合机器人的位置、姿态和速度信息，发布nav_msgs::Odometry消息。
 */
void turn_on_robot::Publish_Odom()
{
    // Convert yaw angle to quaternion
    // 将Z轴偏航角转换为四元数
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id;

    // Set position
    // 设置位置
    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = 0.0; // Assuming 2D motion
     //姿态，通过 Z 轴转角转换的四元数
    odom.pose.pose.orientation = odom_quat;

    // Set velocity
    // 设置速度
    odom.twist.twist.linear.x = Robot_Vel.X;
    odom.twist.twist.linear.y = Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z;

    // Set covariance matrices based on movement state
    // 根据运动状态设置协方差矩阵
     //如果velocity 是零，说明编码器的误差会比较小，认为编码器数据更可靠
    if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
    {
        // Use covariance for stationary state
        // 使用静止状态的协方差
        memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
        memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    }
    else
    {
        //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
        // Use covariance for moving state
        // 使用运动状态的协方差
        memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
    }

    odom_publisher.publish(odom);
}

/**
 * @brief 发布电池电压信息。
 * @details 定期发布std_msgs::Float32类型的电池电压数据。
 */
void turn_on_robot::Publish_Voltage()
{
    std_msgs::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    // Publish voltage at a lower frequency
    // 以较低频率发布电压
    if (Count_Voltage_Pub++ > 10)
    {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher.publish(voltage_msgs);
    }
}

/**
 * @brief 计算BCC（块校验字符）校验和。
 * @details 通过对数据进行逐位异或运算来生成校验和。
 * @param Count_Number 参与校验的数据字节数
 * @param mode 校验模式（0: 接收数据, 1: 发送数据）
 * @return unsigned char 计算出的校验和
 */
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
    unsigned char check_sum = 0;
    const unsigned char* data_ptr = (mode == SEND_DATA_CHECK) ? Send_Data.tx : Receive_Data.rx;

    for (int k = 0; k < Count_Number; k++)
    {
        check_sum ^= data_ptr[k];
    }
    return check_sum;
}

/**
 * @brief 从串口读取并解析来自下位机的传感器数据。
 * @details 采用状态机的方式逐字节读取数据，验证帧头、帧尾和校验和，成功后解析数据。
 * @return bool 如果成功接收并解析了一帧有效数据，则返回true，否则返回false。
 */
bool turn_on_robot::Get_Sensor_Data()
{
    uint8_t current_byte;
    static int count = 0; // State variable to count received bytes // 用于计算接收字节的状态变量

    // Read one byte at a time from the serial port
    // 一次从串口读取一个字节
    if (Stm32_Serial.read(&current_byte, 1) != 1)
    {
        return false; // Failed to read data
    }
    
    // State machine for frame parsing
    // 帧解析状态机
    if (count == 0 && current_byte != FRAME_HEADER)
    {
        // Waiting for frame header, do nothing if not found
        // 等待帧头，如果没找到则什么都不做
        return false;
    }

    // Store the byte in the buffer
    // 将字节存入缓冲区
    Receive_Data.rx[count] = current_byte;
    count++;

    // If a full frame is received, process it
    // 如果接收到完整的一帧，则进行处理
    if (count == 24)
    {
        count = 0; // Reset for the next frame // 为下一帧重置

        // Verify frame tail and checksum
        // 验证帧尾和校验和
        if (Receive_Data.rx[23] == FRAME_TAIL)
        {
            if (Check_Sum(22, READ_DATA_CHECK) == Receive_Data.rx[22])
            {
                // --- Data Parsing ---
                // --- 数据解析 ---
                Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
                Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
                Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

                // IMU Data
                Mpu6050_Data.accele_x_data = Bytes_To_Short(Receive_Data.rx[8], Receive_Data.rx[9]);
                Mpu6050_Data.accele_y_data = Bytes_To_Short(Receive_Data.rx[10], Receive_Data.rx[11]);
                Mpu6050_Data.accele_z_data = Bytes_To_Short(Receive_Data.rx[12], Receive_Data.rx[13]);
                Mpu6050_Data.gyros_x_data = Bytes_To_Short(Receive_Data.rx[14], Receive_Data.rx[15]);
                Mpu6050_Data.gyros_y_data = Bytes_To_Short(Receive_Data.rx[16], Receive_Data.rx[17]);
                Mpu6050_Data.gyros_z_data = Bytes_To_Short(Receive_Data.rx[18], Receive_Data.rx[19]);

                // Convert raw IMU data to standard units
                // 将原始IMU数据转换为标准单位
                Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
                Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
                Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
                Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

                // Battery Voltage (convert mV to V)
                // 电池电压 (从 mV 转换为 V)
                Power_voltage = Bytes_To_Short(Receive_Data.rx[20], Receive_Data.rx[21]) / 1000.0f;

                return true; // Successfully parsed a frame
            }
        }
    }
    return false;
}

/**
 * @brief 主控制循环。
 * @details 循环调用，用于获取传感器数据、计算里程计、发布话题。
 */
void turn_on_robot::Control()
{
    while (ros::ok())
    {
        if (Get_Sensor_Data())
        {
            _Now = ros::Time::now();
            if (_Last_Time.is_zero()) _Last_Time = _Now;

            Sampling_Time = (_Now - _Last_Time).toSec();

            // Apply odometry correction scales
            // 应用里程计误差修正参数
            Robot_Vel.X *= odom_x_scale;
            Robot_Vel.Y *= odom_y_scale;
            Robot_Vel.Z *= (Robot_Vel.Z >= 0) ? odom_z_scale_positive : odom_z_scale_negative;

            // Integrate velocity to get position (odometry)
            // 积分速度以获取位置（里程计）
            double dx = (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
            double dy = (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
            double dz = Robot_Vel.Z * Sampling_Time;
            Robot_Pos.X += dx;
            Robot_Pos.Y += dy;
            Robot_Pos.Z += dz;

            // Calculate orientation from IMU data
            // 从IMU数据计算姿态
            Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

            // Publish all topics
            // 发布所有话题
            Publish_Odom();
            Publish_ImuSensor();
            Publish_Voltage();

            _Last_Time = _Now;
        }
        ros::spinOnce(); // Process callbacks
    }
}

/**
 * @brief 构造函数。
 * @details 用于初始化变量、ROS参数、发布者、订阅者和串口。
 */
turn_on_robot::turn_on_robot() : Sampling_Time(0), Power_voltage(0)
{
    // Clear all data structures
    // 清空所有数据结构
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

    ros::NodeHandle private_nh("~");

    // Load parameters from the ROS parameter server
    // 从ROS参数服务器加载参数
    private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/ttyACM0");
    private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200);
    private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom_combined");
    private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_footprint");
    private_nh.param<std::string>("gyro_frame_id", gyro_frame_id, "gyro_link");
    private_nh.param<float>("odom_x_scale", odom_x_scale, 1.0);
    private_nh.param<float>("odom_y_scale", odom_y_scale, 1.0);
    private_nh.param<float>("odom_z_scale_positive", odom_z_scale_positive, 1.0);
    private_nh.param<float>("odom_z_scale_negative", odom_z_scale_negative, 1.0);

    // Initialize publishers
    // 初始化发布者
    voltage_publisher = n.advertise<std_msgs::Float32>("PowerVoltage", 10);
    odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);
    imu_publisher = n.advertise<sensor_msgs::Imu>("/imu/imu_raw", 20);

    // Initialize subscriber
    // 初始化订阅者
    Cmd_Vel_Sub = n.subscribe("cmd_vel", 100, &turn_on_robot::Cmd_Vel_Callback, this);

    ROS_INFO_STREAM("ROS node initialized, attempting to open serial port...");

    // Initialize and open the serial port
    // 初始化并打开串口
    try
    {
        Stm32_Serial.setPort(usart_port_name);
        Stm32_Serial.setBaudrate(serial_baud_rate);
        serial::Timeout _time = serial::Timeout::simpleTimeout(1000); // 1-second timeout
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open serial port. Please check connection! Error: " << e.what());
    }

    if (Stm32_Serial.isOpen())
    {
        ROS_INFO_STREAM("Serial port opened successfully.");
    }
}

/**
 * @brief 析构函数。
 * @details 在对象销毁前，发送停止指令并关闭串口。
 */
turn_on_robot::~turn_on_robot()
{
    // Before shutting down, send a stop command to the robot
    // 在关闭前，向机器人发送停止指令
    Send_Data.tx[0] = FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;
    Send_Data.tx[3] = 0; // linear.x = 0
    Send_Data.tx[4] = 0;
    Send_Data.tx[5] = 0; // linear.y = 0
    Send_Data.tx[6] = 0;
    Send_Data.tx[7] = 0; // angular.z = 0
    Send_Data.tx[8] = 0;
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
    Send_Data.tx[10] = FRAME_TAIL;

    try
    {
        Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to send stop command. " << e.what());
    }

    Stm32_Serial.close();
    ROS_INFO_STREAM("Shutting down and closing serial port.");
}
