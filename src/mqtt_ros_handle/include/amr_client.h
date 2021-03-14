/**
 * @file amr_client.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-03-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>

#include <string>
#include <vector>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>

#define _ACCEPT_ODOM_MSG 
#define _DEBUG

// #define ROBOT_CRC8Check      Verify_CRC8_Check_Sum_Judge     //CRC8校验函数
// #define ROBOT_CRC8Append     Append_CRC8_Check_Sum_Judge    //CRC8校验码附加函数函数
#define MAX_PACKET_LEN  255
const uint8_t CRC8_INIT = 0xff;
const uint8_t CRC8_TABLE[256] =
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

class AMRClient
{

public:
    /**
     * @brief Construct a new AMRClient object
     * 
     */
    AMRClient();
    /**
     * @brief Destroy the AMRClient object
     * 
     */
    ~AMRClient();
    /**
     * @brief 主循环函数
     * 
     */
    void loop();

    typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;

    enum packetFinderState
    {
        waitingForHead,
        waitingForPayloadType,
        waitingForPayloadSize,
        waitingForPayload,
        waitingForChecksum,
        handlePayload
    };
    enum packetFinderState m_state;
    
    typedef struct
    {
        uint8_t pkt_header;
        uint8_t pkt_type;
        uint8_t pkt_payload_len;            /* 消息帧内容长度 */
        uint8_t pkt_buf[MAX_PACKET_LEN];    /* total frame buffer */
        uint8_t pkt_crc8; 
        
    }Packet_t;

    enum packetHeader{ SendHeader = 0xA5, RecvHeader = 0x5A };

    enum PayloadType {
    // 反馈数据流——高频
    WheelVel = 0x31, Encoder = 0x32, Inertia = 0x33, Bumper = 0x34, Sonar = 0x35, Magnetic = 0x36, Cliff = 0x37, Odom = 0x38,
    // 反馈数据流——低频
    HardwareMode = 0x40, ErrorFlag = 0x41, Battery = 0x42,
    // 服务数据流——请求响应
    HardwareVer = 0x50, FirmwareVer = 0x51, UniqueDeviceID = 0x52
    };

    enum ConfigureType {
    // 控制指令
    CmdWheelVel = 0x01, CmdRobotVel = 0x02,
    // 查询指令
    CheckFireware = 0x06,
    // 系统状态设置指令
    ControlMode = 0x11, MotorMode = 0x12, ErrorClear = 0x18, SysPower = 0x19,
    // 反馈参数设置指令
    OdomMode = 0x20, SonarMode = 0x21, CliffMode = 0x22, MagMode = 0x23, WriteToFlash = 0x25, LedMode = 0x26, DockingMode = 0x27
    };
    

    enum LedColour { Blue = 0x01, Red = 0x02, Green = 0x03, White = 0x04 };
    
    enum ControlMode { AGV_Control=0x00,  IPEGA_Control=0x01,  HOST_Control=0x02, TEST_Control=0x03 };

    struct imu_data
    {
        float angle_x;
        float angle_y;
        float angle_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float accel_x;
        float accel_y;
        float accel_z;
        float q0;
        float q1;
        float q2;
        float q3;
    };
    Packet_t cmdRobotVel_packet;
    
    enum PacketSize {
    CmdRobotVelSize = 16, CmdWheelVelSize = 8, 
    MotorModeSize = 1, ErrorClearSize = 1, SysPowerSize = 1,
    OdomModeSize = 6, SonarModeSize = 4, CliffModeSize = 2, 
    // MagModeSize = , LedModeSize = , DockingMode = , 
    };

    union U_int32_t
    {
        /* data */
        int32_t value;
	    int8_t bit[4];
    };
    union U_float64_t
    {
        /* data */
        double value;
	    int8_t bit[8];
    };
    union U_float32_t
    {
        /* data */
        float value;
	    int8_t bit[4];
    };
    
    typedef struct OdomPackage
    {
        U_float64_t TimeStamp;
        U_float64_t Vx;
        U_float64_t Vyaw;
        U_float64_t Pose_X;
        U_float64_t Pose_Y;
        U_float64_t Pose_Theta;
    }ODOM;

private:

    /**
     * @brief 机器人硬件串口及模型参数（里程计参数）初始化
     * 
     * @return true : 初始化成功
     * @return false : 初始化失败 
     */
    bool initRobot();

    /**
     * @brief 串口接收数据流函数
     * 
     */
    void recv_msg();

    /**
     * @brief 订阅速度指令回调函数
     * 
     * @param msg : 接收的速度消息
     */
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    

    void distribute_data(uint8_t msg_type, uint8_t* buffer_data);
    // void check_sum(uint8_t* data, size_t len, uint8_t& dest);
    uint8_t Get_CRC8_Check_Sum(uint8_t *Data, uint8_t Length, uint8_t CRC8);
    uint8_t Verify_CRC8_Check_Sum(uint8_t *Data, uint8_t Length);
    void Append_CRC8_Check_Sum(uint8_t *Data, uint8_t Length);

    /**
     * @brief 处理数据反馈函数
     * 
     * @param buffer_data 
     */
    void handle_WheelVel_data(uint8_t* buffer_data);
    void handle_Encoder_data(uint8_t* buffer_data);
    void handle_Inertia_data(uint8_t* buffer_data);
    void handle_Bumper_data(uint8_t* buffer_data);
    void handle_Sonar_data(uint8_t* buffer_data);
    void handle_Magnetic_data(uint8_t* buffer_data);
    void handle_Cliff_data(uint8_t* buffer_data);
    void handle_Odom_data(uint8_t* buffer_data);
    void handle_Battery_data(uint8_t* buffer_data);
    void handle_ControlMode_data(uint8_t* buffer_data);

    /**
     * @brief 发送控制指令函数
     * 
     * @param buffer_data 
     * @return true ： 设置成功
     * @return false ： 设置失败
     */
    
    void send_speed_data(const ros::TimerEvent&);//定时发送速度指令
    bool send_OdomMode_data(uint8_t* buffer_data);
    bool send_SonarMode_data(uint8_t* buffer_data);
    bool send_CliffMode_data(uint8_t* buffer_data);
    bool send_WriteToFlash_data(uint8_t* buffer_data);
    bool send_LedMode_data(uint8_t* buffer_data);
    bool send_DockingMode_data(uint8_t* buffer_data);


    bool send_ControlMode_data(uint8_t* buffer_data);   
    bool send_MotorMode_data(uint8_t* buffer_data);
    bool send_ErrorClear_data(uint8_t* buffer_data);
    bool send_SysPower_data(uint8_t* buffer_data);
    bool send_CheckFireware_data(uint8_t* buffer_data);
    /* data */

    struct imu_data imu_data_;
    sensor_msgs::Imu imu_pub_data_;
    std_msgs::Float32  battery_pub_data_;

    boost::mutex cmd_vel_mutex_;
    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    boost::mutex mutex_;
    serialp_ptr sp_;
    geometry_msgs::Twist current_twist_;
    nav_msgs::Odometry odom_;
    ros::Time last_twist_time_;
    ros::Time last_time_;
    ros::Time now_;
    ros::Time check_time_;

    ros::Publisher odom_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher sonar_pub_;

    ros::Publisher lvel_pub_;
    ros::Publisher rvel_pub_;

    ros::Subscriber cmd_sub_;
    std::string port_name_;
    
    int baud_rate_;

    std::string odom_frame_;
    std::string lidar_frame_;
    std::string imu_frame_;
    std::string base_frame_;
    std::string code_version_;
    int control_rate_;
    int sensor_rate_;
    bool publish_odom_transform_;
    bool odom_reset_;
    bool recv_flag;
    ODOM odomMsg_;
    


    //如果不接收底层反馈的里程计信息，屏蔽相关参数
#ifndef _ACCEPT_ODOM_MSG
    double gear_reduction_mec_;
    double encoder_resolution_mec_;
    double wheel_diameter_mec_;
    double wheel_base_dis_;
    double linear_correction_factor_;
    double angular_correction_factor_;
#endif


};
