#include "../include/amr_client.h"
AMRClient::AMRClient():m_state(waitingForHead){}
AMRClient::~AMRClient() //析构函数，释放io端口设备
{
    boost::mutex::scoped_lock look(mutex_);

    // recv_flag_ = false;
    if(sp_)
    {
        sp_->cancel();
        sp_->close();
        sp_.reset();
    }
    io_service_.stop();
    io_service_.reset();
}
/**
 * @brief 主循环函数，参数等获取初始化值，并启动控制线程
 *        获取Launch文件参数
 * 
 * 
 */
void AMRClient::loop() 
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_("~");

    //获取各运行参数初始化值
    nh_p_.param<std::string>("version",code_version_,std::string("V1.0.0"));

    nh_p_.param<std::string>("port_name",port_name_,std::string("/dev/ttyUSB0"));
    nh_p_.param<int>("baud_rate",baud_rate_,115200);

    nh_p_.param<std::string>("odom_frame",odom_frame_,std::string("odom"));
    nh_p_.param<std::string>("base_frame",base_frame_,std::string("base_footprint"));
    nh_p_.param<std::string>("imu_frame",imu_frame_,std::string("base_imu_link"));

    nh_p_.param<int>("control_rate",control_rate_,20);

    //如果不接收底层反馈的里程计信息，屏蔽相关参数
    #ifndef _ACCEPT_ODOM_MSG 
    //里程计所需参数： 电机参数包括编码器分辨率、减速比，机器人本体参数包括轮径、轮轴距
        nh_p_.param<double>("gear_reduction",gear_reduction_mec_,1.0);
        nh_p_.param<double>("encoder_resolution",encoder_resolution_mec_,1600);
        nh_p_.param<double>("wheel_diameter",wheel_diameter_mec_,0.15);
        nh_p_.param<double>("wheel_base_dis",wheel_base_dis_,0.15);
        
        nh_p_.param<double>("linear_correction_factor",linear_correction_factor_,1.0);
        nh_p_.param<double>("angular_correction_factor",angular_correction_factor_,1.0);

    #endif 
    nh_p_.param<bool>("publish_odom_transform",publish_odom_transform_,true);  
    nh_p_.param<bool>("odom_reset",odom_reset_,false);

}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool AMRClient::initRobot()
{
    if(sp_)
    {
        ROS_ERROR("The SerialPort is already opened!");
        return false;
    }
     sp_ = serialp_ptr(new boost::asio::serial_port(io_service_));
     sp_->open(port_name_,ec_);
     if(ec_)
     {
        ROS_ERROR_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
        return false;
     }
    sp_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    sp_->set_option(boost::asio::serial_port_base::character_size(8));
    sp_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    sp_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    sp_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    // uint8_t data[11];
    // data[0] = head1;
    // data[1] = head2;
    // data[2] = 0x0b;
    // data[3] = sendType_pid;
    // data[4] = (p_send_ >> 8) & 0xff;
    // data[5] = p_send_ & 0xff;
    // data[6] = (i_send_ >> 8) & 0xff;
    // data[7] = i_send_ & 0xff;
    // data[8] = (d_send_ >> 8) & 0xff;
    // data[9] = d_send_ & 0xff;
    // check_sum(data,10,data[10]);
    // boost::asio::write(*sp_.get(),boost::asio::buffer(data,11),ec_);

    return true;
}


//cmd_vel话题回调函数
void AMRClient::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    try
    {
        cmd_vel_mutex_.lock();
        last_twist_time_ = ros::Time::now();
        current_twist_ = *msg.get();
        cmd_vel_mutex_.unlock();
    }
    catch(...)
    {
        cmd_vel_mutex_.unlock();
    }
}

void AMRClient::send_speed_data(const ros::TimerEvent&)
{
    U_float64_t x_speed,yaw_speed;
    if((ros::Time::now() - last_twist_time_).toSec() <= 1.0)
    {
        x_speed.value = current_twist_.linear.x;
        yaw_speed.value = current_twist_.angular.z;
    }
    else
    {
        x_speed.value = 0;
        yaw_speed.value = 0;
    }

    if((ros::Time::now() - now_).toSec() >=1)
    {
        ROS_WARN_THROTTLE(1,"Didn't received odom data,Please check your connection!");
    }
    uint8_t data[CmdRobotVelSize+3];
    data[0] = SendHeader;
    data[1] = CmdRobotVel;
    data[2] = CmdRobotVelSize;

    data[3] = x_speed.bit[0];
    data[4] = x_speed.bit[1];
    data[5] = x_speed.bit[2];
    data[6] = x_speed.bit[3];
    data[7] = x_speed.bit[4];
    data[8] = x_speed.bit[5];
    data[9] = x_speed.bit[6];
    data[10] =x_speed.bit[7];
    
    data[11] = yaw_speed.bit[0];
    data[12] = yaw_speed.bit[1];
    data[13] = yaw_speed.bit[2];
    data[14] = yaw_speed.bit[3];
    data[15] = yaw_speed.bit[4];
    data[16] = yaw_speed.bit[5];
    data[17] = yaw_speed.bit[6];
    data[18] = yaw_speed.bit[7];

    Append_CRC8_Check_Sum(data,CmdRobotVelSize+3);

    boost::asio::write(*sp_.get(),boost::asio::buffer(data,(CmdRobotVelSize+3)),ec_);
    ROS_DEBUG_STREAM("send speed -> Linear: " << x_speed.value <<"; Angular: " << yaw_speed.value);
}

/**
 * @brief 根据消息帧和长度，查表获取校验码
 * 
 * @param Data 消息帧
 * @param Length 长度( Data )
 * @param CRC8 校验码
 * @return uint8_t 校验码
 */
uint8_t AMRClient::Get_CRC8_Check_Sum(uint8_t *Data, uint8_t Length, uint8_t CRC8)
{
	uint8_t Index;
	while (Length--)
	{
		Index = CRC8 ^ (*Data++);
		CRC8 = CRC8_TABLE[Index];
	}
	return(CRC8);
}

/**
 * @brief 检验消息帧CRC校验码正确性
 * 
 * @param Data 消息帧
 * @param Length 长度（Data + checksum）
 * @return uint8_t 1：校验成功 ，0：校验失败
 */
uint8_t AMRClient::Verify_CRC8_Check_Sum(uint8_t *Data, uint8_t Length)
{
	uint8_t Expected = 0;
	if ((Data == 0) || (Length <= 2))
		return 0;
	Expected = AMRClient::Get_CRC8_Check_Sum(Data, Length - 1, CRC8_INIT);
	return (Expected == Data[Length - 1]);
}


/**
 * @brief 消息帧尾添加校验码
 * 
 * @param Data 消息帧
 * @param Length 长度（Data + checksum）
 */
void AMRClient::Append_CRC8_Check_Sum(uint8_t *Data, uint8_t Length)
{
	uint8_t CRC = 0;
	if ((Data == 0) || (Length <= 2))
		return;
	CRC = AMRClient::Get_CRC8_Check_Sum((uint8_t *)Data, Length - 1, CRC8_INIT);
	Data[Length - 1] = CRC;
}



/**
 * @brief 串口数据解包函数
 * 
 */
void AMRClient::recv_msg()
{
    uint8_t payload_size, check_num, buffer_data[MAX_PACKET_LEN],payload_type;
    m_state = waitingForHead;
    recv_flag = true;
    while(recv_flag)
    {
        switch (m_state)
        {
            case waitingForHead:
                check_num = 0x00;
                boost::asio::read(*sp_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
                m_state = buffer_data[0] == RecvHeader ? waitingForPayloadType : waitingForHead;
                if(m_state == waitingForHead)
                {
                    ROS_DEBUG_STREAM("recv header error : ->"<<(int)buffer_data[0]);
                }
                break;
            case waitingForPayloadType:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[1],1),ec_);
                payload_type = buffer_data[1];
                m_state = waitingForPayloadSize;
                break;
            case waitingForPayloadSize:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[2],1),ec_);
                payload_size = buffer_data[2];
                m_state = waitingForPayload;
                break;
            case waitingForPayload:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3],payload_size),ec_);
                m_state = waitingForChecksum;
                break;
            case waitingForChecksum:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3+payload_size],1),ec_);
                Get_CRC8_Check_Sum(buffer_data,payload_size + 3,check_num);
                m_state = buffer_data[payload_size + 3] == check_num ? handlePayload : waitingForHead;
                if(m_state == waitingForHead)
                {
                    ROS_DEBUG_STREAM("check sum error! recv is  : ->"<<(int)buffer_data[3+payload_size]<<" checksum in table is "<<check_num);
                }
                break;
            case handlePayload:
                distribute_data(payload_type, buffer_data);
                m_state = waitingForHead;
                break;
        
            default:
                m_state = waitingForHead;
                break;
        }
    }
}

//串口接收数据打包分发函数
void AMRClient::distribute_data(uint8_t msg_type, uint8_t* buffer_data)
{
    switch (msg_type)
    {
        case Battery:
            handle_Battery_data(buffer_data);
           break;
        case Inertia:
            handle_Inertia_data(buffer_data);
            break;
        case Sonar:
            handle_Sonar_data(buffer_data);
            break;
        case Odom:
            handle_Odom_data(buffer_data);
            break;
        case Magnetic:
            handle_Magnetic_data(buffer_data);
            break;
        case Cliff:
            handle_Cliff_data(buffer_data);
            //  ROS_WARN("sonar handle didn't finish");
            break;
        case Bumper:
            handle_Bumper_data(buffer_data);
            break;
        case WheelVel:
            handle_WheelVel_data(buffer_data);
            break;
        case Encoder:
            handle_Encoder_data(buffer_data);
            break;
        case ControlMode:
            handle_ControlMode_data(buffer_data);
            break;
        default:
            //ROS_WARN("Undefined data type : %d", msg_type);
            break;
    }
}

void AMRClient::handle_WheelVel_data(uint8_t* buffer_data){}
void AMRClient::handle_Encoder_data(uint8_t* buffer_data){}
void AMRClient::handle_Inertia_data(uint8_t* buffer_data){}
void AMRClient::handle_Bumper_data(uint8_t* buffer_data){}
void AMRClient::handle_Sonar_data(uint8_t* buffer_data){}
void AMRClient::handle_Magnetic_data(uint8_t* buffer_data){}
void AMRClient::handle_Cliff_data(uint8_t* buffer_data){}
void AMRClient::handle_Battery_data(uint8_t* buffer_data){}
void AMRClient::handle_ControlMode_data(uint8_t* buffer_data){}
void AMRClient::handle_Odom_data(uint8_t* buffer_data)
{
    for (int var = 0; var < 8; ++var) 
    {
        odomMsg_.TimeStamp.bit[var] = buffer_data[3+var];
        odomMsg_.Pose_X.bit[var] = buffer_data[3+8*1+var];
        odomMsg_.Pose_Y.bit[var] = buffer_data[3+8*2+var];
        odomMsg_.Pose_Theta.bit[var] = buffer_data[3+8*3+var];
        odomMsg_.Vx.bit[var] = buffer_data[3+8*4+var];
        odomMsg_.Vyaw.bit[var] = buffer_data[3+8*5+var];
    // ROS_INFO_STREAM("left["<<var<<"]:->"<<std::hex << 
    // static_cast<int>((int)recv_encoder_left.bit[var])<<
    // "\tright["<<var<<"]:->"<<std::hex << 
    // static_cast<int>((int)recv_encoder_right.bit[var]));
    }

    odom_.pose.pose.position.x = odomMsg_.Pose_X.value;
    odom_.pose.pose.position.y = odomMsg_.Pose_Y.value;
    odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomMsg_.Pose_Theta.value);
    odom_.twist.twist.linear.x = odomMsg_.Vx.value;
    odom_.twist.twist.angular.z = odomMsg_.Vyaw.value;


}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"AMR_node");
    AMRClient amrDriver;
    amrDriver.loop();
    return 0;
}