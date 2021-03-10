#include <jsoncpp/json/json.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


class Listener
{
public:

    ros::Publisher mqtt_pub;
    Json::Value value;
    Json::Value return_value;
    Json::Reader reader;
    Json::FastWriter fast_writer;
    std_msgs::String pub_data;
    std::string sub_msg = "init init init";
    std::string pub_msg = "init init init";
    int count = 0;
public:
  void callback(const std_msgs::String::ConstPtr& msg);
  void handle_data()
    {

       if (reader.parse(sub_msg, value))
        {
            std::cout << "enter" << std::endl;
            std::string out = value["Action"].asString();
            std::cout << out << std::endl;
            value["Action"] = Json::Value("Start");
            pub_msg = fast_writer.write(value);
            std::cout << pub_msg << std::endl;
            pub_data.data = pub_msg;
            Listener::mqtt_pub.publish(pub_data);
            // 
            // std::cout << "enter" << std::endl;
            // value["Action"] = "Start";
            // std::string out = value["Action"].asString();
            // std::cout << out << std::endl;
        }
    }
};


void Listener::callback(const std_msgs::String::ConstPtr& msg)
{
    // ROS_INFO("I heard: %s", msg->data.c_str());
    sub_msg = msg->data;
    std::cout << "I heard data is :    " <<sub_msg << std::endl;
    
    // std::stringstream ss;
    // ss << msg->data.c_str();
    // ss >> copy_data;
    // std::cout <<"copy_data is: " << copy_data <<"\n";
    handle_data();
    // ++count;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_ros_handle");
  ros::NodeHandle n;

// %Tag(SUBSCRIBER)%
  Listener listener;
  ros::Subscriber mqtt_sub = n.subscribe("/robot_task", 1000, &Listener::callback, &listener);
  listener.mqtt_pub = n.advertise<std_msgs::String>("/server_task", 1000);
  ros::Rate loop_rate(10);
// %EndTag(SUBSCRIBER)%
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "After spin: \n";
  listener.handle_data();
  return 0;
}