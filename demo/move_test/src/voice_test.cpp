#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"voice_test");
    
    ROS_INFO("voice_ test_start!");

    ros::NodeHandle nh;
    ros::Publisher voice_pub = nh.advertise<std_msgs::Int32>("/voice_switch", 1000);
    ros::Publisher test_pub = nh.advertise<std_msgs::String>("/xfyun/iat", 1000);
    ros::Duration(3.0).sleep();
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

    printf("send %d\n", 1);
    std_msgs::Int32 msg;
    msg.data = 1;
    voice_pub.publish(msg);
    ros::spinOnce();

    std_msgs::String msg1;
    ros::Duration(3.0).sleep();
    printf("send %d\n", 0);
    msg1.data = "forward";
    test_pub.publish(msg1);
    ros::spinOnce();
return 0;
}