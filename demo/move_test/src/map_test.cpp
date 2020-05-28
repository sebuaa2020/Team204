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
    ros::Publisher test_pub = nh.advertise<std_msgs::String>("/map_manager", 1000);
    ros::Duration(3.0).sleep();
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

    int i = 0;

    std_msgs::String msg1;
    ros::Duration(3.0).sleep();
    /*
    i++;
    printf("send %d\n", i);
    msg1.data = "list";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
*/
    i++;
    printf("send %d\n", i);
    msg1.data = "load map";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();

    i++;
    printf("send %d\n", i);
    msg1.data = "unload";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
/*
    i++;
    printf("send %d\n", i);
    msg1.data = "unload";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();

    i++;
    printf("send %d\n", i);
    msg1.data = "delete test";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();

    i++;
    printf("send %d\n", i);
    msg1.data = "delete test";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
    
    i++;
    printf("send %d\n", i);
    msg1.data = "load test";
    test_pub.publish(msg1);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
*/
return 0;
}
