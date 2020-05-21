#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nNum = scan->ranges.size();
    
    
    int nLeft = nNum/4;
    float leftDist = scan->ranges[nLeft];

    int nMid = nNum/2;
    float frontDist = scan->ranges[nMid];
    int nLeftMid = (nMid+nLeft)/2;
    float leftMidDist = scan->ranges[nLeftMid];
    
    int nRight = nLeft+nMid;
    float rightDist = scan->ranges[nRight];
    int nRightMid = (nMid+nRight)/2;
    float rightMidDist = scan->ranges[nRightMid];
    ROS_INFO("Point[%d] = %f,Point[%d] = %f,Point[%d] = %f, Point[%d] = %f,Point[%d] = %f", 
        nLeft, leftDist, nLeftMid, leftMidDist, nMid, frontDist, nRightMid, rightMidDist, nRight, rightDist); 

std_msgs::Float32MultiArray  vel_msg;
float x = 0;
float y = 0;
   // geometry_msgs::Twist vel_cmd;
    float dis = 1.5;
    if(frontDist > dis && leftDist > dis && rightDist > dis && leftMidDist > dis && rightMidDist > dis)
    {
        //vel_cmd.linear.x = 0.4;
        x = 0.4;
    }
    else
    {
        //vel_cmd.angular.z = 0.3;
        y = 0.3;
    }

    vel_msg.data.push_back(x);
    vel_msg.data.push_back(y);
    //vel_pub.publish(vel_cmd);
    vel_pub.publish(vel_msg);
     ros::spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"auto_move");
    
    ROS_INFO("auto_move_ start!");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
    vel_pub = nh.advertise<std_msgs::Float32MultiArray>("/move_vel", 10);
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

    ros::spin();
}
