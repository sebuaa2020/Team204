#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher main_pub;
bool swit;

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

std_msgs::Int32  bar_msg;
   // geometry_msgs::Twist vel_cmd;
    float dis = 1.5;
    bool barrier = false;
    if(frontDist > dis && leftDist > dis && rightDist > dis && leftMidDist > dis && rightMidDist > dis)
    {
        //vel_cmd.linear.x = 0.4;
        barrier = false;
    }
    else
    {
        //vel_cmd.angular.z = 0.3;
        barrier = true;
    }

    bar_msg.data = barrier?1:0;
    //vel_pub.publish(vel_cmd);
    if (swit) main_pub.publish(bar_msg);
}

void barrier_ctrl(const std_msgs::Int32::ConstPtr & msg)
{
   if (msg->data == 0)
   {
      swit = false;
   }
   else
   {
     swit = true;
   }
}

int main(int argc, char** argv)
{
    swit = false;
    ros::init(argc,argv,"auto_move");
    
    ROS_INFO("auto_move_ start!");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
     ros::Subscriber barrier_switch = nh.subscribe("/barrier_ctrl", 10, barrier_ctrl);
    main_pub = nh.advertise<std_msgs::Int32>("/main_msgl", 10);
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

    ros::spin();
}
