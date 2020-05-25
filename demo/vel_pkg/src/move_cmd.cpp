#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define CMD_STOP        0
#define CMD_FORWARD     1
#define CMD_BACKWARD    2  
#define CMD_LEFT        3
#define CMD_RIGHT       4   

geometry_msgs::Twist Move(double x)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = x;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    return vel_cmd;
}

geometry_msgs::Twist Turn(double x)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = x;
    return vel_cmd;
}

geometry_msgs::Twist Stop()
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    return vel_cmd;
}

void move_action(const std_msgs::Int32::ConstPtr & msg)
{
    ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<std_msgs::String>("/cmd_vel", 10);

 geometry_msgs::Twist vel_cmd;

switch(msg->data)
{
    case CMD_STOP:
        vel_cmd = Stop();
        break;
    case CMD_FORWARD:
        vel_cmd = Move(0.5);
        break;
    case CMD_BACKWARD:
        vel_cmd = Move(-0.5);
        break;
    case CMD_LEFT:
        vel_cmd = Turn(0.15);
        break;
    case CMD_RIGHT:
        vel_cmd = Turn(-0.15);
        break;
    default:
        vel_cmd = Stop();
        break;
}
  vel_pub.publish(vel_cmd);
   ros::spinOnce();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");
  ros::NodeHandle n;
   ros::Subscriber sub_sr = n.subscribe("/move_cmd", 10, move_action);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

geometry_msgs::Twist vel_cmd;

ros::Rate r(10);
while(ros::ok()){
  ros::spinOnce();
  r.sleep();
}

return 0;
}