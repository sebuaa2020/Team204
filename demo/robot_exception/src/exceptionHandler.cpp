#include "ros_exception/exception.h"
#include "ros/ros.h"
#include <string>

void printExceptionInfo(const  ros_exception::exception::ConstPtr& msg) {
    ROS_INFO("Exception occured while in state %s， exception type=%s" , msg->state.c_str(), msg->type.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exceptionHandler");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("exceptions", 10, printExceptionInfo);
    ros::spin();
    return 0;
}