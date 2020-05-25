#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>


static ros::Publisher mani_pub;
static ros::Subscriber sub_sr; 

sensor_msgs::JointState changeManiState(double height, double hspeed, double clawdist, double cspeed) {
    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(2);
    ctrl_msg.position.resize(2);
    ctrl_msg.velocity.resize(2);
    ctrl_msg.name[0] = "lift";
    ctrl_msg.name[1] = "gripper";
    ctrl_msg.position[0] = height;
    ctrl_msg.position[1] = clawdist;
    ctrl_msg.velocity[0] = hspeed;
    ctrl_msg.velocity[1] = cspeed;
    return ctrl_msg;
}

void mani_action(const std_msgs::Float32MultiArray::ConstPtr & msg) {
    float height, hspeed, clawdist, cspeed;
    sensor_msgs::JointState mani_cmd;
    height = msg->data.at(0);
    clawdist = msg->data.at(2);
    hspeed = msg->data.at(1);
    cspeed = msg->data.at(3);
    mani_cmd = changeManiState(height, hspeed, clawdist, cspeed);
    mani_pub.publish(mani_cmd);
    ros::spinOnce();

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mani_ctrl_base");

    ros::NodeHandle n;
    mani_pub =  n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
    sub_sr = n.subscribe("/mani_ctrl_base", 10, mani_action);

    ros::Rate r(10);
    while(ros::ok()){
    ros::spinOnce();
    r.sleep();
    }
    return 0;
}