//
// Created by yspjack on 2020/5/28.
//

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <memory>
#include "wpr_grab_control/wpr_grab_control_node.h"

using namespace wpr_grab_control_node;

class WPRJointControl : public JointControl {
private:
    ros::Publisher mani_ctrl_pub;
    sensor_msgs::JointState ctrl_msg;

public:
    void init(ros::NodeHandle &nh) {
        mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
        mani_ctrl_pub.publish(ctrl_msg);
    }

    WPRJointControl() {
        ctrl_msg.name.resize(2);
        ctrl_msg.position.resize(2);
        ctrl_msg.velocity.resize(2);
        ctrl_msg.name[0] = "lift";
        ctrl_msg.name[1] = "gripper";
        ctrl_msg.position[0] = 0;
        ctrl_msg.position[1] = 0.16;
        ctrl_msg.velocity[0] = 0.5;
        ctrl_msg.velocity[1] = 5;
    }

    double getLift() {
        return ctrl_msg.position[0];
    }

    double getGripper() {
        return ctrl_msg.position[1];
    }

    void lift(double liftValue) {
        ctrl_msg.position[0] = liftValue;
        mani_ctrl_pub.publish(ctrl_msg);
    }

    void gripper(double gripperValue) {
        ctrl_msg.position[1] = gripperValue;
        mani_ctrl_pub.publish(ctrl_msg);
    }
};

static BoxMarker boxLastObject;
static BoxMarker boxPlane;

bool unpackBoxMsg(const std_msgs::Float32MultiArray::ConstPtr &msg, BoxMarker &box, int idx) {
    if (msg->data.empty()) {
        return false;
    }
    if (msg->data.size() % 6 != 0) {
        return false;
    }
    if (idx < 0 || idx >= msg->data.size() / 6) {
        return false;
    }
    int i = idx * 6;
    box.xMin = msg->data[i + 0];
    box.xMax = msg->data[i + 1];
    box.yMin = msg->data[i + 2];
    box.yMax = msg->data[i + 3];
    box.zMin = msg->data[i + 4];
    box.zMax = msg->data[i + 5];
    return true;
}

bool foundPlane = false;

void boxLastObjectCB(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    if (!unpackBoxMsg(msg, boxLastObject, 0)) {
        ROS_INFO("[boxLastObjectCB] No object");
        return;
    }
    if (!foundPlane) {
        ROS_INFO("[boxLastObjectCB] No plane");
        return;
    }
//    ROS_INFO("[boxLastObjectCB] Get object");
}

void boxPlaneCB(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    foundPlane = false;
    if (!unpackBoxMsg(msg, boxPlane, 0)) {
        ROS_INFO("[boxPlaneCB] No plane");
        return;
    }
//    ROS_INFO("[boxPlaneCB] Get plane");
    foundPlane = true;
}

enum GrabCMD {
    GRABCMD_STOP,
    GRABCMD_GRAB,
    GRABCMD_RELEASE
};

GrabCMD curCmd;

void cmdCB(const std_msgs::String::ConstPtr &msg) {
    if (msg->data.find("grab") != std::string::npos) {
        curCmd = GRABCMD_GRAB;
    } else if (msg->data.find("release") != std::string::npos) {
        curCmd = GRABCMD_RELEASE;
    } else if (msg->data.find("stop") != std::string::npos) {
        curCmd = GRABCMD_STOP;
    } else {
        ROS_WARN("Unknown command: %s", msg->data.c_str());
    }
}

float GrabControl::grabHorizontalOffset;
float GrabControl::grabLiftOffset;
float GrabControl::grabForwardOffset;
float GrabControl::grabGripperValue;

int main(int argc, char **args) {
    ros::init(argc, args, "wpr_grab_control_node");
    ros::NodeHandle nh;
    ROS_INFO("wpr_grab_control_node start");
    nh.param("grab_y_offset", GrabControl::grabHorizontalOffset, 0.0f);
    nh.param("grab_lift_offset", GrabControl::grabLiftOffset, 0.0f);
    nh.param("grab_forward_offset", GrabControl::grabForwardOffset, 0.0f);
    nh.param("grab_gripper_value", GrabControl::grabGripperValue, 0.035f);
    ros::Subscriber plane_sub = nh.subscribe("/box_plane", 1, boxPlaneCB);
    ros::Subscriber objects_sub = nh.subscribe("/box_objects", 1, boxLastObjectCB);
    ros::Subscriber cmd_sub = nh.subscribe("/do_grab", 0, cmdCB);
    auto jointControl = std::make_shared<WPRJointControl>();
    jointControl->init(nh);
    auto grabControl = std::make_shared<GrabControl>(jointControl);
    grabControl->init(nh);
    grabControl->reset();
    curCmd = GRABCMD_STOP;

    while (nh.ok()) {
        if (curCmd == GRABCMD_GRAB) {
            if (foundPlane) {
                auto st = grabControl->grab(&boxPlane, &boxLastObject);
                if (st == GrabControl::STEP_DONE) {
                    ROS_WARN("STEP_DONE");
                    curCmd = GRABCMD_STOP;
                } else if (st == GrabControl::STEP_EXCEPTION) {
                    ROS_WARN("STEP_EXCEPTION");
                    curCmd = GRABCMD_STOP;
                } else {
                    ros::Duration(0.5).sleep();
                }
            }
        } else if (curCmd == GRABCMD_RELEASE) {

        } else {
            grabControl->reset();
            grabControl->VelCmd(0, 0, 0);
        }
        ros::spinOnce();
    }
    grabControl->VelCmd(0, 0, 0);
    return 0;
}
