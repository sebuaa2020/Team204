//
// Created by yspjack on 2020/5/28.
//

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <unistd.h>
#include "wpr_grab_control/wpr_grab_control_node.h"

using namespace wpr_grab_control_node;

std::string home_dir;

class TestJointControl : public JointControl {
private:
    sensor_msgs::JointState ctrl_msg;

public:

    TestJointControl() {
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
    }

    void gripper(double gripperValue) {
        ctrl_msg.position[1] = gripperValue;
    }
};

std::shared_ptr<TestJointControl> jointControl;
std::shared_ptr<GrabControl> grabControl;
float GrabControl::grabHorizontalOffset = 0;
float GrabControl::grabLiftOffset = 0;
float GrabControl::grabForwardOffset = 0;
float GrabControl::grabGripperValue = 0.035f;

TEST(TestSuite, testCase1) {

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    home_dir = std::string(getenv("HOME"));
    jointControl = std::make_shared<TestJointControl>();
    grabControl = std::make_shared<GrabControl>(jointControl);
    return RUN_ALL_TESTS();
}
