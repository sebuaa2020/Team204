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

static BoxMarker boxLastObject;
static BoxMarker boxPlane;

bool unpackBoxMsg(const std::vector<float> &data, BoxMarker &box, int idx) {
    if (data.empty()) {
        return false;
    }
    if (data.size() % 6 != 0) {
        return false;
    }
    if (idx < 0 || idx >= data.size() / 6) {
        return false;
    }
    int i = idx * 6;
    box.xMin = data[i + 0];
    box.xMax = data[i + 1];
    box.yMin = data[i + 2];
    box.yMax = data[i + 3];
    box.zMin = data[i + 4];
    box.zMax = data[i + 5];
    return true;
}

static std::vector<float> PlaneData = {
        0.7830662727355957, 1.5963191986083984,
        -0.7709423303604126, 0.7335463762283325,
        0.9711573719978333, 1.0292726755142212
};
static std::vector<float> ObjectData = {
        1.107193946838379, 1.211625099182129,
        -0.413059800863266, -0.2507193684577942,
        1.0301960706710815, 1.3148088455200195,
        1.194014072418213, 1.2955882549285889,
        0.44832712411880493, 0.547940194606781,
        1.0312579870224, 1.0992393493652344
};

TEST(TestSuite, testInitial) {
    grabControl->reset();
    EXPECT_EQ(GrabControl::STEP_WAIT, grabControl->getState());
}

TEST(TestSuite, testUnpack) {
    static std::vector<float> data;
    static BoxMarker box;
    EXPECT_FALSE(unpackBoxMsg(data, box, 0));
    data.push_back(0);
    EXPECT_FALSE(unpackBoxMsg(data, box, 0));
    data.clear();
    for (int i = 0; i < 7; i++) {
        data.push_back(0);
    }
    EXPECT_FALSE(unpackBoxMsg(data, box, 0));
    data.clear();
    for (int i = 0; i < 6; i++) {
        data.push_back(i);
    }
    EXPECT_TRUE(unpackBoxMsg(data, box, 0));
    EXPECT_FALSE(unpackBoxMsg(data, box, 1));
}

TEST(TestSuite, testCase1) {
    EXPECT_TRUE(unpackBoxMsg(PlaneData, boxPlane, 0));
    EXPECT_TRUE(unpackBoxMsg(ObjectData, boxLastObject, 0));

    grabControl->reset();

    auto st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FIND_PLANE, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_PLANE_DIST, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_PLANE_DIST, st);

    // Adjust X position
    float diff = boxPlane.xMin - 0.6;
    boxPlane.xMin -= diff;
    boxPlane.xMax -= diff;
    boxLastObject.xMin -= diff;
    boxLastObject.xMax -= diff;

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FIND_OBJ, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_OBJ_DIST, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_OBJ_DIST, st);

    // Adjust Y position
    diff = (boxLastObject.yMin + boxLastObject.yMax) / 2;
    boxPlane.yMin -= diff;
    boxPlane.yMax -= diff;
    boxLastObject.yMin -= diff;
    boxLastObject.yMax -= diff;

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_HAND_UP, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FORWARD, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FORWARD, st);

    diff = (boxLastObject.xMin - 0.55);
    boxPlane.xMin -= diff;
    boxPlane.xMax -= diff;
    boxLastObject.xMin -= diff;
    boxLastObject.xMax -= diff;

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_GRAB, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_OBJ_UP, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_BACKWARD, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_DONE, st);
}

TEST(TestSuite, testCase2) {
    EXPECT_TRUE(unpackBoxMsg(PlaneData, boxPlane, 0));
    EXPECT_TRUE(unpackBoxMsg(ObjectData, boxLastObject, 0));

    // Adjust X position
    float diff = boxPlane.xMin - 0.6;
    boxPlane.xMin -= diff;
    boxPlane.xMax -= diff;
    boxLastObject.xMin -= diff;
    boxLastObject.xMax -= diff;

    // Adjust Y position
    diff = (boxLastObject.yMin + boxLastObject.yMax) / 2;
    boxPlane.yMin -= diff;
    boxPlane.yMax -= diff;
    boxLastObject.yMin -= diff;
    boxLastObject.yMax -= diff;

    grabControl->reset();

    auto st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FIND_PLANE, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_PLANE_DIST, st);

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FIND_OBJ, st);

    // Object unreachable
    boxLastObject.xMin += 10.0;
    boxLastObject.xMax += 10.0;

    st = grabControl->grab(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_EXCEPTION, st);
}

TEST(TestSuite, testCaseRelease1) {
    EXPECT_TRUE(unpackBoxMsg(PlaneData, boxPlane, 0));
    EXPECT_TRUE(unpackBoxMsg(ObjectData, boxLastObject, 0));

    grabControl->reset();

    auto st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FIND_PLANE, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_PLANE_DIST, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_PLANE_DIST, st);

    // Adjust X position
    float diff = boxPlane.xMin - 0.6;
    boxPlane.xMin -= diff;
    boxPlane.xMax -= diff;

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FIND_PLACE, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_HAND_UP, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_OBJ_DIST, st);

    // Adjust Y position
    diff = (boxPlane.yMin + boxPlane.yMax) / 2;
    boxPlane.yMin -= diff;
    boxPlane.yMax -= diff;

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_FORWARD, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_RELEASE, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_OBJ_FREE, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_BACKWARD, st);

    st = grabControl->release(&boxPlane, &boxLastObject);
    EXPECT_EQ(GrabControl::STEP_DONE, st);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_wpr_grab_control");
    ros::NodeHandle nh("~");
    home_dir = std::string(getenv("HOME"));
    jointControl = std::make_shared<TestJointControl>();
    grabControl = std::make_shared<GrabControl>(jointControl);
    grabControl->init(nh);
    return RUN_ALL_TESTS();
}
