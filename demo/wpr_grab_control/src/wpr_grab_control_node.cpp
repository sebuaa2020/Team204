//
// Created by yspjack on 2020/5/25.
// Using code from tutorial
// Warning: NOT TESTED

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <memory>
#include "wpr_grab_control/wpr_grab_control_node.h"

using namespace wpr_grab_control_node;

static float grabHorizontalOffset;
static float grabLiftOffset;
static float grabForwardOffset;
static float grabGripperValue;

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


void VelCmd(float inVx, float inVy, float inTz) {

}


class GrabControl {
    enum State {
        STEP_WAIT,
        STEP_FIND_PLANE,
        STEP_PLANE_DIST,
        STEP_FIND_OBJ,
        STEP_OBJ_DIST,
        STEP_HAND_UP,
        STEP_FORWARD,
        STEP_GRAB,
        STEP_OBJ_UP,
        STEP_BACKWARD,
        STEP_DONE,
        STEP_EXCEPTION
    };
    State nStep;
    std::shared_ptr<JointControl> jointControl;

    ros::Publisher ctrl_pub;
    std_msgs::String ctrl_msg;

    ros::Subscriber pose_diff_sub;
    geometry_msgs::Pose2D pose_diff;

    BoxMarker boxLastObject;
    BoxMarker boxPlane;

    float fObjGrabX;
    float fObjGrabY;
    float fObjGrabZ;
    float fMoveTargetX;
    float fMoveTargetY;
    float fPlaneDist;

    float fTargetPlaneDist;
    float fTargetGrabX;
    float fTargetGrabY;

    float fPlaneHeight;
public:
    GrabControl(const std::shared_ptr<JointControl> &jointControl) : jointControl(jointControl) {
        fTargetPlaneDist = 0.6;
        fTargetGrabX = 0.8;
        fTargetGrabY = 0.0;
        nStep = STEP_WAIT;
    }

    //2、前后运动控制到平面的距离
    bool stepPlaneDist() {
//        float fMinDist = 100;
        fPlaneDist = boxPlane.xMin;
        ROS_WARN("[PLANE_DIST] dist= %.2f", fPlaneDist);
        float diff = fPlaneDist - fTargetPlaneDist;
        if (fabs(diff) < 0.02) {
            VelCmd(0, 0, 0);
            return true;
        } else {
            if (diff > 0) {
                //距离还太远，前进
                VelCmd(0.1, 0, 0);
            } else {
                //距离还太近，后退
                VelCmd(-0.1, 0, 0);
            }
        }
        return false;
    }

    //3、检测物品，挑选出准备抓取的目标物品
    bool stepFindObject() {
        VelCmd(0, 0, 0);
        ctrl_msg.data = "pose_diff reset";
        ctrl_pub.publish(ctrl_msg);
        fObjGrabX = boxLastObject.xMin;
        fObjGrabY = (boxLastObject.yMin + boxLastObject.yMax) / 2;
        fObjGrabZ = boxLastObject.zMax;
        ROS_WARN("[OBJ_TO_GRAB] x = %.2f y= %.2f ,z= %.2f", fObjGrabX, fObjGrabY, fObjGrabZ);
        // 判断一下物品和桌子边缘的距离，如果放得太靠里，放弃抓取以免机器人撞击桌子
        if (fabs(fObjGrabX - fTargetPlaneDist) > 0.4) {
            ROS_WARN("[OBJ_TO_GRAB] Object is hard to reach !!!");
            return false;
        } else {
            fMoveTargetX = 0.0f;
            fMoveTargetY = fObjGrabY - fTargetGrabY + grabHorizontalOffset;
            //ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
            return true;
        }
        return true;
    }

    //4、左右平移对准目标物品
    bool stepObjectDist() {
        float vx, vy;
        vx = (fMoveTargetX - pose_diff.x) / 2;
        vy = (fMoveTargetY - pose_diff.y) / 2;

        VelCmd(vx, vy, 0);

        if (fabs(vx) < 0.01 && fabs(vy) < 0.01) {
            VelCmd(0, 0, 0);
            ctrl_msg.data = "pose_diff reset";
            ctrl_pub.publish(ctrl_msg);
            return true;
        }
        return false;
    }

    //5、抬起手臂
    bool stepHandUp() {
        jointControl->lift(fPlaneHeight + grabLiftOffset);
        jointControl->gripper(0.16);
        ros::Duration(1.0).sleep();
        VelCmd(0, 0, 0);
        fMoveTargetX = fObjGrabX - 0.55 + grabForwardOffset;
        fMoveTargetY = 0;
        ROS_WARN("[STEP_FORWARD] x = %.2f y= %.2f ", fMoveTargetX, fMoveTargetY);
        return true;
    }

    //6、前进靠近物品
    bool stepForward() {
        float vx, vy;
        vx = (fMoveTargetX - pose_diff.x) / 2;
        vy = (fMoveTargetY - pose_diff.y) / 2;

        VelCmd(vx, vy, 0);

        if (fabs(vx) < 0.01 && fabs(vy) < 0.01) {
            VelCmd(0, 0, 0);
            ctrl_msg.data = "pose_diff reset";
            ctrl_pub.publish(ctrl_msg);
            return true;
        }
        return false;
    }

    //7、抓取物品
    bool stepGrab() {
        jointControl->gripper(grabGripperValue);
        VelCmd(0, 0, 0);
        ros::Duration(1.0).sleep();
        return true;
    }

    //8、拿起物品
    bool stepObjUp() {
        jointControl->lift(jointControl->getLift() + 0.03);
        VelCmd(0, 0, 0);
        ros::Duration(1.0).sleep();
        fMoveTargetX = -(fTargetGrabX - 0.4);
        fMoveTargetY = 0;
        return true;
    }

    //9、带着物品后退
    bool stepBackward() {
        float vx, vy;
        vx = (fMoveTargetX - pose_diff.x) / 2;
        vy = (fMoveTargetY - pose_diff.y) / 2;

        VelCmd(vx, vy, 0);

        //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

        if (fabs(vx) < 0.01 && fabs(vy) < 0.01) {
            VelCmd(0, 0, 0);
            ctrl_msg.data = "pose_diff reset";
            ctrl_pub.publish(ctrl_msg);
            return true;
        }
        return false;
    }

    void grab(const BoxMarker &boxPlane, const BoxMarker &boxLastObject) {
        this->boxPlane = boxPlane;
        this->boxLastObject = boxLastObject;
        while (nStep != STEP_DONE && nStep != STEP_EXCEPTION) {
            ROS_INFO("nStep: %d", nStep);
            switch (nStep) {
                case STEP_WAIT:
                    nStep = STEP_FIND_PLANE;
                    break;
                case STEP_FIND_PLANE:
                    nStep = STEP_PLANE_DIST;
                    break;
                case STEP_PLANE_DIST:
                    nStep = stepPlaneDist() ? STEP_FIND_OBJ : STEP_PLANE_DIST;
                    break;
                case STEP_FIND_OBJ:
                    nStep = stepFindObject() ? STEP_OBJ_DIST : STEP_EXCEPTION;
                    break;
                case STEP_OBJ_DIST:
                    nStep = stepObjectDist() ? STEP_HAND_UP : STEP_OBJ_DIST;
                    break;
                case STEP_HAND_UP:
                    nStep = STEP_FORWARD;
                    break;
                case STEP_FORWARD:
                    nStep = stepForward() ? STEP_GRAB : STEP_FORWARD;
                    break;
                case STEP_GRAB:
                    nStep = STEP_OBJ_UP;
                    break;
                case STEP_OBJ_UP:
                    nStep = STEP_BACKWARD;
                    break;
                case STEP_BACKWARD:
                    nStep = STEP_DONE;
                    break;
                case STEP_DONE:
                case STEP_EXCEPTION:
                    break;
            }
            ros::Duration(0.5).sleep();
        }
        nStep = STEP_WAIT;
    }

    void release() {
        // TODO
    }

    void init(ros::NodeHandle &nh) {
        ctrl_pub = nh.advertise<std_msgs::String>("/wpb_home/ctrl", 30);
        pose_diff_sub = nh.subscribe("/wpb_home/pose_diff", 1, &GrabControl::PoseDiffCallback,
                                     this);
    }

    void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
        ROS_INFO("x=%f y=%f theta=%f", msg->x, msg->y, msg->theta);
    }
};

int main(int argc, char **args) {
    ros::init(argc, args, "wpr_grab_control_node");
    ros::NodeHandle nh;
    nh.getParam("grab_y_offset", grabHorizontalOffset);
    nh.getParam("grab_lift_offset", grabLiftOffset);
    nh.getParam("grab_forward_offset", grabForwardOffset);
    nh.getParam("grab_gripper_value", grabGripperValue);
    auto jointControl = std::make_shared<WPRJointControl>();
    jointControl->init(nh);
    auto grabControl = std::make_shared<GrabControl>(jointControl);
    grabControl->init(nh);

    BoxMarker boxLastObject;
    BoxMarker boxPlane;

    grabControl->grab(boxPlane, boxLastObject);

    ros::spin();
    return 0;
}