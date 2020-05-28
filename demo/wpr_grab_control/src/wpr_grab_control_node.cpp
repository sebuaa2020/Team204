//
// Created by yspjack on 2020/5/25.
// Using code from tutorial
// Warning: NOT TESTED

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <memory>
#include "wpr_grab_control/wpr_grab_control_node.h"

using namespace wpr_grab_control_node;

GrabControl::GrabControl(const std::shared_ptr<JointControl> &jointControl) : jointControl(jointControl) {
    fTargetPlaneDist = 0.6;
    fTargetGrabX = 0.8;
    fTargetGrabY = 0.0;
    nStep = STEP_WAIT;
}

void GrabControl::VelCmd(float inVx, float inVy, float inTz) {
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

//2、前后运动控制到平面的距离
bool GrabControl::stepPlaneDist() {
//        float fMinDist = 100;
    fPlaneDist = boxPlane->xMin;
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
bool GrabControl::stepFindObject() {
    VelCmd(0, 0, 0);
    ctrl_msg.data = "pose_diff reset";
    ctrl_pub.publish(ctrl_msg);
    fObjGrabX = boxLastObject->xMin;
    fObjGrabY = (boxLastObject->yMin + boxLastObject->yMax) / 2;
    fObjGrabZ = boxLastObject->zMax;
    ROS_WARN("[OBJ_TO_GRAB] x = %.2f y= %.2f ,z= %.2f", fObjGrabX, fObjGrabY, fObjGrabZ);
    // 判断一下物品和桌子边缘的距离，如果放得太靠里，放弃抓取以免机器人撞击桌子
    if (fabs(fObjGrabX - fTargetPlaneDist) > 0.4) {
        ROS_WARN("[OBJ_TO_GRAB] Object is hard to reach !!!");
        return false;
    } else {
        fMoveTargetX = 0.0f;
        fMoveTargetY = fObjGrabY - fTargetGrabY + grabHorizontalOffset;
        ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f ", fMoveTargetX, fMoveTargetY);
        return true;
    }
    return true;
}

//3.5、确定物品释放位置
//当物品释放位置不合适or物品和桌子边缘的距离不合适时返回false, 进入异常状态
bool GrabControl::stepFindPlace() {
    //TODO
    return false;
}

//4、左右平移对准目标物品
//左右平移对准
bool GrabControl::stepObjectDist() {
#if defined(HAVE_POS_DIFF)
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
#else
    float diff = (boxLastObject->yMin + boxLastObject->yMax) / 2;
    if (fabs(diff) < 0.02) {
        VelCmd(0, 0, 0);
        return true;
    } else {
        if (diff > 0) {
            //距离还太远，前进
            VelCmd(0, 0.1, 0);
        } else {
            //距离还太近，后退
            VelCmd(0, -0.1, 0);
        }
        return false;
    }
    return false;
#endif
}

//5、抬起手臂
bool GrabControl::stepHandUp() {
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
bool GrabControl::stepForward() {
#if defined(HAVE_POS_DIFF)
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
#else
    float diff = (boxLastObject->xMin - 0.55 + grabForwardOffset) / 2;
    if (fabs(diff) < 0.02) {
        VelCmd(0, 0, 0);
        return true;
    } else {
        if (diff > 0) {
            VelCmd(0.1, 0, 0);
        } else {
            VelCmd(-0.1, 0, 0);
        }
        return false;
    }
    return false;
#endif
}

//7、抓取物品
bool GrabControl::stepGrab() {
    jointControl->gripper(grabGripperValue);
    VelCmd(0, 0, 0);
    ros::Duration(1.0).sleep();
    return true;
}

//7.5、 松开机械臂
bool GrabControl::stepRelease() {
    jointControl->gripper(grabGripperValue + 0.05);
    VelCmd(0, 0, 0);
    ros::Duration(1.0).sleep();
    return true;
}

//8、拿起物品
bool GrabControl::stepObjUp() {
    jointControl->lift(jointControl->getLift() + 0.03);
    VelCmd(0, 0, 0);
    ros::Duration(1.0).sleep();
    fMoveTargetX = -(fTargetGrabX - 0.4);
    fMoveTargetY = 0;
    return true;
}

//8.5、放下物品，收回机械臂
bool GrabControl::stepObjFree() {
    //TODO
    return true;
}

//9、带着物品后退
//后退
bool GrabControl::stepBackward() {
#if defined(HAVE_POS_DIFF)
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
#else
    // Not implemented
    return true;
#endif
}

void GrabControl::reset() {
    nStep = STEP_WAIT;
}

GrabControl::State GrabControl::grab(BoxMarker *boxPlane, BoxMarker *boxLastObject) {
    this->boxPlane = boxPlane;
    this->boxLastObject = boxLastObject;
    if (nStep != STEP_DONE && nStep != STEP_EXCEPTION) {
        ROS_INFO("[grab] nStep: %d", nStep);
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
                stepHandUp();
                nStep = STEP_FORWARD;
                break;
            case STEP_FORWARD:
                nStep = stepForward() ? STEP_GRAB : STEP_FORWARD;
                break;
            case STEP_GRAB:
                stepGrab();
                nStep = STEP_OBJ_UP;
                break;
            case STEP_OBJ_UP:
                stepObjUp();
                nStep = STEP_BACKWARD;
                break;
            case STEP_BACKWARD:
                stepBackward();
                nStep = STEP_DONE;
                break;
            case STEP_DONE:
            case STEP_EXCEPTION:
                VelCmd(0, 0, 0);
                break;
            default:
                ROS_WARN("[grab] Illeagal State");
                VelCmd(0, 0, 0);
                nStep = STEP_EXCEPTION;
                break;
        }
        ros::Duration(0.5).sleep();
    }
    return nStep;
}


//此处 boxPlane为目标放置平面，boxLastObject为目标放置位置
GrabControl::State GrabControl::release(BoxMarker *boxPlane, BoxMarker *boxLastObject) {
    this->boxPlane = boxPlane;
    this->boxLastObject = boxLastObject;
    if (nStep != STEP_DONE && nStep != STEP_EXCEPTION) {
        ROS_INFO("nStep: %d", nStep);
        switch (nStep) {
            case STEP_WAIT:
                nStep = STEP_FIND_PLANE;
                break;
            case STEP_FIND_PLANE:
                nStep = STEP_PLANE_DIST;
                break;
            case STEP_PLANE_DIST:
                nStep = stepPlaneDist() ? STEP_FIND_PLACE : STEP_PLANE_DIST;
                break;
            case STEP_FIND_PLACE:
                nStep = stepFindPlace() ? STEP_HAND_UP : STEP_EXCEPTION;
                break;
                //Release 应该先HAND_UP再对准
            case STEP_HAND_UP:
                //TODO
                stepHandUp();
                nStep = STEP_OBJ_DIST;
                break;
            case STEP_OBJ_DIST:
                nStep = stepObjectDist() ? STEP_FORWARD : STEP_OBJ_DIST;
                break;
            case STEP_FORWARD:
                nStep = stepForward() ? STEP_RELEASE : STEP_FORWARD;
                break;
            case STEP_RELEASE:
                //此处暂用释放动作
                stepRelease();
                nStep = STEP_OBJ_FREE;
                break;
            case STEP_OBJ_FREE:
                nStep = STEP_BACKWARD;
                break;
            case STEP_BACKWARD:
                stepBackward();
                nStep = STEP_DONE;
                break;
                // 以下几个状态不应在RELEASE()中出现，因此转移为EXCEPTION状态
            case STEP_FIND_OBJ:
            case STEP_GRAB:
            case STEP_OBJ_UP:
                ROS_WARN("[release] Illeagal State");
                VelCmd(0, 0, 0);
                nStep = STEP_EXCEPTION;
                break;
            case STEP_DONE:
                break;
            case STEP_EXCEPTION:
                //TODO?
                ROS_WARN("ERROR IN RELEASE");
                break;
        }
        ros::Duration(0.5).sleep();
    }
    return nStep;
}

void GrabControl::init(ros::NodeHandle &nh) {
    ctrl_pub = nh.advertise<std_msgs::String>("/wpb_home/ctrl", 30);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    pose_diff_sub = nh.subscribe("/wpb_home/pose_diff", 1, &GrabControl::PoseDiffCallback,
                                 this);
}

void GrabControl::PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
    ROS_INFO("x=%f y=%f theta=%f", msg->x, msg->y, msg->theta);
}
