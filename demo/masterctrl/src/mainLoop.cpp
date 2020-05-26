#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <wpr_msgs/instruction.h>
#include <wpr_msgs/navGoal.h>

enum STATE {
    NOT_LOAD_MAP,
    MAPPING,
    IDLE,
    NAVIGATING,
    GRABING,
};

enum STATE state;
static ros::Publisher navPuber;

void navStart(const wpr_msgs::instruction& goalMsg)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = goalMsg.goalPose.x;
    msg.pose.position.y = goalMsg.goalPose.y;
    msg.pose.position.z = 0;
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(goalMsg.goalPose.theta);
    if (goalMsg.goalPose.reference == goalMsg.goalPose.REF_ROBOT)
        msg.header.frame_id = "base_footprint";
    else
        msg.header.frame_id = "map";
    navPuber.publish(msg);
}

void navCancel()
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.z = 1;
    navPuber.publish(msg);
}

void executeMoveCtrl(const wpr_msgs::instruction& msg)
{
}

void executeMappingCtrl(const wpr_msgs::instruction& msg)
{
}

void executeMapManage(const wpr_msgs::instruction& msg)
{
}

void executeNavigation(const wpr_msgs::instruction& msg)
{
    if (msg.type == wpr_msgs::instruction::NAV_START) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case GRABING:
            /**/
            break;
        case IDLE:
        case NAVIGATING:
            navStart(msg);
            state = IDLE;
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::NAV_CANCEL) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case IDLE:
        case GRABING:
            /**/
            break;
        case NAVIGATING:
            navCancel();
            state = IDLE;
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else {
        ROS_INFO("unrecognized msg in executeNavigation");
    }
}
void executeGrab(const wpr_msgs::instruction& msg)
{
}

void exceptionNavigation(const wpr_msgs::instruction& msg)
{
    if (state != NAVIGATING) {
        ROS_INFO("[exception] the exception shouldn't be published during this state");
    } else {
        if (msg.type == wpr_msgs::instruction::NAV_UNREACHABLE) {
            /**/
        }
        state = IDLE;
    }
}

void subCallback(const wpr_msgs::instruction& msg)
{
    switch (msg.type) {
    //instruction
    case wpr_msgs::instruction::FORWARD:
    case wpr_msgs::instruction::BACKWARD:
    case wpr_msgs::instruction::TURNLEFT:
    case wpr_msgs::instruction::TURNRIGHT:
    case wpr_msgs::instruction::LINEAR_SPEEDUP:
    case wpr_msgs::instruction::LINEAR_SPEEDDOWN:
    case wpr_msgs::instruction::ANGULAR_SPEEDUP:
    case wpr_msgs::instruction::ANGULAR_SPEEDDOWN:
        executeMoveCtrl(msg);
        break;
    case wpr_msgs::instruction::MAPPING_START:
    case wpr_msgs::instruction::MAPPING_END:
        executeMappingCtrl(msg);
        break;
    case wpr_msgs::instruction::LIST_MAP:
    case wpr_msgs::instruction::DELETE_MAP:
    case wpr_msgs::instruction::LOAD_MAP:
    case wpr_msgs::instruction::UNLOAD_MAP:
        executeMapManage(msg);
        break;
    case wpr_msgs::instruction::NAV_START:
    case wpr_msgs::instruction::NAV_CANCEL:
        executeNavigation(msg);
        break;
    case wpr_msgs::instruction::GRAB_START:
        executeGrab(msg);
        break;
    //exception
    case wpr_msgs::instruction::NAV_UNREACHABLE:
    case wpr_msgs::instruction::NAV_ARRIVED:
    case wpr_msgs::instruction::NAV_REJECTED:
        exceptionNavigation(msg);
        break;
    //unrecognized
    default:
        ROS_INFO("unrecognized instruction or exception");
        break;
    }
}

int main()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("instrction", 1000, subCallback);

    // init every Publisher
    navPuber = nh.advertise<geometry_msgs::PoseStamped>("navigation_ctrl", 1000);

    // init state
    state = NOT_LOAD_MAP;

    ros::spin();
    return 0;
}