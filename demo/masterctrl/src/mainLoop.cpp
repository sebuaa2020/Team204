#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <wpr_msgs/instruction.h>
#include <wpr_msgs/navGoal.h>

using namespace std;

enum STATE {
    NOT_LOAD_MAP,
    MAPPING,
    IDLE,
    NAVIGATING,
    GRABING,
};

std::string state_str(enum STATE sta)
{
    switch (sta) {
    case NOT_LOAD_MAP:
        return "NOT_LOAD_MAP";
        break;
    case MAPPING:
        return "MAPPING";
        break;
    case IDLE:
        return "IDLE";
        break;
    case NAVIGATING:
        return "NAVIGATING";
        break;
    case GRABING:
        return "GRABING";
        break;
    default:
        ROS_INFO("state_str() unknown state");
    }
}

enum STATE state;
static ros::Publisher navPuber;
static ros::Publisher movePuber;
static ros::Publisher mapPuber;
static ros::Publisher barrierPuber;
static ros::Subscriber cloudSub;
static ros::Publisher objDetectPuber;
static ros::Publisher grubBoxPlanePuber;
static ros::Publisher grubBoxObjPuber;

void stateChangeTo(enum STATE to)
{
    ROS_INFO("state from %s change to %s\n", state_str(state).c_str(), state_str(to).c_str());
    state = to;
}

void start_barrier()
{
    std_msgs::Int32 msg;
    msg.data = 1;
    barrierPuber.publish(msg);
}

void end_barrier()
{
    std_msgs::Int32 msg;
    msg.data = 0;
    barrierPuber.publish(msg);
}

void map_save(string name)
{
    std_msgs::String msg;

    string command = "save " + name;
    msg.data = command;
    mapPuber.publish(msg);
}

void map_list()
{
    std_msgs::String msg;

    msg.data = "list";
    mapPuber.publish(msg);
}

void map_del(string name)
{
    std_msgs::String msg;

    string command = "delete " + name;
    msg.data = command;
    mapPuber.publish(msg);
}

void map_load(string name)
{
    std_msgs::String msg;

    string command = "load " + name;
    msg.data = command;
    mapPuber.publish(msg);
}

void map_unload()
{
    std_msgs::String msg;

    msg.data = "unload";
    mapPuber.publish(msg);
}

void start_mapping()
{
    system("roslaunch vel_pkg ros_mapping");
}

void end_mapping()
{
    std_msgs::String msg;
    system("rosnode kill joint_state_publisher");
    system("rosnode kill hector_mapping");
    system("rosnode kill lidar_filter");
}

void pub_move(int command)
{
    std_msgs::Int32 msg;
    msg.data = command;
    movePuber.publish(msg);
}

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
    switch (state) {
    case NOT_LOAD_MAP:
    case MAPPING:
    case IDLE:
        pub_move(msg.type);
        break;
    case GRABING:
    case NAVIGATING:
        break;
    default:
        ROS_INFO("unrecognized state");
        break;
    }
}

void executeMappingCtrl(const wpr_msgs::instruction& msg)
{
    if (msg.type == wpr_msgs::instruction::MAPPING_START) {
        switch (state) {
        case NOT_LOAD_MAP:
            start_mapping();
            stateChangeTo(MAPPING);
            break;
        case MAPPING:
        case GRABING:
        case IDLE:
        case NAVIGATING:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::MAPPING_END) {
        switch (state) {
        case MAPPING:
            end_mapping();
            stateChangeTo(NOT_LOAD_MAP);
            break;
        case NOT_LOAD_MAP:
        case IDLE:
        case GRABING:
        case NAVIGATING:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else {
        ROS_INFO("unrecognized msg in executeMappingCtrl");
    }
}

void executeMapManage(const wpr_msgs::instruction& msg)
{
    if (msg.type == wpr_msgs::instruction::SAVE_MAP) {
        switch (state) {
        case MAPPING:
            map_save(msg.argStr);
            break;
        case GRABING:
        case IDLE:
        case NAVIGATING:
        case NOT_LOAD_MAP:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::LIST_MAP) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case GRABING:
        case IDLE:
        case NAVIGATING:
            map_list();
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::DELETE_MAP) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case GRABING:
        case IDLE:
        case NAVIGATING:
            map_del(msg.argStr);
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::LOAD_MAP) {
        switch (state) {
        case NOT_LOAD_MAP:
            map_load(msg.argStr);
            break;
        case MAPPING:
        case IDLE:
        case GRABING:
        case NAVIGATING:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::UNLOAD_MAP) {
        switch (state) {
        case IDLE:
            map_unload();
            break;
        case NOT_LOAD_MAP:
        case MAPPING:
        case GRABING:
        case NAVIGATING:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else {
        ROS_INFO("unrecognized msg in executeNavigation");
    }
}

void executeNavigation(const wpr_msgs::instruction& msg)
{
    if (msg.type == wpr_msgs::instruction::NAV_START) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case GRABING:
            /**/
            ROS_INFO("the state doesn't allow navigation");
            break;
        case IDLE:
        case NAVIGATING:
            navStart(msg);
            stateChangeTo(NAVIGATING);
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
            ROS_INFO("the state doesn't allow navigation");
            break;
        case NAVIGATING:
            navCancel();
            stateChangeTo(IDLE);
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else {
        ROS_INFO("unrecognized msg in executeNavigation");
    }
}

bool exeObjDetectOnce;
void cloudSubCallback(const sensor_msgs::PointCloud2& cloud)
{
    if (exeObjDetectOnce) {
        objDetectPuber.publish(cloud);
        exeObjDetectOnce = false;
    }
}

void executeObjDetect(const wpr_msgs::instruction& msg)
{
    switch (state) {
    case NOT_LOAD_MAP:
    case MAPPING:
    case IDLE:
        exeObjDetectOnce = true;
        break;
    case NAVIGATING:
    case GRABING:
        break;
    default:
        ROS_INFO("unrecognized state");
    }
}

void executeGrab(const wpr_msgs::instruction& msg)
{
    switch (state) {
    case NOT_LOAD_MAP:
    case MAPPING:
    case NAVIGATING:
        break;
    case IDLE: {
        std_msgs::Float32MultiArray box_plane_msg;
        std_msgs::Float32MultiArray box_obj_msg;
        box_plane_msg.data = msg.box_plane;
        box_obj_msg.data = msg.box_obj;
        grubBoxPlanePuber.publish(box_plane_msg);
        grubBoxObjPuber.publish(box_obj_msg);
    } break;
    case GRABING:
        break;
    default:
        ROS_INFO("unrecognized state");
    }
}

void executeBarrier(const wpr_msgs::instruction& msg)
{
    if (msg.type == wpr_msgs::instruction::BARRIER_START) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case IDLE:
        case NAVIGATING:
            start_barrier();
            break;
        case GRABING:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::BARRIER_END) {
        switch (state) {
        case MAPPING:
        case NOT_LOAD_MAP:
        case IDLE:
        case NAVIGATING:
            end_barrier();
            break;
        case GRABING:
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else {
        ROS_INFO("unrecognized msg in executeMappingCtrl");
    }
}

void exceptionMapdel(const wpr_msgs::instruction& msg)
{
    ROS_INFO("Can't delete map");
}

void exceptionMapload(const wpr_msgs::instruction& msg)
{
    if (state != IDLE) {
        ROS_INFO("[exception] the exception shouldn't be published during this state");
    } else {
        if (msg.type == wpr_msgs::instruction::MAP_LOAD_ERROR) {
            /**/
        }
        stateChangeTo(NOT_LOAD_MAP);
    }
}

void exceptionNavigation(const wpr_msgs::instruction& msg)
{
    if (state != NAVIGATING) {
        ROS_INFO("[exception] the exception shouldn't be published during this state");
    } else {
        if (msg.type == wpr_msgs::instruction::NAV_UNREACHABLE) {
            ROS_INFO("[exception] nvaigation unreachable");
        } else if (msg.type == wpr_msgs::instruction::NAV_ARRIVED) {
            ROS_INFO("navigation goal arrived");
        } else if (msg.type == wpr_msgs::instruction::NAV_STUCK) {
            ROS_INFO("robot maybe got stuck");
        }
        stateChangeTo(IDLE);
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
    case wpr_msgs::instruction::STOP:
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
    case wpr_msgs::instruction::OBJ_DETECT:
        executeObjDetect(msg);
        break;
    case wpr_msgs::instruction::GRAB_START:
        executeGrab(msg);
        break;
    case wpr_msgs::instruction::BARRIER_START:
    case wpr_msgs::instruction::BARRIER_END:
        executeBarrier(msg);
        break;
    //exception
    case wpr_msgs::instruction::NAV_UNREACHABLE:
    case wpr_msgs::instruction::NAV_ARRIVED:
    case wpr_msgs::instruction::NAV_REJECTED:
        exceptionNavigation(msg);
        break;
    case wpr_msgs::instruction::MAP_DEL_ERROR:
        exceptionMapdel(msg);
        break;
    case wpr_msgs::instruction::MAP_LOAD_ERROR:
        exceptionMapload(msg);
        break;
    //unrecognized
    default:
        ROS_INFO("unrecognized instruction or exception");
        break;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "wpr_master_ctrl");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("instruction", 1000, subCallback);

    // init every Publisher
    navPuber = nh.advertise<geometry_msgs::PoseStamped>("navigation_ctrl", 1000);
    movePuber = nh.advertise<std_msgs::Int32>("inter_move", 1000);
    mapPuber = nh.advertise<std_msgs::Int32>("map_manager", 1000);
    barrierPuber = nh.advertise<std_msgs::Int32>("barrier_switch", 1000);

    //TODO: rename the topic
    ros::Subscriber cloudSub = nh.subscribe("/kinect2/sd/points", 1, cloudSubCallback);
    objDetectPuber = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pub", 1000);
    grubBoxPlanePuber = nh.advertise<std_msgs::Float32MultiArray>("/box_plane", 1000);
    grubBoxObjPuber = nh.advertise<std_msgs::Float32MultiArray>("/box_objects", 1000);

    // init state
    state = IDLE;
    exeObjDetectOnce = false;

    ros::spin();
    return 0;
}