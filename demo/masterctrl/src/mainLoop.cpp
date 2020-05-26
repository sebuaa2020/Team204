#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <wpr_msgs/instruction.h>
#include <wpr_msgs/navGoal.h>
#include <std_msgs/Int32.h>
#include<std_msgs/String.h>
#include <string>

using namespace std;

enum STATE {
    NOT_LOAD_MAP,
    MAPPING,
    IDLE,
    NAVIGATING,
    GRABING,
};

enum STATE state;
static ros::Publisher navPuber;
static ros::Publisher movePuber;
static ros::Publisher mapPuber;
static ros::Publisher barrierPuber;

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
            state = MAPPING;
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
            state = NOT_LOAD_MAP;
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
    }else {
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
     if (state != NAVIGATING) {
        ROS_INFO("[exception] the exception shouldn't be published during this state");
    } else {
        if (msg.type == wpr_msgs::instruction::NAV_UNREACHABLE) {
            /**/
        }
        state = IDLE;
    }
}

void exceptionNavigation(const wpr_msgs::instruction& msg)
{
    if (state != IDLE) {
        ROS_INFO("[exception] the exception shouldn't be published during this state");
    } else {
        if (msg.type == wpr_msgs::instruction::MAP_LOAD_ERROR) {
            /**/
        }
        state = NOT_LOAD_MAP;
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

int main()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("instruction", 1000, subCallback);

    // init every Publisher
    navPuber = nh.advertise<geometry_msgs::PoseStamped>("navigation_ctrl", 1000);
    movePuber = nh.advertise<std_msgs::Int32>("inter_move", 1000);
    mapPuber = nh.advertise<std_msgs::Int32>("map_manager", 1000);
    barrierPuber = nh.advertise<std_msgs::Int32>("barrier_switch", 1000);
    // init state
    state = NOT_LOAD_MAP;

    ros::spin();
    return 0;
}