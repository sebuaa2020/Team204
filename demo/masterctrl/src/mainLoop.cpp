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
static ros::Publisher voicePuber;
static ros::Publisher grabCmdPuber;
static ros::Publisher feedbackPuber;

void feedback2user(std::string str)
{
    ROS_INFO("%s", str.c_str());
    std_msgs::String ret;
    ret.data = str;
    feedbackPuber.publish(ret);
}

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

void start_voice()
{
    std_msgs::Int32 msg;
    msg.data = 1;
    voicePuber.publish(msg);
}

void end_voice()
{
    std_msgs::Int32 msg;
    msg.data = 0;
    voicePuber.publish(msg);
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
    system("roslaunch vel_pkg ros_mapping.launch");
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
        feedback2user("manual control");
        break;
    case GRABING:
        feedback2user("during grabing can't control manually");
        break;
    case NAVIGATING:
        feedback2user("during navigating can't control manually");
        break;
    default:
        feedback2user("unrecognized state");
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
            feedback2user("start to build map");
            break;
        case MAPPING:
            feedback2user("buliding map, needn't open twice");
            break;
        case GRABING:
        case IDLE:
        case NAVIGATING:
            feedback2user("map loaded, please unload map and do again");
            break;
        default:
            feedback2user("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::MAPPING_END) {
        switch (state) {
        case MAPPING:
            end_mapping();
            stateChangeTo(NOT_LOAD_MAP);
            feedback2user("end building map");
            break;
        case NOT_LOAD_MAP:
        case IDLE:
        case GRABING:
        case NAVIGATING:
            feedback2user("not building map. needn't end it");
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
            feedback2user("map saved");
            break;
        case GRABING:
        case IDLE:
        case NAVIGATING:
        case NOT_LOAD_MAP:
            feedback2user("can't save map: not building map now");
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
            feedback2user("list all map");
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
            feedback2user("delete map");
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::LOAD_MAP) {
        switch (state) {
        case NOT_LOAD_MAP:
            map_load(msg.argStr);
            feedback2user("map loaded");
            break;
        case MAPPING:
            feedback2user("building map, please quit first and do again");
            break;
        case IDLE:
        case GRABING:
        case NAVIGATING:
            feedback2user("map loaded, needn't load");
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::UNLOAD_MAP) {
        switch (state) {
        case IDLE:
            map_unload();
            feedback2user("map unloaded");
            break;
        case NOT_LOAD_MAP:
            feedback2user("unload failed: not load map before.");
            break;
        case MAPPING:
            feedback2user("unload failed: during building map");
            break;
        case GRABING:
        case NAVIGATING:
            feedback2user("unload failed: map is using");
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
            feedback2user("navigation failed: no map is load");
            break;
        case GRABING:
            feedback2user("navigation failed: grab mission is taking control");
            break;
        case IDLE:
            navStart(msg);
            stateChangeTo(NAVIGATING);
            feedback2user("start navigation");
            break;
        case NAVIGATING:
            navStart(msg);
            stateChangeTo(NAVIGATING);
            feedback2user("start new navigation");
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
            feedback2user("no navigation mission is running");
            break;
        case NAVIGATING:
            navCancel();
            stateChangeTo(IDLE);
            feedback2user("navigation mission cancelled");
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
    feedback2user("grab mission can't be done: no robot arm");
    return;

    switch (state) {
    case NOT_LOAD_MAP:
    case MAPPING:
    case NAVIGATING:
        break;
    case IDLE: {
        std_msgs::String msgPub;
        if (msg.type == wpr_msgs::instruction::GRAB_START) {
            msgPub.data = "grab";
        } else if (msg.type == wpr_msgs::instruction::GRAB_STOP) {
            msgPub.data = "stop";
        } else if (msg.type == wpr_msgs::instruction::RELEASE_START) {
            msgPub.data = "release";
        }
        grabCmdPuber.publish(msgPub);
        stateChangeTo(GRABING);
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
            start_barrier();
            feedback2user("barrier detect enable");
            break;
        case NAVIGATING:
        case GRABING:
            feedback2user("barrier detect shouldn't enable during navigating and grabing");
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
            end_barrier();
            feedback2user("barrier detect disable");
            break;
        case NAVIGATING:
        case GRABING:
            feedback2user("barrier detect disable failed: shouldn't enable during navigating and grabing");
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else {
        ROS_INFO("unrecognized msg in executeMappingCtrl");
    }
}

void executeVoice(const wpr_msgs::instruction& msg)
{
    if (msg.type == wpr_msgs::instruction::VOICE_START) {
        switch (state) {
        case NOT_LOAD_MAP:
        case MAPPING:
        case IDLE:
            start_voice();
            feedback2user("voice listening");
            break;
        case NAVIGATING:
            feedback2user("during navigating doesn't support listening");
            break;
        case GRABING:
            feedback2user("during grabing doesn't support listening");
            break;
        default:
            ROS_INFO("unrecognized state");
            break;
        }
    } else if (msg.type == wpr_msgs::instruction::VOICE_END) {
        switch (state) {
        case MAPPING:
        case NOT_LOAD_MAP:
        case IDLE:
            end_voice();
            feedback2user("end voice listening");
            break;
        case NAVIGATING:
            feedback2user("during navigating doesn't support listening");
            break;
        case GRABING:
            feedback2user("during grabing doesn't support listening");
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
    feedback2user("can't delete map");
    ROS_INFO("Can't delete map");
}

void exceptionMapload(const wpr_msgs::instruction& msg)
{
    if (state != IDLE) {
        ROS_INFO("[exception] the exception shouldn't be published during this state");
    } else {
        if (msg.type == wpr_msgs::instruction::MAP_LOAD_ERROR) {
            feedback2user("can't load map");
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
            feedback2user("nvaigation unreachable");
        } else if (msg.type == wpr_msgs::instruction::NAV_ARRIVED) {
            feedback2user("navigation goal arrived");
        } else if (msg.type == wpr_msgs::instruction::NAV_STUCK) {
            feedback2user("fatal: robot map got stuck");
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
    case wpr_msgs::instruction::GRAB_START:
    case wpr_msgs::instruction::RELEASE_START:
    case wpr_msgs::instruction::GRAB_STOP:
        executeGrab(msg);
        break;
    case wpr_msgs::instruction::BARRIER_START:
    case wpr_msgs::instruction::BARRIER_END:
        executeBarrier(msg);
        break;
    case wpr_msgs::instruction::VOICE_START:
    case wpr_msgs::instruction::VOICE_END:
        executeVoice(msg);
        break;

    //exception
    case wpr_msgs::instruction::NAV_UNREACHABLE:
    case wpr_msgs::instruction::NAV_ARRIVED:
    case wpr_msgs::instruction::MAP_DEL_ERROR:
        exceptionMapdel(msg);
        break;
    case wpr_msgs::instruction::MAP_LOAD_ERROR:
        exceptionMapload(msg);
        break;
    case wpr_msgs::instruction::GRAB_FINISHED:
        stateChangeTo(IDLE);

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
    feedbackPuber = nh.advertise<std_msgs::String>("feedback2ui", 1000);

    // init every Publisher
    navPuber = nh.advertise<geometry_msgs::PoseStamped>("navigation_ctrl", 1000);
    movePuber = nh.advertise<std_msgs::Int32>("inter_move", 1000);
    mapPuber = nh.advertise<std_msgs::String>("map_manager", 1000);

    // ros::Publisher pub = nh.advertise<wpr_msgs::instruction>("instruction", 1000);
    // wpr_msgs::instruction msg;
    // msg.type = wpr_msgs::instruction::GRAB_FINISHED;
    // pub.publish(msg);

    barrierPuber = nh.advertise<std_msgs::Int32>("barrier_switch", 1000);
    voicePuber = nh.advertise<std_msgs::Int32>("voice_switch", 1000);

    grabCmdPuber = nh.advertise<std_msgs::String>("/do_grab", 1000);

    // init state
    state = NOT_LOAD_MAP;

    ROS_INFO("listening to /instruction for command");
    ros::spin();
    return 0;
}
