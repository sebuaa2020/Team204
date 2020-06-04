#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <wpr_msgs/instruction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

#define S_IDLE 0
#define S_RUN 1
int state;

#define F_NOTSET 0
#define F_GOAL 1
#define F_CANCEL 2
int flag;

move_base_msgs::MoveBaseGoal goal;

void goalCallBack(const geometry_msgs::PoseStamped& msg)
{
    if (msg.pose.position.z != 0) {
        flag = F_CANCEL;
    } else {
        goal.target_pose.pose = msg.pose;
        goal.target_pose.header.frame_id = msg.header.frame_id;
        goal.target_pose.header.stamp = ros::Time::now();
        flag = F_GOAL;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navgationManager");
    MoveBaseClient ac("move_base", true);
    ros::NodeHandle nh;
    ros::Subscriber goalSub = nh.subscribe("navigation_ctrl", 1, goalCallBack);
    ros::Publisher pub = nh.advertise<wpr_msgs::instruction>("instruction", 1000);
    ros::Publisher initPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);

    while (ros::ok() && !ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("[navigation] Waiting for the move_base action server to come up");
    }
    ROS_INFO("[navigation] server is up");

    ros::Duration(1).sleep();
    
    tf::TransformListener listener;
    //1. 阻塞直到frame相通
    std::cout << "1. 阻塞直到frame相通" << std::endl;
    listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(4.0));

    ros::Rate rate(1);
    while (ros::ok()) {
        tf::StampedTransform transform;
        try {
            //2. 监听对应的tf,返回平移和旋转
            std::cout << "2. 监听对应的tf,返回平移和旋转" << std::endl;
            listener.lookupTransform("/odom", "/base_footprint",
                ros::Time(0), transform);
            //ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO("Translation: x=%f, y=%f, z=%f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f", transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
        geometry_msgs::PoseWithCovarianceStamped poseMsg;
        poseMsg.pose.pose.position.x = transform.getOrigin().x();
        poseMsg.pose.pose.position.y = transform.getOrigin().y();
        poseMsg.pose.pose.position.z = transform.getOrigin().z();

        poseMsg.pose.pose.orientation.x = transform.getRotation().getX();
        poseMsg.pose.pose.orientation.y = transform.getRotation().getY();
        poseMsg.pose.pose.orientation.z = transform.getRotation().getZ();
        poseMsg.pose.pose.orientation.w = transform.getRotation().getW();

        boost::array<double, 36ul> v{
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        };
        poseMsg.pose.covariance.swap(v);
        initPose.publish(poseMsg);
        break;
        rate.sleep();
    }
    ROS_INFO("set initial pose done");

    ros::Rate loop_rate(1.0);

    state = S_IDLE;
    wpr_msgs::instruction exp;
    while (ros::ok()) {
        flag = F_NOTSET;
        ros::spinOnce();
        if (flag == F_GOAL) {
            ROS_INFO("[navigation] new goal got. Send to server.");
            ac.sendGoal(goal);
            state = S_RUN;
        } else if (flag == F_CANCEL) {
            ac.cancelGoal();
            state = S_IDLE;
        } else if (state == S_RUN) {
            actionlib::SimpleClientGoalState goalState = ac.getState();
            if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("[navigation] goal arrived.");
                exp.type = exp.NAV_ARRIVED;
                exp.description = "succeeded";
                pub.publish(exp);
                state = S_IDLE;
            } else if (goalState == actionlib::SimpleClientGoalState::ABORTED) {
                exp.type = exp.NAV_UNREACHABLE;
                ROS_INFO("[navigation] goal unreachable.");
                exp.description = "aborted";
                pub.publish(exp);
                state = S_IDLE;
            } else {
                if (goalState != actionlib::SimpleClientGoalState::ACTIVE)
                    ROS_INFO("%s", goalState.toString().c_str());
            }
        }
        loop_rate.sleep();
    }
}