#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
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
    while (ros::ok() && !ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("[navigation] Waiting for the move_base action server to come up");
    }
    ROS_INFO("[navigation] server is up");

    ros::NodeHandle nh;
    ros::Subscriber goalSub = nh.subscribe("navigation_ctrl", 1, goalCallBack);
    ros::Publisher pub = nh.advertise<wpr_msgs::instruction>("instruction", 1000);

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
            } else if (goalState == actionlib::SimpleClientGoalState::RECALLED
                || goalState == actionlib::SimpleClientGoalState::PREEMPTED) {
                ROS_INFO("[navigation] goal cancelled.");
                exp.type = exp.NAV_CANCELLED;
                exp.description = "cancelled";
                pub.publish(exp);
                state = S_IDLE;
            } else if (goalState == actionlib::SimpleClientGoalState::REJECTED) {
                ROS_INFO("[navigation] goal rejected.");
                exp.type = exp.NAV_REJECTED;
                exp.description = "rejected.";
                pub.publish(exp);
                state = S_IDLE;
            } else if (goalState == actionlib::SimpleClientGoalState::ABORTED) {
                exp.type = exp.NAV_UNREACHABLE;
                ROS_INFO("[navigation] goal unreachable.");
                exp.description = "aborted";
                pub.publish(exp);
                state = S_IDLE;
            } else {
                ROS_INFO("%s", goalState.toString().c_str());
            }
        }
        loop_rate.sleep();
    }
}