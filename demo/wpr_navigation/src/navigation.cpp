#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <wpr_msgs/exception.h>

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
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::NodeHandle nh;
    ros::Subscriber goalSub = nh.subscribe("navigation_ctrl", 1, goalCallBack);
    ros::Publisher pub = nh.advertise<wpr_msgs::exception>("exception", 1000);

    ros::Rate loop_rate(1.0);

    state = S_IDLE;
    wpr_msgs::exception exp;
    while (ros::ok()) {
        flag = F_NOTSET;
        ros::spinOnce();
        ROS_INFO("flag:%d,status:%d\n", flag, state);
        if (flag == F_GOAL) {
            ac.sendGoal(goal);
            state = S_RUN;
        } else if (flag == F_CANCEL) {
            ac.cancelGoal();
            state = S_IDLE;
        } else if (state == S_RUN) {
            actionlib::SimpleClientGoalState goalState = ac.getState();
            if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED) {
                exp.type = exp.NAV_ARRIVED;
                exp.description = "succeeded";
                pub.publish(exp);
                state = S_IDLE;
            } else if (goalState == actionlib::SimpleClientGoalState::RECALLED
                || goalState == actionlib::SimpleClientGoalState::PREEMPTED) {
                exp.type = exp.NAV_ARRIVED;
                exp.description = "cancelled";
                pub.publish(exp);
                state = S_IDLE;
            } else if (goalState == actionlib::SimpleClientGoalState::REJECTED) {
                exp.type = exp.NAV_UNREACHABLE;
                exp.description = "rejected";
                pub.publish(exp);
                state = S_IDLE;
            } else if (goalState == actionlib::SimpleClientGoalState::ABORTED) {
                exp.type = exp.NAV_UNREACHABLE;
                exp.description = "aborted";
                pub.publish(exp);
                state = S_IDLE;
            }
        }
        loop_rate.sleep();
    }
}