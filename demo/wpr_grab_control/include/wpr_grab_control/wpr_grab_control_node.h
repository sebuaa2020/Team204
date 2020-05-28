//
// Created by yspjack on 2020/5/25.
//

#ifndef SRC_WPR_GRAB_CONTROL_NODE_H
#define SRC_WPR_GRAB_CONTROL_NODE_H

namespace wpr_grab_control_node {
    /** \brief Bounding box
     */
    struct BoxMarker {
        float xMax;
        float xMin;
        float yMax;
        float yMin;
        float zMax;
        float zMin;
    };

    class JointControl {
    public:
        virtual void lift(double liftValue) = 0;

        virtual void gripper(double gripperValue) = 0;

        virtual double getLift() = 0;

        virtual double getGripper() = 0;
    };

    class GrabControl {
    public:
        enum State {
            STEP_WAIT,
            STEP_FIND_PLANE,
            STEP_PLANE_DIST,
            STEP_FIND_OBJ,
            //STEP_FIND_PLAC: 寻找、检查放置位置情况
            STEP_FIND_PLACE,
            STEP_OBJ_DIST,
            STEP_HAND_UP,
            STEP_FORWARD,
            STEP_GRAB,
            STEP_RELEASE,
            STEP_OBJ_UP,
            //STEP_OBJ_FREE: 此时机械臂已离开目标
            STEP_OBJ_FREE,
            STEP_BACKWARD,
            STEP_DONE,
            STEP_EXCEPTION
        };
        static float grabHorizontalOffset;
        static float grabLiftOffset;
        static float grabForwardOffset;
        static float grabGripperValue;

    private:
        State nStep;
        std::shared_ptr<JointControl> jointControl;

        ros::Publisher ctrl_pub;
        std_msgs::String ctrl_msg;

        ros::Publisher vel_pub;

        ros::Subscriber pose_diff_sub;
        geometry_msgs::Pose2D pose_diff;

        BoxMarker *boxLastObject;
        BoxMarker *boxPlane;

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
        GrabControl(const std::shared_ptr<JointControl> &jointControl);

        void VelCmd(float inVx, float inVy, float inTz);

        //2、前后运动控制到平面的距离
        bool stepPlaneDist();

        //3、检测物品，挑选出准备抓取的目标物品
        bool stepFindObject();

        //3.5、确定物品释放位置
        //当物品释放位置不合适or物品和桌子边缘的距离不合适时返回false, 进入异常状态
        bool stepFindPlace();

        //4、左右平移对准目标物品
        bool stepObjectDist();

        //5、抬起手臂
        bool stepHandUp();

        //6、前进靠近物品
        bool stepForward();

        //7、抓取物品
        bool stepGrab();

        //7.5、 松开机械臂
        bool stepRelease();

        //8、拿起物品
        bool stepObjUp();

        //8.5、放下物品，收回机械臂
        bool stepObjFree();

        //9、带着物品后退
        //后退
        bool stepBackward();

        void reset();

        State grab(BoxMarker *boxPlane, BoxMarker *boxLastObject);


        //此处 boxPlane为目标放置平面，boxLastObject为目标放置位置
        State release(BoxMarker *boxPlane, BoxMarker *boxLastObject);

        void init(ros::NodeHandle &nh);

        void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
    };
}

#endif //SRC_WPR_GRAB_CONTROL_NODE_H
