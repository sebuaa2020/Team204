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
}

#endif //SRC_WPR_GRAB_CONTROL_NODE_H
