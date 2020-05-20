//
// Created by yspjack on 2020/5/20.
//

#ifndef SRC_ROBOT_SOUND_H
#define SRC_ROBOT_SOUND_H

#include <ros/ros.h>
#include "tts_engine.h"

class RobotSound : public TTSEngine {
private:
    ros::Publisher pub;
public:
    RobotSound(ros::NodeHandle &nh);

    void speak(const std::string &str) override;
};

#endif //SRC_ROBOT_SOUND_H
