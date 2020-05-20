//
// Created by yspjack on 2020/5/20.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tts_engine.h"
#include "xfyun_voice.h"
#include "robot_sound.h"

class RobotSoundNode {
    bool online;
    TTSEngine *engine;

public:

    RobotSoundNode(ros::NodeHandle &nh, bool online = false) : online(online) {
        if (online) {
            engine = new XfyunVoice(nh);
        } else {
            engine = new RobotSound(nh);
        }
    }

    void speak(const std::string &str) {
        engine->speak(str);
    }
};

std::shared_ptr<RobotSoundNode> node;

void speakCB(const std_msgs::String::ConstPtr &msg) {
    node->speak(msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wpb_home_speak_node");
    ros::NodeHandle nh("~");
    bool online = false;
    nh.getParam("online", online);
    ROS_INFO_STREAM("using online engine: " << online);
    node = std::make_shared<RobotSoundNode>(RobotSoundNode(nh, online));
    ros::Subscriber sub = nh.subscribe("/robot_speak", 10, speakCB);
    ros::spin();
    return 0;
}
