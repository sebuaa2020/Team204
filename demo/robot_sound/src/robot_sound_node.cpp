//
// Created by yspjack on 2020/5/20.
//

#include <ros/ros.h>
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "wpb_home_speak_node");
    ros::NodeHandle nh;
    ros::Rate r(1);
    RobotSoundNode node(nh, false);
    for (int i = 0; i < 10 && nh.ok(); i++) {
        node.speak("Hello " + std::to_string(i));
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}