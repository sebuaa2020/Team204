//
// Created by yspjack on 2020/5/20.
//

#ifndef SRC_XFYUN_VOICE_H
#define SRC_XFYUN_VOICE_H

#include <ros/ros.h>
#include "tts_engine.h"

class XfyunVoice : public TTSEngine {
private:
    std::string apiKey;
    ros::Publisher pub;
public:
    XfyunVoice(ros::NodeHandle &nh);

    void speak(const std::string &str) override;
};

#endif //SRC_XFYUN_VOICE_H
