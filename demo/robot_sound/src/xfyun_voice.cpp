//
// Created by yspjack on 2020/5/20.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tts_engine.h"
#include "xfyun_voice.h"

XfyunVoice::XfyunVoice(ros::NodeHandle &nh) {
    pub = nh.advertise<std_msgs::String>("/xfyun/tts", 10);
}

void XfyunVoice::speak(const std::string &str) {
    std_msgs::String msg;
    msg.data = str;
    pub.publish(msg);
}