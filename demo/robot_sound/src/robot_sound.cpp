//
// Created by yspjack on 2020/5/20.
//

#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
#include "tts_engine.h"
#include "robot_sound.h"

RobotSound::RobotSound(ros::NodeHandle &nh) {
    pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 10);
}

void RobotSound::speak(const std::string &str) {
    sound_play::SoundRequest soundRequest;
    soundRequest.sound = sound_play::SoundRequest::SAY;
    soundRequest.command = sound_play::SoundRequest::PLAY_ONCE;
    soundRequest.volume = 100;
    soundRequest.arg = str;
    pub.publish(soundRequest);
}
