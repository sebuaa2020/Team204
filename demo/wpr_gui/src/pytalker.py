#!/usr/bin/env python
# coding=utf-8
import rospy
from wpr_msgs.msg import instruction

"""
talk to master control node
"""

pub = None


def init():
    global pub
    pub = rospy.Publisher('/instruction', instruction, queue_size=10)
    rospy.init_node('pytalker', anonymous=True)


def base(type_, description, loginfo=True, publish=True):
    msg = instruction()
    msg.type = type_
    msg.description = description
    msg.box_plane = [0.0]
    msg.box_obj = [0.0]
    if loginfo:
        rospy.loginfo('base msg description: %s, type is %d',
                      description, type_)
    if publish:
        pub.publish(msg)
    return msg


def move(type_, description):
    rospy.loginfo("send move instr: %s, type is %d", description, type_)
    base(type_, description, loginfo=False)


# 面向gui的接口

def go_forward():
    move(instruction.FORWARD, 'go_forward')


def go_backward():
    move(instruction.BACKWARD, 'go_backward')


def turn_left():
    move(instruction.TURNLEFT, 'turn left')


def turn_right():
    move(instruction.TURNRIGHT, 'turn right')


def linear_speedup():
    move(instruction.LINEAR_SPEEDUP, 'linear speedup')


def linear_speeddown():
    move(instruction.LINEAR_SPEEDDOWN, 'linear speeddown')


def angular_speedup():
    move(instruction.ANGULAR_SPEEDUP, 'angular speedup')


def angular_speeddown():
    move(instruction.ANGULAR_SPEEDDOWN, 'angular speeddown')


def move_stop():
    move(instruction.STOP, 'stop move')


def mapping_start():
    base(instruction.MAPPING_START, 'mapping start')


def mapping_end():
    base(instruction.MAPPING_END, 'mapping end')


def save_map(map_name):
    msg = base(instruction.SAVE_MAP, 'save map: ' + map_name, publish=False)
    msg.argStr = map_name
    pub.publish(msg)


def list_map():
    base(instruction.LIST_MAP, 'list map')


def delete_map(map_name):
    msg = base(instruction.DELETE_MAP,
               'delete map: ' + map_name, publish=False)
    msg.argStr = map_name
    pub.publish(msg)


def load_map(map_name):
    msg = base(instruction.LOAD_MAP, 'load map: ' + map_name, publish=False)
    msg.argStr = map_name
    pub.publish(msg)


def unload_map():
    base(instruction.UNLOAD_MAP, 'unload map')


def nav_start(x, y, theta, reference='map'):
    msg = base(instruction.NAV_START, 'navgation start',
               loginfo=False, publish=False)
    msg.goalPose.x = x
    msg.goalPose.y = y
    msg.goalPose.theta = theta
    if reference == 'map':
        msg.goalPose.reference = msg.goalPose.REF_MAP
    elif reference == 'robot':
        msg.goalPose.reference = msg.goalPose.REF_ROBOT
    else:
        rospy.loginfo('unknown reference. change to default reference: map')
        msg.goalPose.reference = msg.goalPose.REF_MAP

    rospy.loginfo('nav goto (%f,%f) facing %f, refer to %s',
                  x, y, theta, reference)

    pub.publish(msg)


def nav_cancel():
    base(instruction.NAV_CANCEL, 'nav cancel')


def grab_start():
    base(instruction.GRAB_START, 'grab start')


def release_start():
    base(instruction.RELEASE_START, 'release start')


def barrier_start():
    base(instruction.BARRIER_START, 'barrier detect enable')


def barrier_end():
    base(instruction.BARRIER_END, 'barrier detect disable')


if __name__ == '__main__':

    def main():
        nav_start(1.0, -2.0, 0.0)

    init()
    rate = rospy.Rate(1)
    x = 1.0
    y = 2.0
    state = 'working'
    while not rospy.is_shutdown():
        nav_start(x, 1.0, 2.0)
        rate.sleep()
        x += 1
        
