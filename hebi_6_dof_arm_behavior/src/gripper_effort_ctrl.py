#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

import sys
import argparse

import rospy
from sensor_msgs.msg import JointState

from hebiros.msg import CommandMsg
from hebiros_utils.hebiros_wrapper import HebirosWrapper


def parse_args(args):
    parser = argparse.ArgumentParser()
    argparse.ArgumentParser()
    parser.add_argument('-hebi_group', type=str, required=True, help="hebi_group_name")
    parser.add_argument('-module', type=str, required=True, help="ModuleFamily/ModuleName")
    return parser.parse_args(args)

if __name__ == '__main__':

    parser = parse_args(rospy.myargv(sys.argv[1:]))

    rospy.init_node(name="gripper_effort_ctrl", anonymous=True)

    # Set up HEBI ROS interface
    hebi_families = parser.module.split('/')[0]
    hebi_names = parser.module.split('/')[1]
    hebi_wrap = HebirosWrapper(parser.hebi_group, [hebi_families], [hebi_names])
    hebi_mapping = [parser.module]

    cmd_msg = CommandMsg()
    cmd_msg.name = hebi_mapping
    cmd_msg.settings.name = hebi_mapping
    cmd_msg.settings.position_gains.name = hebi_mapping
    cmd_msg.settings.effort_gains.kp = [1]
    cmd_msg.settings.effort_gains.ki = [0]
    cmd_msg.settings.effort_gains.kd = [0]
    cmd_msg.settings.effort_gains.i_clamp = [0]
    hebi_wrap.send_command_with_acknowledgement(cmd_msg)

    _hold_position = False
    _hold_joint_effort = None

    def _hold_position_cb(msg):
        if not rospy.is_shutdown() and _hold_position:
            jointstate = JointState()
            jointstate.name = hebi_wrap.hebi_mapping
            jointstate.position = []
            jointstate.velocity = []
            jointstate.effort = _hold_joint_effort
            hebi_wrap.joint_state_publisher.publish(jointstate)

    hebi_wrap.add_feedback_callback(_hold_position_cb)

    # Main loop
    while not rospy.is_shutdown():

        # Get user input
        valid_input = False
        user_input = None
        while not rospy.is_shutdown() and not valid_input:
            print("\nPlease enter target effort:")
            user_input = float(raw_input())
            valid_input = True

        jointstate = JointState()
        jointstate.name = hebi_wrap.hebi_mapping
        jointstate.position = []
        jointstate.velocity = []
        jointstate.effort = [user_input]
        _hold_position = False
        hebi_wrap.joint_state_publisher.publish(jointstate)
        _hold_joint_effort = jointstate.effort
        _hold_position = True
