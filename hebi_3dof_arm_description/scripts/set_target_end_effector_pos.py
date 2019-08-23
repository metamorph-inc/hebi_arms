#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: set_target_end_effector_pos.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 01/30/2018
# Edit Date: 01/30/2018
#
# Description:
# Publishes to to_arm topic a target end-effector position.
# Subscribes to from_arm topic for status.
'''

import sys
import argparse
import math as m

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose


# Global variables - quick hack
msg_from_arm = None


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-to_arm_topic', type=str, required=True, help="ROS Topic")
    parser.add_argument('-from_arm_topic', type=str, required=True, help="ROS Topic")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def arm_callback(msg):
    global msg_from_arm
    msg_from_arm = msg.data


def main():
    global msg_from_arm

    ## Parse args
    parser = parse_args(sys.argv[1:])
    to_arm_topic = parser.to_arm_topic
    from_arm_topic = parser.from_arm_topic

    ## ROS setup
    rospy.init_node('set_target_end_effector_pos')
    rate = rospy.Rate(200)
    arm_sub = rospy.Subscriber(from_arm_topic, String, arm_callback)
    arm_pub = rospy.Publisher(to_arm_topic, Pose, queue_size=1)
    # wait for child node to subscribe
    child_listening_to_arm_pub = False
    while not child_listening_to_arm_pub:
        num_connections = arm_pub.get_num_connections()
        if num_connections > 0:
            child_listening_to_arm_pub = True
        rate.sleep()

    while not rospy.is_shutdown():
        try:
            target_pos_str = raw_input("Enter the target end-effector position\n"
                + "  (xyz relative to base_link reference)\n"
                + "  e.g. 1.2 3 -0.1\n  ")
            target_pos_list = [float(x) for x in target_pos_str.split(' ')]
        except KeyboardInterrupt:
            sys.exit()

        pub_msg = Pose()
        pub_msg.position.x = target_pos_list[0]
        pub_msg.position.y = target_pos_list[1]
        pub_msg.position.z = target_pos_list[2]
        # not using orientation data
        arm_pub.publish(pub_msg)  # send target end effector position to leg node
        while not rospy.is_shutdown() and msg_from_arm is None:
            rate.sleep()
        if msg_from_arm == "success":
            rospy.loginfo("  IK SUCCESS! Joint solution found.\n")
        elif msg_from_arm == "failure":
            rospy.loginfo("  IK FAILED! Joint solution not found.\n"
                + "  Please try another pt...")
        msg_from_arm = None

if __name__ == '__main__':
    main()
