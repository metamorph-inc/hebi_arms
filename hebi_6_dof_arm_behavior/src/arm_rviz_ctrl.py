#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

import sys
import argparse
import math as m

import rospy
from geometry_msgs.msg import Pose

from hebi_generic_arm_controllers.arm_controller import ArmController
from hebi_generic_arm_controllers.rviz_interactive_markers import InteractiveMarkerManager

ABS_THRESHOLD = 0.001


def parse_args(args):
    parser = argparse.ArgumentParser()
    argparse.ArgumentParser()
    parser.add_argument('-hebi_group', type=str, required=True, help="hebi_group_name")
    parser.add_argument('-modules', type=str, nargs='+', required=True,
                        help="Module1Family/Module1Name Module2Family/Module2Name ... ModuleNFamily/ModuleNName")
    parser.add_argument('-base_link_name', type=str, required=True, help="")
    parser.add_argument('-end_link_name', type=str, required=True, help="")
    return parser.parse_args(args)


def abs_dist(pose1, pose2):
    """
    :type pose1: Pose or list
    :param pose1:

    :type pose2: Pose or list
    :param pose2:

    :return:
    :rtype: float
    """
    if type(pose1) is not type(pose2):
        raise ValueError("pose1 type {} does not match pose2 type {}".format(type(pose1), type(pose2)))

    sum_of_squares = 0.0
    cnt = 0

    if isinstance(pose1, Pose):
        cnt = 6
        sum_of_squares += (pose1.position.x-pose2.position.x)**2 \
                        + (pose1.position.y-pose2.position.y)**2 \
                        + (pose1.position.z-pose2.position.z)**2 \
                        + (pose1.orientation.w-pose2.orientation.w)**2 \
                        + (pose1.orientation.x-pose2.orientation.x)**2 \
                        + (pose1.orientation.y-pose2.orientation.y)**2 \
                        + (pose1.orientation.z-pose2.orientation.z)**2
    elif isinstance(pose1, list):
        if len(pose1) != len(pose2):
            raise ValueError("pose1 len {} does not match pose2 len {}".format(len(pose1), len(pose2)))
        cnt = len(pose1)
        for i,j in zip(pose1, pose2):
            sum_of_squares += (i-j)**2

    return m.sqrt(sum_of_squares)/cnt


if __name__ == '__main__':

    parser = parse_args(rospy.myargv(sys.argv[1:]))

    rospy.init_node(name="arm_rviz_ctrl", anonymous=True)
    rate = rospy.Rate(300)

    arm_con = ArmController(parser.hebi_group, parser.modules, parser.base_link_name, parser.end_link_name)
    int_marker_man = InteractiveMarkerManager()

    # Create marker for end-effector
    init_pose = arm_con.get_end_effector_pose()
    eff_int_marker_name = int_marker_man.add_marker(init_pose, frame=parser.base_link_name, description="end-effector")

    # Main loop
    last_pose = None
    while not rospy.is_shutdown():
        int_marker_pose = int_marker_man.get_marker_pose(eff_int_marker_name)
        position_only = int_marker_man.name_to_position_only_flag[eff_int_marker_name]

        _abs_dist = 0.0
        if last_pose is not None:
            _abs_dist = abs_dist(int_marker_pose, last_pose)

        if last_pose is None or _abs_dist > ABS_THRESHOLD:

            if position_only:
                jt_sln_found = arm_con.move_to_pose([int_marker_pose.position.x,
                                                     int_marker_pose.position.y,
                                                     int_marker_pose.position.z,
                                                     '','',''],
                                                    mode="command")
            else:
                #jt_sln_found = arm_con.move_to_pose(int_marker_pose, mode="trajectory", move_duration=2)
                jt_sln_found = arm_con.move_to_pose(int_marker_pose, mode="command")
            if jt_sln_found:
                int_marker_man.change_marker_color(eff_int_marker_name, "white", opacity=0.9)
            else:
                int_marker_man.change_marker_color(eff_int_marker_name, "red", opacity=0.9)

            last_pose = int_marker_pose

        rate.sleep()
