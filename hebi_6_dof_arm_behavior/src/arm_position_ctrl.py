#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

import sys
import argparse

import rospy

from hebi_generic_arm_controllers.arm_controller import ArmController


def parse_args(args):
    parser = argparse.ArgumentParser()
    argparse.ArgumentParser()
    parser.add_argument('-hebi_group', type=str, required=True, help="hebi_group_name")
    parser.add_argument('-modules', type=str, nargs='+', required=True,
                        help="Module1Family/Module1Name Module2Family/Module2Name ... ModuleNFamily/ModuleNName")
    parser.add_argument('-base_link_name', type=str, required=True, help="")
    parser.add_argument('-end_link_name', type=str, required=True, help="")
    return parser.parse_args(args)


if __name__ == '__main__':

    parser = parse_args(rospy.myargv(sys.argv[1:]))

    rospy.init_node(name="arm_position_ctrl", anonymous=True)

    arm_con = ArmController(parser.hebi_group, parser.modules, parser.base_link_name, parser.end_link_name)

    # Main loop
    while not rospy.is_shutdown():

        # Get user input
        valid_input = False
        user_input = None
        while not rospy.is_shutdown() and not valid_input:
            print("\nPlease enter target end-effector pose:")
            print("x,y,z,R,P,Y")  # Tait-Bryan extrinsic xyz Euler angles
            user_input = raw_input()
            if len(user_input.split(",")) == 6:
                for i in range(6):
                    if user_input.split(",")[i] is not "":
                        try:
                            float(user_input.split(",")[i])
                        except ValueError:
                            break
                else:
                    valid_input = True

        arm_con.move_to_pose(user_input.split(","), mode="trajectory", move_duration=4)
