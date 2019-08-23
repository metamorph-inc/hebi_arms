#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: hebi_4dof_arm_position_ctrl.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 02/05/2018
# Edit Date: 02/05/2018
#
# Description:
# Class to control position of arm end effector
'''

import sys
import argparse

from common.ctrl_end_eff_pos_class import CtrlEndEffPos


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-hebi_group_name', type=str, required=True, help="")
    parser.add_argument('-hebi_base_family', type=str, required=True, help="")
    parser.add_argument('-hebi_base_name', type=str, required=True, help="")
    parser.add_argument('-hebi_shoulder_family', type=str, required=True, help="")
    parser.add_argument('-hebi_shoulder_name', type=str, required=True, help="")
    parser.add_argument('-hebi_elbow_family', type=str, required=True, help="")
    parser.add_argument('-hebi_elbow_name', type=str, required=True, help="")
    parser.add_argument('-hebi_wrist_family', type=str, required=True, help="")
    parser.add_argument('-hebi_wrist_name', type=str, required=True, help="")
    parser.add_argument('-from_master_topic', type=str, required=True, help="")
    parser.add_argument('-to_master_topic', type=str, required=True, help="")
    parser.add_argument('-base_link_name', type=str, required=True, help="")
    parser.add_argument('-end_link_name', type=str, required=True, help="")
    parser.add_argument('-ik_timeout', type=float, default=0.01, help="")
    parser.add_argument('-ik_epsilon', type=float, default=1e-4, help="")
    parser.add_argument('-ik_solve_type', type=str, default="Distance", help="")
    parser.add_argument('-ik_position_bounds', type=str, default="0.005,0.005,0.005", help="")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    ## Parse args
    parser = parse_args(sys.argv[1:])
    hebi_group_name = parser.hebi_group_name
    hebi_families = [parser.hebi_base_family,
                     parser.hebi_shoulder_family,
                     parser.hebi_elbow_family
                     parser.hebi_wrist_family]
    hebi_names = [parser.hebi_base_name,
                  parser.hebi_shoulder_name,
                  parser.hebi_elbow_name,
                  parser.hebi_wrist_name]
    from_master_topic = parser.from_master_topic
    to_master_topic = parser.to_master_topic
    base_link_name = parser.base_link_name
    end_link_name = parser.end_link_name
    ik_timeout = parser.ik_timeout
    ik_epsilon = parser.ik_epsilon
    ik_solve_type = parser.ik_solve_type
    ik_position_bounds = [float(val) for val in parser.ik_position_bounds[1:-1].split(",")]

    ctrl_end_eff_pos = CtrlEndEffPos(hebi_group_name, hebi_families, hebi_names,
                                     from_master_topic, to_master_topic,
                                     base_link_name, end_link_name,
                                     ik_timeout, ik_epsilon, ik_solve_type,
                                     ik_position_bounds)
    ctrl_end_eff_pos.start()


if __name__ == '__main__':
    main()
