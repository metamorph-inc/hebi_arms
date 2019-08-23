#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# Name: smach_fsm_using_execute_joint_trajectory_states.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 02/12/2018
# Edit Date: 02/12/2018
#
# Description:
# SMACH FSM using ExecuteJointTrajectoryFromFile states and trajectory *.json files from trajectory_recording_tool
"""

import sys
import argparse
import rospy

from smach import StateMachine
import smach_ros
from trajectory_recording_tool.smach_state_classes import ExecuteJointTrajectoryFromFile, BreakOnMsg
from actionlib.simple_action_client import SimpleActionClient
from hebiros.srv import AddGroupFromNamesSrv, SetCommandLifetimeSrv
from hebiros.msg import TrajectoryAction


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
    parser.add_argument('-from_master_topic', type=str, required=True, help="")
    parser.add_argument('-to_master_topic', type=str, required=True, help="")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    ## Parse args
    parser = parse_args(sys.argv[1:])
    hebi_group_name = parser.hebi_group_name
    hebi_families = [parser.hebi_base_family,
                     parser.hebi_shoulder_family,
                     parser.hebi_elbow_family]
    hebi_names = [parser.hebi_base_name,
                  parser.hebi_shoulder_name,
                  parser.hebi_elbow_name]
    from_master_topic = parser.from_master_topic
    to_master_topic = parser.to_master_topic

    # ROS stuff
    rospy.init_node('smach_fsm_using_execute_joint_trajectory_state', anonymous=True)
    rate = rospy.Rate(200)

    # HEBI setup - NOTE: hebiros_node must be running
    rospy.loginfo("HEBI Group name: " + str(hebi_group_name))
    rospy.loginfo("HEBI families: " + str(hebi_families))
    rospy.loginfo("HEBI names: " + str(hebi_names))
    hebi_mapping = [family + '/' + name for family, name in zip(hebi_families, hebi_names)]

    # Create a service client to create a group
    set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
    # Create a service client to set the command lifetime
    set_command_lifetime = rospy.ServiceProxy("/hebiros/" + hebi_group_name + "/set_command_lifetime",
                                              SetCommandLifetimeSrv)
    # Topic to receive feedback from a group
    hebi_group_fbk_topic = "/hebiros/" + hebi_group_name + "/feedback/joint_state"
    rospy.loginfo("  hebi_group_feedback_topic: %s", "/hebiros/" + hebi_group_name + "/feedback/joint_state")
    # Topic to send commands to a group
    hebi_group_cmd_topic = "/hebiros/" + hebi_group_name + "/command/joint_state"
    rospy.loginfo("  hebi_group_command_topic: %s", "/hebiros/" + hebi_group_name + "/command/joint_state")
    # Call the /hebiros/add_group_from_names service to create a group
    rospy.loginfo("  Waiting for AddGroupFromNamesSrv at %s ...", '/hebiros/add_group_from_names')
    rospy.wait_for_service('/hebiros/add_group_from_names')
    rospy.loginfo("  AddGroupFromNamesSrv AVAILABLE.")
    set_hebi_group(hebi_group_name, hebi_names, hebi_families)

    # TrajectoryAction client
    trajectory_action_client = SimpleActionClient("/hebiros/" + hebi_group_name + "/trajectory", TrajectoryAction)
    rospy.loginfo("  Waiting for TrajectoryActionServer at %s ...", "/hebiros/" + hebi_group_name + "/trajectory")
    trajectory_action_client.wait_for_server()  # block until action server starts
    rospy.loginfo("  TrajectoryActionServer AVAILABLE.")

    ### CREATE ARM STATE INSTANCES ###
    # trace line
    trace_line = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "trace_line_0.json",
                                                hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                hebi_group_cmd_topic, setup_time=3.0)
    # stir pot
    stir_pot_setup = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stir_pot_0.json",
                                                    hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                    hebi_group_cmd_topic, setup_time=2.0)
    stir_pot_loop_1 = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stir_pot_1.json",
                                                     hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                     hebi_group_cmd_topic, setup_time=1.0)
    stir_pot_loop_n = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stir_pot_1.json",
                                                     hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                     hebi_group_cmd_topic, setup_time=0.0)
    break_stir_pot_loop = BreakOnMsg("break_topic_1")
    stir_pot_exit = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stir_pot_2.json",
                                                   hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                   hebi_group_cmd_topic, setup_time=1.0)
    # stab bag of veggies
    stab_bag_setup = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stab_bag_0.json",
                                                    hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                    hebi_group_cmd_topic, setup_time=3.0)
    stab_bag_loop_n = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stab_bag_1.json",
                                                     hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                     hebi_group_cmd_topic, setup_time=0.5)
    break_stab_bag_loop = BreakOnMsg("break_topic_2")
    stab_bag_exit = ExecuteJointTrajectoryFromFile("hebi_3dof_arm_description", "stab_bag_2.json",
                                                   hebi_mapping, trajectory_action_client, set_command_lifetime, hebi_group_fbk_topic,
                                                   hebi_group_cmd_topic, setup_time=1.5)

    ### CREATE TOP SM ###
    top = StateMachine(outcomes=['exit', 'success'])

    ### INITIALIZE USERDATA ###
    # Updated by ExecuteJointTrajectoryFromFile State instances
    top.userdata.final_joint_positions = [None, None, None]

    # Keeps things a tad neater
    remapping_dict = {'final_joint_positions':'final_joint_positions'}

    with top:
        ### ADD LEG STATES TO THE TOP SM ###
        StateMachine.add('TRACE_LINE', trace_line,
                         transitions={'exit':'exit','failure':'STIR_POT_SETUP','success':'STIR_POT_SETUP'},
                         remapping=remapping_dict)

        StateMachine.add('STIR_POT_SETUP', stir_pot_setup,
                         transitions={'exit':'exit','failure':'STIR_POT_SETUP','success':'STIR_POT_LOOP_1'},
                         remapping=remapping_dict)
        StateMachine.add('STIR_POT_LOOP_1', stir_pot_loop_1,
                         transitions={'exit':'exit','failure':'STIR_POT_LOOP_N','success':'STIR_POT_LOOP_N'},
                         remapping=remapping_dict)
        StateMachine.add('STIR_POT_LOOP_N', stir_pot_loop_n,
                         transitions={'exit':'exit','failure':'STIR_POT_LOOP_N','success':'BREAK_STIR_POT_LOOP'},
                         remapping=remapping_dict)
        StateMachine.add('BREAK_STIR_POT_LOOP', break_stir_pot_loop,
                         transitions={'exit':'exit','true':'STIR_POT_EXIT','false':'STIR_POT_LOOP_N'})
        StateMachine.add('STIR_POT_EXIT', stir_pot_exit,
                         transitions={'exit':'exit','failure':'exit','success':'STAB_BAG_SETUP'},
                         remapping=remapping_dict)

        StateMachine.add('STAB_BAG_SETUP', stab_bag_setup,
                         transitions={'exit':'exit','failure':'STAB_BAG_SETUP','success':'STAB_BAG_LOOP_N'},
                         remapping=remapping_dict)
        StateMachine.add('STAB_BAG_LOOP_N', stab_bag_loop_n,
                         transitions={'exit':'exit','failure':'STAB_BAG_LOOP_N','success':'BREAK_STAB_BAG_LOOP'},
                         remapping=remapping_dict)
        StateMachine.add('BREAK_STAB_BAG_LOOP', break_stab_bag_loop,
                         transitions={'exit':'exit','true':'STAB_BAG_EXIT','false':'STAB_BAG_LOOP_N'})
        StateMachine.add('STAB_BAG_EXIT', stir_pot_exit,
                         transitions={'exit':'exit','failure':'exit','success':'success'},
                         remapping=remapping_dict)

    sis = smach_ros.IntrospectionServer(str(rospy.get_name()), top, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    user_input = raw_input("Please press the 'Return/Enter' key to start executing \
        - pkg: smach_fsm_using_execute_joint_trajectory_state.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing "
          "- pkg: smach_fsm_using_execute_joint_trajectory_state | node: "+str(rospy.get_name()) + "\n")

    top.execute()

    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
