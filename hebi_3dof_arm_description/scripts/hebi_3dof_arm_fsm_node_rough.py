#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: hebi_3dof_arm_fsm_node_rough.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 01/26/2018
# Edit Date: 01/26/2018
#
# Description:
# Finite state machine controlling the position and movements of a
# 3-dof leg with a z-axis hip joint, a x-axis knee joint,
# and a x-axis ankle joint.
# Uses hebiros node to communicate with X-Series Actuators.
'''

import sys
import argparse
import math as m
import rospy
from std_msgs.msg import Float32, String
from smach import State, StateMachine
import smach_ros

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from hebiros.srv import AddGroupFromNamesSrv


class MoveToTargetPos(State):
    """SMACH state: Sends a joint state to the hip, knee, & ankle joint controller nodes

    Attributes:
        hip_target_pos              (list of float): hip joint target angles (rad)
        knee_target_pos             (list of float): knee joint target angles (rad)
        ankle_target_pos            (list of float): ankle joint target angles (rad)
        hebi_group_feedback_topic   (str):
        hebi_group_command_topic    (str):
        hebi_mapping                (list of str):
        pub_to_master_topic         (str): publish to master
        success_trigger_msg         (str): message published on pub_to_master_topic when 'success'
        ros_rate                    (float)
        instance_name               (str)
    """
    def __init__(self, hip_target_pos, knee_target_pos, ankle_target_pos,
                 hebi_group_feedback_topic, hebi_group_command_topic,
                 hebi_mapping,
                 pub_to_master_topic, success_trigger_msg,
                 ros_rate, instance_name):
        State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos'],
                             output_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos',
                                          'hip_target_pos', 'knee_target_pos', 'ankle_target_pos'])
        self.hip_target_pos = hip_target_pos
        self.knee_target_pos = knee_target_pos
        self.ankle_target_pos = ankle_target_pos

        # state machine stuff
        self.hip_current_pos = None  # These get initialized in execute
        self.knee_current_pos = None
        self.ankle_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.leg_sub = rospy.Subscriber(hebi_group_feedback_topic, JointState, self.leg_callback)
        self.leg_pub = rospy.Publisher(hebi_group_command_topic, JointState, queue_size=1)
        self.hebi_mapping = hebi_mapping
        self.master_pub = rospy.Publisher(pub_to_master_topic, String, queue_size=100)             # Publish status messages to the master topic
        self.success_trigger_msg = success_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name

    def execute(self, userdata):
        self.active_flag = True
        self.hip_current_pos = userdata.hip_current_pos  # values from previous state
        self.knee_current_pos = userdata.knee_current_pos
        self.ankle_current_pos = userdata.ankle_current_pos

        timeout_counter = rospy.Time.now().to_sec()
        timeout_time = 10.0  # If the state doesn't achieve the target position within
                             # this time, it will return a 'failure' outcome.
        settle_tolerance = 0.08  # < 5 degrees
        settle_time = 0.5

        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False
        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        ankle_settle_start = None
        ankle_settle_countdown = False
        ankle_done = False

        # TODO: Make this communication more robust to compensate for TCP best-effort
        while not rospy.is_shutdown():
            if ((rospy.Time.now().to_sec() - timeout_counter) > timeout_time):
                self.update_userdata(userdata)
                self.clean_up()
                return 'failure'
            else:
                # Check hip joint success condition
                if (abs(self.hip_current_pos - self.hip_target_pos) < settle_tolerance):
                    if not hip_settle_countdown:
                        hip_settle_countdown = True
                        hip_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - hip_settle_start) > settle_time):
                        hip_done = True
                # Check knee joint success condition
                if (abs(self.knee_current_pos - self.knee_target_pos) < settle_tolerance):
                    if not knee_settle_countdown:
                        knee_settle_countdown = True
                        knee_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - knee_settle_start) > settle_time):
                        knee_done = True
                # Check ankle joint success condition
                if (abs(self.ankle_current_pos - self.ankle_target_pos) < settle_tolerance):
                    if not ankle_settle_countdown:
                        ankle_settle_countdown = True
                        ankle_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - ankle_settle_start) > settle_time):
                        ankle_done = True
                if (hip_done and knee_done and ankle_done):
                    if self.success_trigger_msg is not None:
                        print("State: " + self.instance_name + ", Sent trigger: " + self.success_trigger_msg + " to master.")
                        self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                    self.clean_up()
                    self.update_userdata(userdata)
                    return 'success'  # You're done!
                else:
                    # TODO: Add a position/effort ramp/transition function
                    pub_msg = JointState()
                    pub_msg.header = Header()
                    pub_msg.header.stamp = rospy.Time.now()
                    pub_msg.name = self.hebi_mapping
                    pub_msg.position = [self.hip_target_pos, self.knee_target_pos, self.ankle_target_pos]
                    pub_msg.velocity = []
                    pub_msg.effort = []
                    self.leg_pub.publish(pub_msg)
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def leg_callback(self, msg):
        if self.active_flag:
            for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
                if name == self.hebi_mapping[0]:  # hip
                    self.hip_current_pos = pos
                elif name == self.hebi_mapping[1]:  # knee
                    self.knee_current_pos = pos
                elif name == self.hebi_mapping[2]:  # ankle
                    self.ankle_current_pos = pos
                else:
                    print("WARNING: leg_callback - unrecognized name!!!")

    def update_userdata(self, userdata):
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.ankle_current_pos = self.ankle_current_pos
        userdata.hip_target_pos = self.hip_target_pos
        userdata.knee_target_pos = self.knee_target_pos
        userdata.ankle_target_pos = self.ankle_target_pos

    def clean_up(self):
        self.active_flag = False


class MoveAlongTargetPath(State):
    """SMACH state: Sends a series of joint states to the hip, knee, & ankle joint controller nodes

    Attributes:
        max_rot_rate                (float): velocity ramp method parameter (rad/s)
        settle_time                 (float): min time to pause after target angle has been maintained (s)
        timeout_time                (float): time to achieve each set of target angles (s)
        best_effort                 (bool): if False, target angles must be achieved to return 'success'
        hip_target_pos_list         (list of float): hip joint target angles (rad)
        knee_target_pos_list        (list of float): knee joint target angles (rad)
        ankle_target_pos_list       (list of float): ankle joint target angles (rad)
        hebi_group_feedback_topic   (str):
        hebi_group_command_topic    (str):
        hebi_mapping                (list of str):
        pub_to_master_topic         (str): publish to master
        success_trigger_msg         (str): message published on pub_to_master_topic when 'success'
        ros_rate                    (float)
        instance_name               (str)
    """
    def __init__(self, max_rot_rate, settle_time, timeout_time, best_effort,
                 hip_target_pos_list, knee_target_pos_list, ankle_target_pos_list,
                 hebi_group_feedback_topic, hebi_group_command_topic,
                 hebi_mapping,
                 pub_to_master_topic, success_trigger_msg,
                 ros_rate, instance_name):
        State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos'],
                             output_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos',
                                          'hip_target_pos', 'knee_target_pos', 'ankle_target_pos'])
        self.max_rot_rate = max_rot_rate  # rad/s
        self.settle_time = settle_time    # s
        self.timeout_time = timeout_time  # s
        self.best_effort = best_effort
        self.hip_target_pos_list = hip_target_pos_list
        self.knee_target_pos_list = knee_target_pos_list
        self.ankle_target_pos_list = ankle_target_pos_list

        # state machine stuff
        self.hip_current_pos = None  # These get initialized in execute
        self.knee_current_pos = None
        self.ankle_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.leg_sub = rospy.Subscriber(hebi_group_feedback_topic, JointState, self.leg_callback)
        self.leg_pub = rospy.Publisher(hebi_group_command_topic, JointState, queue_size=1)
        self.hebi_mapping = hebi_mapping
        self.master_pub = rospy.Publisher(pub_to_master_topic, String, queue_size=100)      # Publish status messages to the master topic
        self.success_trigger_msg = success_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name

    def execute(self, userdata):
        self.active_flag = True
        self.hip_current_pos = userdata.hip_current_pos  # values from previous state
        self.knee_current_pos = userdata.knee_current_pos
        self.ankle_current_pos = userdata.ankle_current_pos

        # position ramp args
        t_prev = rospy.Time.now().to_sec()
        hip_pos_prev = self.hip_current_pos
        knee_pos_prev = self.knee_current_pos
        ankle_pos_prev = self.ankle_current_pos

        timeout_counter = rospy.Time.now().to_sec()
        settle_tolerance = 0.08  # < 5 degrees

        step = 0
        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False
        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        ankle_settle_start = None
        ankle_settle_countdown = False
        ankle_done = False
        # TODO: Make this communication more robust to compensate for TCP best-effort
        while not rospy.is_shutdown():
            hip_target = hip_pos_prev
            knee_target = knee_pos_prev
            ankle_target = ankle_pos_prev
            if self.hip_target_pos_list[step] is not None:
                hip_target = self.hip_target_pos_list[step]
            if self.knee_target_pos_list[step] is not None:
                knee_target = self.knee_target_pos_list[step]
            if self.ankle_target_pos_list[step] is not None:
                ankle_target = self.ankle_target_pos_list[step]
            if ((rospy.Time.now().to_sec() - timeout_counter) > self.timeout_time):
                if self.best_effort:
                    if self.success_trigger_msg is not None:
                        print("State: " + self.instance_name + ", Sent trigger: " + self.success_trigger_msg + " to master.")
                        self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                    self.update_userdata(userdata)
                    self.clean_up()
                    return 'success'
                else:
                    self.update_userdata(userdata)
                    self.clean_up()
                    return 'false'
            else:
                # Check hip joint success condition
                if (abs(self.hip_current_pos - hip_target) < settle_tolerance):
                    if not hip_settle_countdown:
                        hip_settle_countdown = True
                        hip_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - hip_settle_start) > self.settle_time):
                        hip_done = True
                # Check knee joint success condition
                if (abs(self.knee_current_pos - knee_target) < settle_tolerance):
                    if not knee_settle_countdown:
                        knee_settle_countdown = True
                        knee_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - knee_settle_start) > self.settle_time):
                        knee_done = True
                # Check ankle joint success condition
                if (abs(self.ankle_current_pos - ankle_target) < settle_tolerance):
                    if not ankle_settle_countdown:
                        ankle_settle_countdown = True
                        ankle_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - ankle_settle_start) > self.settle_time):
                        ankle_done = True
                if (hip_done and knee_done and ankle_done):
                    if (step+1 < len(self.hip_target_pos_list) or step+1 < len(self.knee_target_pos_list) or step+1 < len(self.ankle_target_pos_list)):
                        step = step + 1
                        hip_settle_start = None
                        hip_settle_countdown = False
                        hip_done = False
                        knee_settle_start = None
                        knee_settle_countdown = False
                        knee_done = False
                        ankle_settle_start = None
                        ankle_settle_countdown = False
                        ankle_done = False
                    else:
                        if self.success_trigger_msg is not None:
                            print("State: " + self.instance_name + ", Sent trigger: " + self.success_trigger_msg + " to master.")
                            self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                        self.clean_up()
                        self.update_userdata(userdata)
                        return 'success'  # You're done!
                else:
                    t_now = rospy.Time.now().to_sec()
                    hip_target_pos = self.ramped_rot(hip_pos_prev, hip_target, t_prev, t_now, self.max_rot_rate)
                    knee_target_pos = self.ramped_rot(knee_pos_prev, knee_target, t_prev, t_now, self.max_rot_rate)
                    ankle_target_pos = self.ramped_rot(ankle_pos_prev, ankle_target, t_prev, t_now, self.max_rot_rate)

                    pub_msg = JointState()
                    pub_msg.header = Header()
                    pub_msg.header.stamp = rospy.Time.now()
                    pub_msg.name = self.hebi_mapping
                    pub_msg.position = [hip_target_pos, knee_target_pos, ankle_target_pos]
                    pub_msg.velocity = []
                    pub_msg.effort = []
                    self.leg_pub.publish(pub_msg)

                    t_prev = t_now
                    hip_pos_prev = self.hip_current_pos
                    knee_pos_prev = self.knee_current_pos
                    ankle_pos_prev = self.ankle_current_pos
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def ramped_rot(self, pos_prev, pos_target, t_prev, t_now, ramp_rate):
        # compute maximum rotation step
        step = ramp_rate * (t_now - t_prev)
        sign = 1.0 if (pos_target > pos_prev) else -1.0
        error = m.fabs(pos_target - pos_prev)
        if error < step:  # if we can get to target position within timestep, then we're done
            return pos_target
        else:
            return pos_prev + sign*step  # take a step toward the target position

    def leg_callback(self, msg):
        if self.active_flag:
            for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
                if name == self.hebi_mapping[0]:  # hip
                    self.hip_current_pos = pos
                elif name == self.hebi_mapping[1]:  # knee
                    self.knee_current_pos = pos
                elif name == self.hebi_mapping[2]:  # ankle
                    self.ankle_current_pos = pos
                else:
                    print("WARNING: leg_callback - unrecognized name!!!")

    def update_userdata(self, userdata):
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.ankle_current_pos = self.ankle_current_pos
        if self.hip_target_pos_list[-1] is not None:
            userdata.hip_target_pos = self.hip_target_pos_list[-1]
        if self.knee_target_pos_list[-1] is not None:
            userdata.knee_target_pos = self.knee_target_pos_list[-1]
        if self.ankle_target_pos_list[-1] is not None:
            userdata.ankle_target_pos = self.ankle_target_pos_list[-1]

    def clean_up(self):
        self.active_flag = False


#TODO: This needs a velocity ramp function (maybe) like MoveAlongTargetPath class
class WaitForMasterCmd(State):
    """SMACH state: Waits for a trigger message from a master FSM

    Attributes:
        hebi_group_feedback_topic   (str):
        hebi_group_command_topic    (str):
        hebi_mapping                (list of str):
        sub_to_master_topic         (str): subscribe to master
        master_trigger_msgs         (list of str): recognized commands from master
        ros_rate                    (float)
        instance_name               (str)
    """
    def __init__(self, hebi_group_feedback_topic, hebi_group_command_topic,
                 hebi_mapping,
                 sub_to_master_topic,
                 master_trigger_msgs, ros_rate, instance_name):
        State.__init__(self, outcomes=master_trigger_msgs+['failure'],
                       input_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos',
                                   'hip_target_pos', 'knee_target_pos', 'ankle_target_pos'],
                       output_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos'])
        # state machine stuff
        self.hip_target_pos = None  # These get initialized in execute
        self.knee_target_pos = None
        self.ankle_target_pos = None
        self.hip_current_pos = None
        self.knee_current_pos = None
        self.ankle_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.leg_sub = rospy.Subscriber(hebi_group_feedback_topic, JointState, self.leg_callback)
        self.leg_pub = rospy.Publisher(hebi_group_command_topic, JointState, queue_size=1)
        self.hebi_mapping = hebi_mapping
        self.master_sub = rospy.Subscriber(sub_to_master_topic, String, self.master_callback)  # Publish status messages to the master topic
        self.master_trigger_msgs = master_trigger_msgs
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name
        self.master_trigger_msgs_set = frozenset(self.master_trigger_msgs)
        self.msg_from_master = None

    def execute(self, userdata):
        self.active_flag = True
        self.hip_current_pos = userdata.hip_current_pos  # values from previous state
        self.knee_current_pos = userdata.knee_current_pos
        self.ankle_current_pos = userdata.ankle_current_pos
        self.hip_target_pos = userdata.hip_target_pos
        self.knee_target_pos = userdata.knee_target_pos
        self.ankle_target_pos = userdata.ankle_target_pos

        # TODO: Make this communication more robust to compensate for TCP best-effort
        while not rospy.is_shutdown():
            if self.msg_from_master is not None:
                # process msg_from_master
                if self.msg_from_master in self.master_trigger_msgs_set:
                    trigger_msg = self.msg_from_master
                    self.update_userdata(userdata)
                    self.clean_up()
                    return trigger_msg
                else:
                    print("Unrecognized msg_from_master: " + self.msg_from_master)
                    self.msg_from_master = None
            else:
                # TODO: Add a position/effort ramp/transition function
                pub_msg = JointState()
                pub_msg.header = Header()
                pub_msg.header.stamp = rospy.Time.now()
                pub_msg.name = self.hebi_mapping
                pub_msg.position = [self.hip_target_pos, self.knee_target_pos, self.ankle_target_pos]
                pub_msg.velocity = []
                pub_msg.effort = []
                self.leg_pub.publish(pub_msg)

            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def leg_callback(self, msg):
        if self.active_flag:
            for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
                if name == self.hebi_mapping[0]:  # hip
                    self.hip_current_pos = pos
                elif name == self.hebi_mapping[1]:  # knee
                    self.knee_current_pos = pos
                elif name == self.hebi_mapping[2]:  # ankle
                    self.ankle_current_pos = pos
                else:
                    print("WARNING: leg_callback - unrecognized name!!!")

    def master_callback(self, msg):
        if self.active_flag:
            print("State: " + self.instance_name + ", Received trigger: " + msg.data + " from master")
            self.msg_from_master = msg.data

    def update_userdata(self, userdata):
        print("saving userdata: ", self.hip_current_pos, self.knee_current_pos, self.ankle_current_pos)
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.ankle_current_pos = self.ankle_current_pos

    def clean_up(self):
        self.active_flag = False
        self.msg_from_master = None


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, default=100, Description='rate at which ROS node publishes'")
    parser.add_argument('-hebi_group_name', type=str, required=True, help="hebi_group_name")
    parser.add_argument('-hip_family', type=str, required=True, help="hip_family")
    parser.add_argument('-hip_name', type=str, required=True, help="hip_knee")
    parser.add_argument('-knee_family', type=str, required=True, help="knee_family")
    parser.add_argument('-knee_name', type=str, required=True, help="knee_name")
    parser.add_argument('-ankle_family', type=str, required=True, help="ankle_family")
    parser.add_argument('-ankle_name', type=str, required=True, help="ankle_name")
    parser.add_argument('-from_master', type=str, required=True, help="master_angle_topic")
    parser.add_argument('-to_master', type=str, required=True, help="master_cmd_topic")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('three_dof_leg_fsm_node', anonymous=True)

    parser = parse_args(sys.argv[1:])
    hebi_group_name = parser.hebi_group_name
    hip_family = parser.hip_family
    hip_name = parser.hip_name
    knee_family = parser.knee_family
    knee_name = parser.knee_name
    ankle_family = parser.ankle_family
    ankle_name = parser.ankle_name
    master_sub_topic = parser.from_master
    master_pub_topic = parser.to_master
    ros_rate = parser.ros_rate

    # HEBI stuff - hebiros_node
    # Based on: https://github.com/HebiRobotics/HEBI-ROS/blob/master/hebiros_basic_examples/src/example_03_command_node.cpp
    hebi_group_name = hebi_group_name
    hebi_families = [hip_family,knee_family,ankle_family]
    hebi_module_names = [hip_name,knee_name,ankle_name]
    rospy.loginfo("HEBI Group name"+ str(hebi_group_name))
    rospy.loginfo("HEBI Family Names"+ str(hebi_families))
    rospy.loginfo("HEBI Module names"+ str(hebi_module_names))

    # [hip, knee, ankle]
    hebi_mapping = [hebi_families[0]+'/'+hebi_module_names[0],
                    hebi_families[1]+'/'+hebi_module_names[1],
                    hebi_families[2]+'/'+hebi_module_names[2]]

    # Create a client which uses the service to create a group
    set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
    # Topic to receive feedback from a group
    hebi_group_feedback_topic = "/hebiros/"+hebi_group_name+"/feedback/joint_state"
    # Topic to send commands to a group
    hebi_group_command_topic = "/hebiros/"+hebi_group_name+"/command/joint_state"

    # Construct a group using 3 known modules
    # Call the /hebiros/add_group_from_names service to create a group
    rospy.wait_for_service('/hebiros/add_group_from_names')
    set_hebi_group(hebi_group_name,hebi_module_names,hebi_families)

    # Create leg state instances
    wait_start = WaitForMasterCmd(hebi_group_feedback_topic, hebi_group_command_topic,
                                  hebi_mapping,
                                  master_sub_topic, ['stand_up'], ros_rate, 'wait_start')
    stand_up_path = MoveAlongTargetPath(6.23, 0.05, 10.0, True,
                                        [0.0]*11,
                                        [-1.2094, -1.2094, -0.934, -0.6655, -0.4221, -0.205, -0.0078, 0.1782, 0.362, 0.5558, 0.7854],
                                        [0, 2.4189, 2.3921, 2.3159, 2.1995, 2.0512, 1.8758, 1.6734, 1.439, 1.1579, 0.7854],
                                        hebi_group_feedback_topic, hebi_group_command_topic,
                                        hebi_mapping,
                                        master_pub_topic, 'done', ros_rate, 'stand_up_path')
    wait_orders = WaitForMasterCmd(hebi_group_feedback_topic, hebi_group_command_topic,
                                   hebi_mapping,
                                   master_sub_topic,
                                   ['lift', 'lift_high',
                                   'lift_north', 'plant_north',
                                   'plant_northwest', 'plant_northeast', 'plant_southwest', 'plant_southeast',
                                   'plant_north_plus_25', 'plant_north_minus_25',
                                   'push_north', 'push_south', 'push_east', 'push_west',
                                   'pull_east', 'pull_west',
                                   'push_east_from_push_south', 'push_west_from_push_south',
                                   'push_north_from_push_east', 'push_north_from_push_west',
                                   'push_west_from_north_plus_25', 'push_east_from_north_minus_25',
                                   'reset'],
                                    ros_rate, 'wait_orders')
    reset_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                     [0.0],
                                     [0.7854],
                                     [0.7854],
                                     hebi_group_feedback_topic, hebi_group_command_topic,
                                     hebi_mapping,
                                     master_pub_topic, 'done', ros_rate, 'reset_path')
    lift_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [None, None, None],
                                    [0.5558, 0.362, 0.1782],
                                    [1.1579, 1.439, 1.6734],
                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                    hebi_mapping,
                                    master_pub_topic, 'done', ros_rate, 'lift_path')
    lift_high_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                         [None, None, None, None],
                                         [0.5558, 0.362, 0.1782, -1.5],
                                         [1.1579, 1.439, 1.6734, 1.6734],
                                         hebi_group_feedback_topic, hebi_group_command_topic,
                                         hebi_mapping,
                                         master_pub_topic, 'done', ros_rate, 'lift_high_path')
    lift_north_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                          [None, None, None, 0.0],
                                          [0.5558, 0.362, 0.1782, 0.1782],
                                          [1.1579, 1.439, 1.6734, 1.6734],
                                          hebi_group_feedback_topic, hebi_group_command_topic,
                                          hebi_mapping,
                                          master_pub_topic, 'done', ros_rate, 'lift_north_path')
    plant_north_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                           [0.0]*3,
                                           [None, None, 1.0131],
                                           [None, 0.0194, 0.0194],
                                           hebi_group_feedback_topic, hebi_group_command_topic,
                                           hebi_mapping,
                                           master_pub_topic, 'done', ros_rate, 'plant_north_path')
    plant_north_plus_25_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                              [-0.4363, -0.4363, -0.4363],
                                              [None, None, 0.80595],
                                              [None, 0.0, 0.0],
                                              hebi_group_feedback_topic, hebi_group_command_topic,
                                              hebi_mapping,
                                              master_pub_topic, 'done', ros_rate, 'plant_north_plus_25_path')
    plant_north_minus_25_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                              [0.4363, 0.4363, 0.4363],
                                              [None, None, 0.80595],
                                              [None, 0.0, 0.0],
                                              hebi_group_feedback_topic, hebi_group_command_topic,
                                              hebi_mapping,
                                              master_pub_topic, 'done', ros_rate, 'plant_north_minus_25_path')
    plant_northwest_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [0.1529, 0.1529, 0.1529],
                                               [None, None, 1.0228],
                                               [None, 0.005, 0.005],
                                               hebi_group_feedback_topic, hebi_group_command_topic,
                                               hebi_mapping,
                                               master_pub_topic, 'done', ros_rate, 'plant_northwest_path')
    plant_northeast_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [-0.1529, -0.1529, -0.1529],
                                               [None, None, 1.0228],
                                               [None, 0.005, 0.005],
                                               hebi_group_feedback_topic, hebi_group_command_topic,
                                               hebi_mapping,
                                               master_pub_topic, 'done', ros_rate, 'plant_northeast_path')
    plant_southwest_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [0.1939, 0.1939, 0.1939],
                                               [None, None, 0.7862],
                                               [None, 0.7515, 0.7515],
                                               hebi_group_feedback_topic, hebi_group_command_topic,
                                               hebi_mapping,
                                               master_pub_topic, 'done', ros_rate, 'plant_southwest_path')
    plant_southeast_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [-0.1939, -0.1939, -0.1939],
                                               [None, None, 0.7862],
                                               [None, 0.7515, 0.7515],
                                               hebi_group_feedback_topic, hebi_group_command_topic,
                                               hebi_mapping,
                                               master_pub_topic, 'done', ros_rate, 'plant_southwest_path')
    push_north = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                     [0.0]*10,
                                     [0.7854, 0.7864, 0.7897, 0.7955, 0.8044, 0.8171, 0.8351, 0.8607, 0.9007, 1.0131],
                                     [0.7854, 0.7464, 0.7035, 0.6561, 0.6031, 0.5431, 0.4733, 0.3886, 0.2759, 0.0194],
                                     hebi_group_feedback_topic, hebi_group_command_topic,
                                     hebi_mapping,
                                     master_pub_topic, 'done', ros_rate, 'push_north')
    push_south = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                     [0.0]*10,
                                     [0.7854, 0.7863, 0.7891, 0.7934, 0.7993, 0.8065, 0.8151, 0.8249, 0.8359, 0.848],
                                     [0.7854, 0.8211, 0.8537, 0.8837, 0.9112, 0.9363, 0.9594, 0.9804, 0.9994, 1.0166],
                                     hebi_group_feedback_topic, hebi_group_command_topic,
                                     hebi_mapping,
                                     master_pub_topic, 'done', ros_rate, 'push_south')
    push_east = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [-0.0, -0.0218, -0.0436, -0.0654, -0.0871, -0.1087, -0.1302, -0.1516, -0.1728, -0.1939],
                                    [0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7855, 0.7855, 0.7857, 0.7859, 0.7862],
                                    [0.7854, 0.785, 0.7838, 0.7817, 0.7789, 0.7752, 0.7706, 0.7651, 0.7588, 0.7515],
                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                    hebi_mapping,
                                    master_pub_topic, 'done', ros_rate, 'push_east')
    push_west = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [0.0, 0.0218, 0.0436, 0.0654, 0.0871, 0.1087, 0.1302, 0.1516, 0.1728, 0.1939],
                                    [0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7855, 0.7855, 0.7857, 0.7859, 0.7862],
                                    [0.7854, 0.785, 0.7838, 0.7817, 0.7789, 0.7752, 0.7706, 0.7651, 0.7588, 0.7515],
                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                    hebi_mapping,
                                    master_pub_topic, 'done', ros_rate, 'push_west')
    pull_east = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [0.1939, 0.1728, 0.1516, 0.1302, 0.1087, 0.0871, 0.0654, 0.0436, 0.0218, 0.0],
                                    [0.7862, 0.7859, 0.7857, 0.7855, 0.7855, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854],
                                    [0.7515, 0.7588, 0.7651, 0.7706, 0.7752, 0.7789, 0.7817, 0.7838, 0.785, 0.7854],
                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                    hebi_mapping,
                                    master_pub_topic, 'done', ros_rate, 'pull_east')
    pull_west = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [-0.1939, -0.1728, -0.1516, -0.1302, -0.1087, -0.0871, -0.0654, -0.0436, -0.0218, -0.0],
                                    [0.7862, 0.7859, 0.7857, 0.7855, 0.7855, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854],
                                    [0.7515, 0.7588, 0.7651, 0.7706, 0.7752, 0.7789, 0.7817, 0.7838, 0.785, 0.7854],
                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                    hebi_mapping,
                                    master_pub_topic, 'done', ros_rate, 'pull_east')
    push_east_from_push_south = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [-0.0, -0.0271, -0.0543, -0.0813, -0.1082, -0.1349, -0.1615, -0.1878, -0.2139, -0.2397],
                                                    [0.848, 0.8478, 0.8473, 0.8465, 0.8453, 0.8438, 0.842, 0.8399, 0.8375, 0.8349],
                                                    [1.0166, 1.0164, 1.0157, 1.0146, 1.013, 1.011, 1.0085, 1.0055, 1.002, 0.9979],
                                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                                    hebi_mapping,
                                                    master_pub_topic, 'done', ros_rate, 'push_east_from_push_south')
    push_west_from_push_south = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [0.0, 0.0271, 0.0543, 0.0813, 0.1082, 0.1349, 0.1615, 0.1878, 0.2139, 0.2397],
                                                    [0.848, 0.8478, 0.8473, 0.8465, 0.8453, 0.8438, 0.842, 0.8399, 0.8375, 0.8349],
                                                    [1.0166, 1.0164, 1.0157, 1.0146, 1.013, 1.011, 1.0085, 1.0055, 1.002, 0.9979],
                                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                                    hebi_mapping,
                                                    master_pub_topic, 'done', ros_rate, 'push_west_from_push_south')
    push_north_from_push_west = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [0.1939, 0.1902, 0.1866, 0.1832, 0.1799, 0.1767, 0.1736, 0.1706, 0.1677, 0.1649],
                                                    [0.7862, 0.7887, 0.7933, 0.8002, 0.81, 0.8233, 0.8415, 0.8669, 0.906, 0.906],
                                                    [0.7515, 0.7136, 0.672, 0.6263, 0.5753, 0.5177, 0.4508, 0.37, 0.2624, 0.0084],
                                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                                    hebi_mapping,
                                                    master_pub_topic, 'done', ros_rate, 'push_north_from_push_west')
    push_north_from_push_east = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [-0.1939, -0.1902, -0.1866, -0.1832, -0.1799, -0.1767, -0.1736, -0.1706, -0.1677, -0.1649],
                                                    [0.7862, 0.7887, 0.7933, 0.8002, 0.81, 0.8233, 0.8415, 0.8669, 0.906, 0.906],
                                                    [0.7515, 0.7136, 0.672, 0.6263, 0.5753, 0.5177, 0.4508, 0.37, 0.2624, 0.0084],
                                                    hebi_group_feedback_topic, hebi_group_command_topic,
                                                    hebi_mapping,
                                                    master_pub_topic, 'done', ros_rate, 'push_north_from_push_east')
    push_east_from_north_minus_25_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                            [0.4358, 0.3474, 0.253, 0.154, 0.0517, -0.0517, -0.154, -0.253, -0.3474, -0.4358],
                                                            [0.8118, 0.7942, 0.7875, 0.7856, 0.7854, 0.7854, 0.7856, 0.7875, 0.7942, 0.8118],
                                                            [0.5665, 0.6652, 0.7292, 0.768, 0.7865, 0.7865, 0.768, 0.7292, 0.6652, 0.5665],
                                                            hebi_group_feedback_topic, hebi_group_command_topic,
                                                            hebi_mapping,
                                                            master_pub_topic, 'done', ros_rate, 'push_east_from_north_minus_25_path')

    push_west_from_north_plus_25_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                            [-0.4358, -0.3474, -0.253, -0.154, -0.0517, 0.0517, 0.154, 0.253, 0.3474, 0.4358],
                                                            [0.8118, 0.7942, 0.7875, 0.7856, 0.7854, 0.7854, 0.7856, 0.7875, 0.7942, 0.8118],
                                                            [0.5665, 0.6652, 0.7292, 0.768, 0.7865, 0.7865, 0.768, 0.7292, 0.6652, 0.5665],
                                                            hebi_group_feedback_topic, hebi_group_command_topic,
                                                            hebi_mapping,
                                                            master_pub_topic, 'done', ros_rate, 'push_west_from_north_plus_25_path')
    #TODO: Add the closed-form kinematics solution script as a function
    #TODO: Add more leg state instances here!!!

    three_dof_leg_fsm_node = StateMachine(outcomes=['success'])
    three_dof_leg_fsm_node.userdata.hip_current_pos = 0.0  # initial values}
    three_dof_leg_fsm_node.userdata.knee_current_pos = 0.0
    three_dof_leg_fsm_node.userdata.ankle_current_pos = 0.0
    three_dof_leg_fsm_node.userdata.hip_target_pos = 0.0
    three_dof_leg_fsm_node.userdata.knee_target_pos = 0.0
    three_dof_leg_fsm_node.userdata.ankle_target_pos = 0.0

    # Open the SMACH state machine
    with three_dof_leg_fsm_node:
        # Add states to the container
        StateMachine.add('WAIT_START', wait_start, transitions={'failure':'WAIT_START', 'stand_up':'STAND_UP_PATH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
                                                              'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        StateMachine.add('STAND_UP_PATH', stand_up_path, transitions={'success':'WAIT_ORDERS', 'failure':'STAND_UP_PATH'},
                                                         remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('WAIT_ORDERS', wait_orders, transitions={'failure':'WAIT_ORDERS', 'reset':'RESET_PATH',
                                                                  'lift':'LIFT_PATH', 'lift_high':'LIFT_HIGH_PATH',
                                                                  'lift_north':'LIFT_NORTH_PATH', 'plant_north':'PLANT_NORTH_PATH',
                                                                  'plant_north_plus_25':'PLANT_NORTH_PLUS_25_PATH', 'plant_north_minus_25':'PLANT_NORTH_MINUS_25_PATH',
                                                                  'plant_northwest':'PLANT_NORTHWEST_PATH', 'plant_northeast':'PLANT_NORTHEAST_PATH',
                                                                  'plant_southwest': 'PLANT_SOUTHWEST_PATH', 'plant_southeast':'PLANT_SOUTHEAST_PATH',
                                                                  'push_north':'PUSH_NORTH', 'push_south':'PUSH_SOUTH', 'push_east':'PUSH_EAST', 'push_west':'PUSH_WEST',
                                                                  'pull_east':'PULL_EAST', 'pull_west':'PULL_WEST',
                                                                  'push_east_from_push_south':'PUSH_EAST_FROM_PUSH_SOUTH', 'push_west_from_push_south':'PUSH_WEST_FROM_PUSH_SOUTH',
                                                                  'push_north_from_push_east':'PUSH_NORTH_FROM_PUSH_EAST', 'push_north_from_push_west':'PUSH_NORTH_FROM_PUSH_WEST',
                                                                  'push_east_from_north_minus_25':'PUSH_EAST_FROM_NORTH_MINUS_25', 'push_west_from_north_plus_25':'PUSH_WEST_FROM_NORTH_PLUS_25'},
                                                     remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
                                                                'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        StateMachine.add('RESET_PATH', reset_path, transitions={'success':'WAIT_ORDERS', 'failure':'RESET_PATH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('LIFT_PATH', lift_path, transitions={'success':'WAIT_ORDERS', 'failure':'LIFT_PATH'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('LIFT_HIGH_PATH', lift_high_path, transitions={'success':'WAIT_ORDERS', 'failure':'LIFT_HIGH_PATH'},
                                                           remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('LIFT_NORTH_PATH', lift_north_path, transitions={'success':'WAIT_ORDERS', 'failure':'LIFT_NORTH_PATH'},
                                                             remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTH_PATH', plant_northwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTH_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTH_PLUS_25_PATH', plant_north_plus_25_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTH_PLUS_25_PATH'},
                                                                               remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTH_MINUS_25_PATH', plant_north_minus_25_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTH_MINUS_25_PATH'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTHWEST_PATH', plant_northwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTHWEST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTHEAST_PATH', plant_northeast_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTHEAST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_SOUTHWEST_PATH', plant_southwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_SOUTHWEST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_SOUTHEAST_PATH', plant_southeast_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_SOUTHEAST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_NORTH', push_north, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_NORTH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_SOUTH', push_south, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_SOUTH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_EAST', push_east, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_EAST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_WEST', push_west, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_WEST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PULL_EAST', pull_east, transitions={'success':'WAIT_ORDERS', 'failure':'PULL_EAST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PULL_WEST', pull_west, transitions={'success':'WAIT_ORDERS', 'failure':'PULL_WEST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_EAST_FROM_PUSH_SOUTH', push_east_from_push_south, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_EAST_FROM_PUSH_SOUTH'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_WEST_FROM_PUSH_SOUTH', push_west_from_push_south, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_WEST_FROM_PUSH_SOUTH'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_NORTH_FROM_PUSH_WEST', push_north_from_push_west, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_NORTH_FROM_PUSH_WEST'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_NORTH_FROM_PUSH_EAST', push_north_from_push_east, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_NORTH_FROM_PUSH_EAST'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_EAST_FROM_NORTH_MINUS_25', push_east_from_north_minus_25_path, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_EAST_FROM_NORTH_MINUS_25'},
                                                                                              remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_WEST_FROM_NORTH_PLUS_25', push_west_from_north_plus_25_path, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_WEST_FROM_NORTH_PLUS_25'},
                                                                                            remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('three_dof_leg_v2_fsm_node' + str(rospy.get_name()), three_dof_leg_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    #user_input = raw_input("Please press the 'Return/Enter' key to start executing - pkg: three_dof_leg_v2_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: three_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    outcome = three_dof_leg_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))

if __name__ == '__main__':
    main()
