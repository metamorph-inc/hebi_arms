#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: end_effector_position_control.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 01/29/2018
# Edit Date: 02/05/2018
#
# Description:
# Uses trac-ik-python to calculate inverse kinematics.
# Uses hebiros node to communicate with X-Series Actuators.
'''

import sys
import argparse
import math as m

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
#from smach import State, StateMachine
#import smach_ros
from hebiros.srv import AddGroupFromNamesSrv
#from kdl_parser_py import urdf as kdl_urdf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from trac_ik_python.trac_ik import IK


# Global variables - quick hack
hebi_mapping = []
arm_sub = None
arm_pub = None
current_jt_pos = {}
current_jt_vel = {}
current_jt_eff = {}
last_valid_jt_ang = []
msg_from_master = None


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-base_link', type=str, required=True, help="base_link")
    parser.add_argument('-end_link', type=str, required=True, help="end_link")
    parser.add_argument('-hebi_group_name', type=str, required=True, help="hebi_group_name")
    parser.add_argument('-base_family', type=str, required=True, help="base_family")
    parser.add_argument('-base_name', type=str, required=True, help="base_name")
    parser.add_argument('-shoulder_family', type=str, required=True, help="shoulder_family")
    parser.add_argument('-shoulder_name', type=str, required=True, help="shoulder_name")
    parser.add_argument('-elbow_family', type=str, required=True, help="elbow_family")
    parser.add_argument('-elbow_name', type=str, required=True, help="elbow_name")
    parser.add_argument('-from_master_topic', type=str, required=True, help="ROS Topic")
    parser.add_argument('-to_master_topic', type=str, required=True, help="ROS Topic")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def arm_callback(msg):
    global current_jt_pos, current_jt_vel, current_jt_eff

    for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
        if name not in hebi_mapping:
            print("WARNING: arm_callback - unrecognized name!!!")
        else:
            current_jt_pos[name] = pos
            current_jt_vel[name] = vel
            current_jt_eff[name] = eff


def generate_joint_trajectory(xyz_pts, wxyz_pts,
                              ik_solver, pos_bounds, orient_bounds):

    seed_state = [current_jt_pos[motor] for motor in hebi_mapping]
    joint_trajectory = []
    for pos,rot in zip(xyz_pts,wxyz_pts):
        rospy.loginfo("  seed_state: %s", seed_state)
        target_jt_ang = ik_solver.get_ik(seed_state,
                                         pos[0],pos[1],pos[2],
                                         rot[0],rot[1],rot[2],rot[3],
                                         pos_bounds[0],pos_bounds[1],pos_bounds[2],
                                         orient_bounds[0],orient_bounds[1],orient_bounds[2]
                                         )
        rospy.loginfo("  target_jt_ang: %s", target_jt_ang)
        if target_jt_ang is None:
            return False, None
        joint_trajectory.append(target_jt_ang)
        seed_state = target_jt_ang  # for next iteration

        # Maintain current position while planning - TODO: better to replace with HEBI Trajectory Action?
        if last_valid_jt_ang is not None:
            pub_msg = JointState()
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.name = hebi_mapping
            pub_msg.position = last_valid_jt_ang
            pub_msg.velocity = []
            pub_msg.effort = []
            arm_pub.publish(pub_msg)

    return True, joint_trajectory


def execute_joint_trajectory(joint_trajectory, time_to_execute, ros_pub, ros_rate):
    global last_valid_jt_ang

    num_pts = len(joint_trajectory)
    time_start = rospy.Time.now().to_sec()  # works with simulation time too
    time_end = time_start + time_to_execute
    time_incr = (time_end - time_start) / float(num_pts)

    index = 0
    done = False
    while not rospy.is_shutdown() and not done:
        time_now = rospy.Time.now().to_sec()
        if time_now > time_end:
            done = True
        else:
            if time_now > time_start + index*time_incr:
                if index < num_pts-1:
                    index += 1
            pub_msg = JointState()
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.name = hebi_mapping
            pub_msg.position = joint_trajectory[index]
            pub_msg.velocity = []  #TODO
            pub_msg.effort = []    #TODO: Gravity compensation
            ros_pub.publish(pub_msg)
        ros_rate.sleep()
    last_valid_jt_ang = joint_trajectory[index]


def master_callback(msg):
    global msg_from_master
    msg_from_master = msg


def main():
    global hebi_mapping, arm_sub, arm_pub, msg_from_master

    ## Parse args
    parser = parse_args(sys.argv[1:])
    base_link = parser.base_link
    end_link = parser.end_link
    hebi_group_name = parser.hebi_group_name
    base_family = parser.base_family
    base_name = parser.base_name
    shoulder_family = parser.shoulder_family
    shoulder_name = parser.shoulder_name
    elbow_family = parser.elbow_family
    elbow_name = parser.elbow_name
    from_master_topic = parser.from_master_topic
    to_master_topic = parser.to_master_topic

    ## ROS setup
    rospy.init_node('trac_ik_end_traj_test')
    rate = rospy.Rate(200)

    ## HEBI setup
    # NOTE: hebiros_node must be running
    hebi_group_name = hebi_group_name
    rospy.loginfo("HEBI Group name: "+ str(hebi_group_name))
    hebi_families = [base_family,shoulder_family,elbow_family]
    rospy.loginfo("HEBI Family Names: "+ str(hebi_families))
    hebi_module_names = [base_name,shoulder_name,elbow_name]
    rospy.loginfo("HEBI Module names: "+ str(hebi_module_names))
    # [base, shoulder, elbow]
    hebi_mapping = [hebi_families[0]+'/'+hebi_module_names[0],
                    hebi_families[1]+'/'+hebi_module_names[1],
                    hebi_families[2]+'/'+hebi_module_names[2]]
    rospy.loginfo("hebi_mapping: "+ str(hebi_mapping))

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

    master_sub = rospy.Subscriber(from_master_topic, Pose, master_callback)
    master_pub = rospy.Publisher(to_master_topic, String, queue_size=1)
    arm_sub = rospy.Subscriber(hebi_group_feedback_topic, JointState, arm_callback)
    arm_pub = rospy.Publisher(hebi_group_command_topic, JointState, queue_size=1)

    ## trac_ik setup
    # Get urdf from ROS parameter server
    urdf_str = ""
    urdf_loaded = False
    while not rospy.is_shutdown() and not urdf_loaded:
        if rospy.has_param('/robot_description'):
            urdf_str = rospy.get_param('/robot_description')
            urdf_loaded = True
            rospy.loginfo("Pulled /robot_description from parameter server.")
        else:
            rospy.sleep(0.2)  # sleep for 0.2s of ROS time

    ik_solver = IK(base_link,
                   end_link,
                   urdf_string=urdf_str,
                   timeout=0.01,           # Can tune
                   epsilon=1e-4,           # Can tune
                   solve_type="Distance")  # Can tune
    rospy.loginfo("Created trac-ik-python IK instance: ik_solver")
    rospy.loginfo("  ik_solver.base_link: %s", ik_solver.base_link)
    rospy.loginfo("  ik_solver.tip_link: %s", ik_solver.tip_link)
    rospy.loginfo("  ik_solver.link_names: %s", ik_solver.link_names)
    rospy.loginfo("  ik_solver.joint_names: %s", ik_solver.joint_names)
    lb, ub = ik_solver.get_joint_limits()
    rospy.loginfo("  ik_solver.get_joint_limits()")
    rospy.loginfo("    lower_bounds: %s", lb)
    rospy.loginfo("    upper_bounds: %s", ub)
    xyz_position_bounds = [0.01, 0.01, 0.01]
    rospy.loginfo("  xyz_position_bounds: %s", xyz_position_bounds)
    orientation_bounds = [31416.0, 31416.0, 31416.0]  #NOTE: This effectly implements position only IK
    rospy.loginfo("  orientation_bounds: %s", orientation_bounds)

    # pykdl_utils setup
    robot_urdf = URDF.from_xml_string(urdf_str)
    kdl_kin = KDLKinematics(robot_urdf, base_link, end_link)
    # Wait for connections to be setup
    while not rospy.is_shutdown() and len(current_jt_pos) < len(hebi_mapping):
        rate.sleep()
    jt_angles = [current_jt_pos[motor] for motor in hebi_mapping]
    pose = kdl_kin.forward(jt_angles)
    rospy.loginfo(pose)
    cur_pos = [round(pose[0,3], 5), round(pose[1,3], 5), round(pose[2,3], 5)]  #TODO: Probably should include pose information too!!!
                                                                               #      Would need to convert homogeneous matrix to proper normalized quaternion.
    rospy.loginfo("cur_pos [x, y, z]: %s", cur_pos)

    max_step = 0.01  # m
    while not rospy.is_shutdown():
        if msg_from_master is None:
            if last_valid_jt_ang is not None:
                pub_msg = JointState()
                pub_msg.header.stamp = rospy.Time.now()
                pub_msg.name = hebi_mapping
                pub_msg.position = last_valid_jt_ang
                pub_msg.velocity = []  #TODO
                pub_msg.effort = []    #TODO: Gravity compensation
                arm_pub.publish(pub_msg)
        else:
            target_pos = [msg_from_master.position.x, msg_from_master.position.y, msg_from_master.position.z]
            rospy.loginfo("  target_pos: %s", target_pos)
            msg_from_master = None
            squared_dif = (target_pos[0]-cur_pos[0])**2.0 \
                + (target_pos[1]-cur_pos[1])**2.0 \
                + (target_pos[2]-cur_pos[2])**2.0
            if squared_dif < 1e-6:
                rospy.loginfo("  Target end-effector position is same as current position!\n"
                    + "  Please try another pt...")
            else:
                straight_line_dist = m.sqrt(squared_dif)
                rospy.loginfo("  straight_line_dist: %s [m]", straight_line_dist)
                num_pts = int(m.ceil(straight_line_dist/max_step))
                rospy.loginfo("  num_pts: %s", num_pts)
                x_dif = target_pos[0]-cur_pos[0]
                y_dif = target_pos[1]-cur_pos[1]
                z_dif = target_pos[2]-cur_pos[2]
                x_delta = x_dif / num_pts
                y_delta = y_dif / num_pts
                z_delta = z_dif / num_pts
                xyz_pts=[]
                for i in range(num_pts):
                    pt = (cur_pos[0] + (i+1)*x_delta, cur_pos[1] + (i+1)*y_delta, cur_pos[2] + (i+1)*z_delta)
                    xyz_pts.append(pt)
                wxyz_pts = [(0,0,0,1) for pt in xyz_pts]  #NOTE: Temporary - still only interested in position-only IK
                result, joint_trajectory = generate_joint_trajectory(xyz_pts=xyz_pts, wxyz_pts=wxyz_pts,
                                                                     ik_solver=ik_solver,
                                                                     pos_bounds=xyz_position_bounds,
                                                                     orient_bounds=orientation_bounds)
                if not result:
                    rospy.loginfo("  IK FAILED! Joint solution not found.\n"
                        + "  Please try another pt...")
                    master_pub.publish("failure")
                else:
                    rospy.loginfo("  IK SUCCESS! Joint solution found.")
                    rospy.loginfo("  Executing joint_trajectory...")
                    avg_vel = 0.25  #NOTE: This is not a good way to do this. I need to use some kind of trajectory filter.
                    time_to_execute = straight_line_dist / avg_vel
                    execute_joint_trajectory(joint_trajectory, time_to_execute, arm_pub, rate)
                    rospy.loginfo("  Finished executing joint_trajectory.")
                    master_pub.publish("success")

                    pose = kdl_kin.forward(last_valid_jt_ang)
                    cur_pos = [round(pose[0,3], 5), round(pose[1,3], 5), round(pose[2,3], 5)]
                    rospy.loginfo("cur_pos [x, y, z]: %s", cur_pos)
            rate.sleep()


if __name__ == '__main__':
    main()
