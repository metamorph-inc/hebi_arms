#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: end_eff_pos_ctrl_class.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 02/05/2018
# Edit Date: 02/05/2018
#
# Description:
# Class to control position of arm end effector
'''

import math as m

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from hebiros.srv import AddGroupFromNamesSrv
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from trac_ik_python.trac_ik import IK


class CtrlEndEffPos(object):
    """Control end effector position

    Attributes
        hebi_group_name     (str):
        hebi_families       (list of str): HEBI actuator families [base,..., tip]
        hebi_names          (list of str): HEBI actuator names [base,..., tip]
        from_master_topic   (str):
        to_master_topic     (str):
        base_link_name      (str): arm base link name in urdf
        end_link_name       (str): end base link name in urdf
        ik_timeout          (float):
        ik_epsilon          (float):
        ik_solve_type       (str): https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib/
        ik_position_bounds  (list of float): [x_pos_bound, y_pos_bound, z_pos_bound]
    """
    def __init__(self, hebi_group_name, hebi_families, hebi_names,
                 from_master_topic, to_master_topic,
                 base_link_name, end_link_name,
                 ik_timeout, ik_epsilon, ik_solve_type, ik_position_bounds):

        self.hebi_mapping = []
        self.current_jt_pos = {}
        self.current_jt_vel = {}
        self.current_jt_eff = {}
        self._last_valid_jt_ang = []
        self._msg_from_master = None

        ## ROS node setup
        rospy.init_node('end_eff_pos_ctrl_node')  # Should be renamed in .launch
        self._rate = rospy.Rate(200)  #NOTE: Could set via arg

        ## HEBI setup - NOTE: hebiros_node must be running
        self.hebi_group_name = hebi_group_name
        rospy.loginfo("HEBI Group name: "+ str(hebi_group_name))
        self.hebi_families = hebi_families
        rospy.loginfo("HEBI families: "+ str(hebi_families))
        self.hebi_names = hebi_names
        rospy.loginfo("HEBI names: "+ str(hebi_names))
        self.hebi_mapping = [family+'/'+name for family,name in zip(hebi_families,hebi_names)]
        rospy.loginfo("self.hebi_mapping: "+ str(self.hebi_mapping))

        # Create a service client to create HEBI group
        set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        # Topic to receive feedback from HEBI group
        hebi_group_feedback_topic = "/hebiros/"+hebi_group_name+"/feedback/joint_state"
        # Topic to send commands to HEBI group
        hebi_group_command_topic = "/hebiros/"+hebi_group_name+"/command/joint_state"
        # Call the /hebiros/add_group_from_names service to create a group
        rospy.wait_for_service('/hebiros/add_group_from_names')
        set_hebi_group(hebi_group_name,hebi_names,hebi_families)

        ## ROS pub/sub setup
        self._master_sub = rospy.Subscriber(from_master_topic, Pose, self._master_cb)
        self._master_pub = rospy.Publisher(to_master_topic, String, queue_size=1)
        self._arm_sub = rospy.Subscriber(hebi_group_feedback_topic, JointState, self._arm_cb)
        self._arm_pub = rospy.Publisher(hebi_group_command_topic, JointState, queue_size=1)

        ## Simulation
        self._base_pub = rospy.Publisher("/to_x5_base", Float32, queue_size=1)
        self._shoulder_pub = rospy.Publisher("/to_x5_shoulder", Float32, queue_size=1)
        self._elbow_pub = rospy.Publisher("/to_x5_elbow", Float32, queue_size=1)

        ## trac_ik setup
        # Get urdf from ROS parameter server
        urdf_str = ""
        urdf_loaded = False
        while not rospy.is_shutdown() and not urdf_loaded:
            if rospy.has_param('/robot_description'):  #NOTE: Could set via arg
                urdf_str = rospy.get_param('/robot_description')
                urdf_loaded = True
                rospy.loginfo("Pulled /robot_description from parameter server.")
            else:
                rospy.sleep(0.2)  # sleep for 0.2s of ROS time

        self._ik_solver = IK(base_link_name,
                             end_link_name,
                             urdf_string=urdf_str,
                             timeout=ik_timeout,
                             epsilon=ik_epsilon,
                             solve_type=ik_solve_type)
        rospy.loginfo("  self.ik_solver.base_link: %s", self._ik_solver.base_link)
        rospy.loginfo("  self.ik_solver.tip_link: %s", self._ik_solver.tip_link)
        rospy.loginfo("  self.ik_solver.link_names: %s", self._ik_solver.link_names)
        rospy.loginfo("  self.ik_solver.joint_names: %s", self._ik_solver.joint_names)
        lb, ub = self._ik_solver.get_joint_limits()
        rospy.loginfo("  self.ik_solver.get_joint_limits()")
        rospy.loginfo("    lower_bounds: %s", lb)
        rospy.loginfo("    upper_bounds: %s", ub)
        self._xyz_position_bounds = ik_position_bounds
        rospy.loginfo("  self._xyz_position_bounds: %s", self._xyz_position_bounds)
        self._orientation_bounds = [31416.0, 31416.0, 31416.0]  #NOTE: This implements position-only IK
        rospy.loginfo("  self._orientation_bounds: %s", self._orientation_bounds)

        ## pykdl_utils setup
        robot_urdf = URDF.from_xml_string(urdf_str)
        self._kdl_kin = KDLKinematics(robot_urdf, base_link_name, end_link_name)
        # Wait for connections to be setup
        while not rospy.is_shutdown() and len(self.current_jt_pos) < len(self.hebi_mapping):
            rospy.sleep(0.2)  # sleep for 0.2s of ROS time
        jt_angles = [self.current_jt_pos[motor] for motor in self.hebi_mapping]
        pose = self._kdl_kin.forward(jt_angles)
        rospy.loginfo("current pose: %s", pose)
        self.current_eff_pos = [round(pose[0,3], 5), round(pose[1,3], 5), round(pose[2,3], 5)]
        rospy.loginfo("self.current_eff_pos [x, y, z]: %s", self.current_eff_pos)
        #TODO: Probably should include pose information too!!!
        #      Would need to convert homogeneous matrix to proper normalized quaternion.

    def start(self):
        ## main loop
        max_step = 0.01  # m - NOTE: Could set via message field
        while not rospy.is_shutdown():
            if self._msg_from_master is None:
                if self._last_valid_jt_ang is not None:
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = self.hebi_mapping
                    msg.position = self._last_valid_jt_ang
                    msg.velocity = []  #TODO
                    msg.effort = []    #TODO: Gravity compensation
                    self._arm_pub.publish(msg)
            else:
                target_pos = [self._msg_from_master.position.x, self._msg_from_master.position.y, self._msg_from_master.position.z]
                rospy.loginfo("  target_pos: %s", target_pos)
                self._msg_from_master = None
                squared_dif = (target_pos[0]-self.current_eff_pos[0])**2.0 \
                    + (target_pos[1]-self.current_eff_pos[1])**2.0 \
                    + (target_pos[2]-self.current_eff_pos[2])**2.0
                if squared_dif < 1e-6:
                    rospy.loginfo("  Target end-effector position is same as current position!\n"
                        + "  Please try another pt...")
                else:
                    straight_line_dist = m.sqrt(squared_dif)
                    rospy.loginfo("  straight_line_dist: %s [m]", straight_line_dist)
                    num_pts = int(m.ceil(straight_line_dist/max_step))
                    rospy.loginfo("  num_pts: %s", num_pts)
                    x_dif = target_pos[0]-self.current_eff_pos[0]
                    y_dif = target_pos[1]-self.current_eff_pos[1]
                    z_dif = target_pos[2]-self.current_eff_pos[2]
                    x_delta = x_dif / num_pts
                    y_delta = y_dif / num_pts
                    z_delta = z_dif / num_pts
                    xyz_pts=[]
                    for i in range(num_pts):
                        pt = (self.current_eff_pos[0] + (i+1)*x_delta,
                              self.current_eff_pos[1] + (i+1)*y_delta,
                              self.current_eff_pos[2] + (i+1)*z_delta)
                        xyz_pts.append(pt)
                    wxyz_pts = [(0,0,0,1) for pt in xyz_pts]  #NOTE: Temporary - still only interested in position-only IK
                    result, joint_trajectory = self._generate_joint_trajectory(xyz_pts=xyz_pts, wxyz_pts=wxyz_pts)
                    if not result:
                        rospy.loginfo("  IK FAILED! Joint solution not found.\n"
                            + "  Please try another pt...")
                        self._master_pub.publish("failure")
                    else:
                        rospy.loginfo("  IK SUCCESS! Joint solution found.")
                        rospy.loginfo("  Executing joint_trajectory...")
                        avg_vel = 0.25  #NOTE: This is not a good way to do this. I need to use some kind of trajectory filter.
                        time_to_execute = straight_line_dist / avg_vel
                        self._execute_joint_trajectory(joint_trajectory, time_to_execute)
                        rospy.loginfo("  Finished executing joint_trajectory.")
                        self._master_pub.publish("success")

                        pose = self._kdl_kin.forward(self._last_valid_jt_ang)
                        self.current_eff_pos = [round(pose[0,3], 5), round(pose[1,3], 5), round(pose[2,3], 5)]                                                                             #      Would need to convert homogeneous matrix to proper normalized quaternion.
                        rospy.loginfo("self.current_eff_pos [x, y, z]: %s", self.current_eff_pos)
                self._rate.sleep()


    def _arm_cb(self, msg):
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name not in self.hebi_mapping:
                print("WARNING: arm_callback - unrecognized name!!!")
            else:
                self.current_jt_pos[name] = pos
                self.current_jt_vel[name] = vel
                self.current_jt_eff[name] = eff


    def _master_cb(self, msg):
        self._msg_from_master = msg


    def _generate_joint_trajectory(self, xyz_pts, wxyz_pts):
        seed_state = [self.current_jt_pos[motor] for motor in self.hebi_mapping]
        joint_trajectory = []
        for pos,rot in zip(xyz_pts,wxyz_pts):
            rospy.loginfo("  seed_state: %s", seed_state)
            target_jt_ang = self._ik_solver.get_ik(seed_state,
                                                   pos[0],pos[1],pos[2],
                                                   rot[0],rot[1],rot[2],rot[3],
                                                   self._xyz_position_bounds[0],
                                                   self._xyz_position_bounds[1],
                                                   self._xyz_position_bounds[2],
                                                   self._orientation_bounds[0],
                                                   self._orientation_bounds[1],
                                                   self._orientation_bounds[2])
            rospy.loginfo("  target_jt_ang: %s", target_jt_ang)
            if target_jt_ang is None:
                return False, None
            joint_trajectory.append(target_jt_ang)
            seed_state = target_jt_ang  # for next iteration

            # Maintain current position while planning - TODO: better to replace with HEBI Trajectory Action?
            if self._last_valid_jt_ang is not None:
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = self.hebi_mapping
                msg.position = self._last_valid_jt_ang
                msg.velocity = []
                msg.effort = []
                self._arm_pub.publish(msg)

        return True, joint_trajectory


    def _execute_joint_trajectory(self, joint_trajectory, time_to_execute):

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
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = self.hebi_mapping
                msg.position = joint_trajectory[index]
                msg.velocity = []  #TODO
                msg.effort = []    #TODO: Gravity compensation
                self._arm_pub.publish(msg)
                ## Simulation
                self._base_pub.publish(joint_trajectory[index][0])
                self._shoulder_pub.publish(joint_trajectory[index][1])
                self._elbow_pub.publish(joint_trajectory[index][2])
            self._rate.sleep()
        self._last_valid_jt_ang = joint_trajectory[index]
