import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from hebiros.msg import WaypointMsg, TrajectoryGoal, CommandMsg

from hebiros_utils.hebiros_wrapper import HebirosWrapper
from pykdl_utils.kdl_kinematics import KDLKinematics
from trac_ik_python.trac_ik import IK
import common.transformations as transforms


def _get_pose_from_homogeneous_matrix(homogeneous_matrix):
    pose = Pose()
    pose.position.x = round(homogeneous_matrix[0, 3], 6)
    pose.position.y = round(homogeneous_matrix[1, 3], 6)
    pose.position.z = round(homogeneous_matrix[2, 3], 6)
    quaternion = transforms.quaternion_from_matrix(homogeneous_matrix[:3, :3])
    pose.orientation.w = quaternion[0]
    pose.orientation.x = quaternion[1]
    pose.orientation.y = quaternion[2]
    pose.orientation.z = quaternion[3]
    return pose


class ArmController(object):
    def __init__(self, hebi_group_name, modules, base_link_name, end_link_name):

        self._hold_position = False
        self._hold_joint_angles = []

        # Set up HEBI ROS interface
        self.hebi_families = [module.split('/')[0] for module in modules]
        self.hebi_names = [module.split('/')[1] for module in modules]
        self.hebi_wrap = HebirosWrapper(hebi_group_name, self.hebi_families, self.hebi_names)
        self.hebi_mapping = modules

        #   Set HEBI Gains  # FIXME: Hardcoded for now  # FIXME: Uncomment if simulating
        """
        cmd_msg = CommandMsg()
        cmd_msg.name = self.hebi_mapping
        cmd_msg.settings.name = self.hebi_mapping
        cmd_msg.settings.position_gains.name = self.hebi_mapping
        cmd_msg.settings.position_gains.kp = [1.3, 2.6, 2.5, 1, 0.8]
        cmd_msg.settings.position_gains.ki = [0] * 5
        cmd_msg.settings.position_gains.kd = [0] * 5
        cmd_msg.settings.position_gains.i_clamp = [0] * 5
        self.hebi_wrap.send_command_with_acknowledgement(cmd_msg)
        """

        # Check ROS Parameter server for robot_description URDF
        urdf_str = ""
        urdf_loaded = False
        while not rospy.is_shutdown() and not urdf_loaded:
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                urdf_loaded = True
                rospy.loginfo("Pulled /robot_description from parameter server.")
            else:
                rospy.sleep(0.01)  # sleep for 10 ms of ROS time

        # pykdl_utils setup
        robot_urdf = URDF.from_xml_string(urdf_str)
        self.kdl_fk = KDLKinematics(robot_urdf, base_link_name, end_link_name)

        # trac-ik setup
        self.trac_ik = IK(base_link_name, end_link_name, urdf_string=urdf_str,
                          timeout=0.1, epsilon=1e-4, solve_type="Distance")  # FIXME: Decrease timeout as able

        # joint state publisher
        # Wait for connections to be setup
        while not rospy.is_shutdown() and len(self.hebi_wrap.get_joint_positions()) < len(self.hebi_mapping):
            rospy.sleep(0.1)
        self._joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self._active_joints = self.kdl_fk.get_joint_names()
        self.hebi_wrap.add_feedback_callback(self._joint_state_cb)
        self.hebi_wrap.add_feedback_callback(self._hold_position_cb)

    def _joint_state_cb(self, msg):
        if not rospy.is_shutdown() and self._active_joints is not None:
            jointstate = JointState()
            jointstate.header.stamp = rospy.Time.now()
            jointstate.name = self._active_joints
            jointstate.position = self.hebi_wrap.get_joint_positions()
            jointstate.velocity = []
            jointstate.effort = []
            self._joint_state_pub.publish(jointstate)

    def _hold_position_cb(self, msg):
        if not rospy.is_shutdown() and self._hold_position:
            jointstate = JointState()
            jointstate.name = self.hebi_wrap.hebi_mapping
            jointstate.position = self._hold_joint_angles
            jointstate.velocity = []
            jointstate.effort = []
            self.hebi_wrap.joint_state_publisher.publish(jointstate)

    def move_to_pose(self, target_pose, mode, move_duration=0):
        """
        :param target_pose: [x,y,z,R,P,Y] or Pose msg
        :type target_pose: list or Pose

        :param mode: "trajectory" or "command"
        :type mode: str

        :param move_duration: time in seconds to complete move
        :type move_duration: float

        :return:
        """
        xyz_bounds = [0.01, 0.01, 0.01]
        wxyz_bounds = [0.05, 0.05, 0.05]

        if not isinstance(target_pose, Pose):
            # xyz
            target_xyz = [0, 0, 0]
            for i in range(3):
                if target_pose[i] in ("", " "):
                    xyz_bounds[i] = 1e5
                else:
                    target_xyz[i] = float(target_pose[i])

            # RPY
            target_rpy = [0, 0, 0]  # Tait-Byran Extrinsic xyz Euler angles
            for i in range(3):
                if target_pose[3 + i] in ("", " "):
                    wxyz_bounds[i] = 1e5
                else:
                    target_rpy[i] = float(target_pose[3 + i])
            # wxyz
            target_wxyz = transforms.quaternion_from_euler(target_rpy[0], target_rpy[1], target_rpy[2],
                                                           axes='sxyz').tolist()
        else:
            target_xyz = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
            target_wxyz = [target_pose.orientation.w,
                           target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z]

        print("Seed angles: {}".format([round(ang,4) for ang in self.hebi_wrap.get_joint_positions()]))
        print("Target End-Effector Pose...")
        print("  xyz: {}".format(target_xyz))
        print("  wxyz: {}".format(target_wxyz))
        print("Bounds...")
        print("  xyz: {}".format(xyz_bounds))
        print("  wxyz: {}".format(wxyz_bounds))

        # Get target joint angles
        target_jt_angles = self.trac_ik.get_ik(self.hebi_wrap.get_joint_positions(),
                                               target_xyz[0], target_xyz[1], target_xyz[2],
                                               target_wxyz[0], target_wxyz[1], target_wxyz[2], target_wxyz[3],
                                               xyz_bounds[0], xyz_bounds[1], xyz_bounds[2],
                                               wxyz_bounds[0], wxyz_bounds[1], wxyz_bounds[2])

        if target_jt_angles is None:
            print("NO JOINT SOLUTION FOUND FOR END-EFFECTOR POSE "
                  "\n(x: {}, y: {}, z: {}, W: {}, X: {}, Y: {}, Z: {})"
                  "\nPLEASE TRY AGAIN.".format(*target_xyz+target_wxyz))
            return False

        else:
            if mode == "trajectory":
                goal = TrajectoryGoal()
                start_wp = WaypointMsg()
                start_wp.names = self.hebi_mapping
                start_wp.positions = self.hebi_wrap.get_joint_positions()
                start_wp.velocities = [0] * len(self.hebi_mapping)
                start_wp.accelerations = [0] * len(self.hebi_mapping)
                goal.waypoints.append(start_wp)

                end_wp = WaypointMsg()
                end_wp.names = self.hebi_mapping
                end_wp.positions = target_jt_angles
                end_wp.velocities = [0] * len(self.hebi_mapping)
                end_wp.accelerations = [0] * len(self.hebi_mapping)
                goal.waypoints.append(end_wp)
                goal.times.extend([0, move_duration])

                self._hold_position = False
                self.hebi_wrap.trajectory_action_client.send_goal_and_wait(goal)
                self._hold_joint_angles = self.hebi_wrap.get_joint_positions()
                self._hold_position = True

            else:  # mode == "command"
                cmd = JointState()
                cmd.name = self.hebi_mapping
                cmd.position = target_jt_angles
                cmd.velocity = [0] * len(self.hebi_mapping)
                cmd.effort = [0] * len(self.hebi_mapping)

                self._hold_position = False
                self.hebi_wrap.joint_state_publisher.publish(cmd)
                self._hold_joint_angles = target_jt_angles
                self._hold_position = True
            return True

    def get_end_effector_pose(self):
        h_transform_mat = self.kdl_fk.forward(self.hebi_wrap.get_joint_positions())
        return _get_pose_from_homogeneous_matrix(h_transform_mat)
