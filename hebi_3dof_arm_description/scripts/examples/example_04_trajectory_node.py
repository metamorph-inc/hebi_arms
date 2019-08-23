#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python version of
# https://github.com/HebiRobotics/HEBI-ROS/blob/master/hebiros_basic_examples/src/example_04_trajectory_node.cpp

from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState
from actionlib.simple_action_client import SimpleActionClient
from hebiros.srv import AddGroupFromNamesSrv
from hebiros.msg import WaypointMsg, TrajectoryGoal, TrajectoryAction

# Global constants
M_PI = 3.14159265358979323846
M_PI_2 = 1.57079632679489661923
NAN = float('nan')

# Global variable and callback function used to store feedback data
feedback = JointState()


def feedback_callback(data):
    global feedback
    feedback = data


# Callback which is called when the action goal completes
def trajectory_done(state, result):
    print("Final state: {}".format(str(state)))

    for name, position, velocity, effort \
            in zip(result.final_state.name, result.final_state.position, result.final_state.velocity, result.final_state.effort):
        print("{}: ".format(name))
        print(" Position: {}".format(position))
        print(" Velocity: {}".format(velocity))
        print(" Effort: {}".format(effort))

    rospy.signal_shutdown("Trajectory Action Goal Completed")


# Callback which is called once when the action goal becomes active
def trajectory_active():
    print("Goal just went active")


# Callback which is called every time feedback is received for the action goal
def trajectory_feedback(t_feedback):
    print("Trajectory percent completion: {}".format(t_feedback.percent_complete))


def main():
    # Initialize ROS node
    rospy.init_node("example_04_trajectory_node", disable_signals=True)
    rate = rospy.Rate(200)

    group_name = "my_group"
    num_joints = 3
    num_waypoints = 5

    # Create a client which uses the service to create a group
    add_group_client = rospy.ServiceProxy("hebiros/add_group_from_names", AddGroupFromNamesSrv)

    # Create a subscriber to receive feedback from a group
    # Register feedback callback which runs when feedback is received
    feedback_subscriber = rospy.Subscriber("/hebiros/"+group_name+"/feedback/joint_state", JointState, feedback_callback, queue_size=100)

    # Construct a group using 3 known modules
    rospy.wait_for_service("hebiros/add_group_from_names")
    req_group_name = group_name
    req_names = ["base", "shoulder", "elbow"]
    req_families = ["HEBI"]
    add_group_client(req_group_name, req_names, req_families)

    # Create an action client for executing a trajectory
    client = SimpleActionClient("/hebiros/"+group_name+"/trajectory", TrajectoryAction)
    # Wait for the action server corresponding to the action client
    client.wait_for_server()

    user_input = raw_input("Press Enter/Return to execute Trajectory")

    # Construct a trajectory to be sent as an action goal
    goal = TrajectoryGoal()

    # Set the times to reach each waypoint in seconds
    times = [0, 5, 10, 15, 20]
    names = ["HEBI/base", "HEBI/shoulder", "HEBI/elbow"]

    # Wait for feedback from actuators
    while not rospy.is_shutdown() and feedback.position is not None and len(feedback.position) < len(names):
        rate.sleep()

    # Set positions, velocities, and accelerations for each waypoint and each joint
    # The following vectors have one joint per row and one waypoint per column
    positions = [[feedback.position[0], 0, M_PI_2, 0,      0],
                 [feedback.position[1], 0, M_PI_2, M_PI_2, 0],
                 [feedback.position[2], 0, 0,      M_PI_2, 0]]
    velocities = [[0, NAN, NAN, NAN, 0],
                  [0, NAN, NAN, NAN, 0],
                  [0, NAN, NAN, NAN, 0]]
    accelerations = [[0, NAN, NAN, NAN, 0],
                     [0, NAN, NAN, NAN, 0],
                     [0, NAN, NAN, NAN, 0]]

    # Construct the goal using the TrajectoryGoal format
    for i in range(num_waypoints):
        waypoint = WaypointMsg()
        waypoint.names = names
        waypoint.positions = [joint[i] for joint in positions]
        waypoint.velocities = [joint[i] for joint in velocities]
        waypoint.accelerations = [joint[i] for joint in accelerations]
        goal.waypoints.append(waypoint)
        goal.times.append(times[i])

    # Send the goal, executing the trajectory
    client.send_goal(goal, trajectory_done, trajectory_active, trajectory_feedback)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
