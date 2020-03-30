#!/usr/bin/env python
import rospy, os, argparse, ast
import actionlib
from copy import copy
# Trajectory controller
from crifranka_control.controllers import JointTrajectoryController
# Messages
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def restricted_float(x):
  x = float(x)
  if x < 1.0 or x > 10.0:
      raise argparse.ArgumentTypeError("%r not in range [0.0, 10.0]"%(x,))
  return x

if __name__ == '__main__':
  rospy.init_node("move_to_goal")
  # rospy.loginfo('Starting [%s] node' % node_name)

  # Go for it
  # goal0 = [0, -0.7853, 0, -2.356, 0, 1.57079, 0.7853]
  # goal1 = [-0.18793771, -0.59311493, -0.30475546, -2.42954096, -0.372601,    1.36793785, 0.4]
  # goal2 =[-0.49316243, -0.11630055,  0.2546418,  -2.11719429, -0.55126729,  2.06888692, 0.8]
  # goal3=[-0.325144482, -0.269454440,  0.617236715, -1.54583180, 0,  1.82637558,  1.1]
  # goal4=[ 0.10607209, -0.81663013,  0.10617642, -1.92972408,  0.62014378,  1.28415871, 1.2       ]
  goal = [0.02178009, -0.79307472, -0.02139999, -2.83863367, -0.0171439, 2.04561368, 0.8]
  #

  time=5
  traj = JointTrajectoryController()
  rospy.on_shutdown(traj.stop)
  traj.add_point(goal, time)
  traj.start()
  rospy.loginfo('Moving to %s. Will wait [%d] seconds' % (str(goal),time))
  traj.wait(time)
  rospy.loginfo('Shuting down node')
