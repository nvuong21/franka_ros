// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  // for (size_t i = 0; i < q_start.size(); i++) {
  //   if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionExampleController: Robot is not in the expected starting position for "
  //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //     return false;
  //   }
  // }

  sub_run_control_ = node_handle.subscribe("command", 1,
    &JointPositionExampleController::callback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = 0.0;
  run_controller_ = false;
}

void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period.toSec();
  double goal_dif = 0.1;
  double delta_angle{0.0};

  if (run_controller_) {

    // step response
    // delta_angle = goal_dif;
    // for (size_t i = 0; i < 7; ++i) {
    //   if (i == 0) {
    //     position_joint_handles_[i].setCommand(initial_pose_[i] + goal_dif);
    //   }
    //   else if (i == 6){
    //     position_joint_handles_[i].setCommand(initial_pose_[i] + goal_dif);
    //   }
    // }

    // linear response
    // double T = 4;
    // if (elapsed_time_ > T) elapsed_time_ = T;
    // delta_angle = elapsed_time_ / T * goal_dif;

    // trajectory response
    delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_)) * 0.2;

  }
  else {
    delta_angle = 0.0;
    elapsed_time_ = 0.0;
  }
  // position_joint_handles_[0].setCommand(initial_pose_[0]-delta_angle);

  // for (size_t i = 0; i < 7; ++i) {
  //   if (i == 2) {
  //     position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
  //   } else {
  //     position_joint_handles_[i].setCommand(initial_pose_[i]+ delta_angle);
  //   }
  // }
}

void JointPositionExampleController::callback(const std_msgs::Bool& msg){
  ROS_INFO("Receive running command");
  run_controller_ = true;
  // elapsed_time_ = 0;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
