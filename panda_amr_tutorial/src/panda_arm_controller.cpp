#include "panda_amr_tutorial/panda_arm_controller.h"
#include <ros/ros.h>

PandaArmController::PandaArmController(const std::string& planning_group)
  : move_group_(planning_group)
{
  move_group_.setMaxVelocityScalingFactor(0.2);
  move_group_.setMaxAccelerationScalingFactor(0.2);
}

void PandaArmController::moveToJointPositions(
  const JointArray& joint_positions)
{
  std::vector<double> joint_vec(
    joint_positions.begin(), joint_positions.end());

  move_group_.setJointValueTarget(joint_vec);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

    // ROS_INFO("1");

  bool success =
    (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // ROS_INFO("2");

  if (success)
  {
    move_group_.execute(plan);
    ROS_INFO("Joint motion executed");
  }
  else
  {
    ROS_WARN("Joint planning failed");
  }
}

PandaArmController::JointArray
PandaArmController::getCurrentJointPositions() const
{
  auto joint_vec = move_group_.getCurrentJointValues();

  JointArray joints{};
  std::copy_n(joint_vec.begin(), ARM_DOF, joints.begin());
  return joints;
}

void PandaArmController::stop()
{
  move_group_.stop();
}
