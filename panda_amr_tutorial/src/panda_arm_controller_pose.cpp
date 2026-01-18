#include "panda_amr_tutorial/panda_arm_controller_pose.h"
#include <ros/ros.h>

PandaArmControllerPose::PandaArmControllerPose(const std::string& planning_group)
  : move_group_(planning_group)
{
  ros::Duration(1.0).sleep();
  ROS_INFO("PandaArmControllerPose initialized");
}

void PandaArmControllerPose::moveToPose(
  const geometry_msgs::Pose& target_pose)
{
  move_group_.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
    (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
    move_group_.execute(plan);
  else
    ROS_WARN("Planning failed");

  move_group_.clearPoseTargets();
}

void PandaArmControllerPose::stop()
{
  move_group_.stop();
  move_group_.clearPoseTargets();
}
