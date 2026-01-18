#pragma once

#include <string>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>

class PandaArmControllerPose
{
public:
  explicit PandaArmControllerPose(const std::string& planning_group);

  void moveToPose(const geometry_msgs::Pose& target_pose);
  void stop();

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
};
