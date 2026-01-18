#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <array>
#include <string>

class PandaArmController
{
public:
  static constexpr size_t ARM_DOF = 7;
  using JointArray = std::array<double, ARM_DOF>;

  explicit PandaArmController(const std::string& planning_group);

  void moveToJointPositions(const JointArray& joint_positions);

  JointArray getCurrentJointPositions() const;

  void stop();

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
};
