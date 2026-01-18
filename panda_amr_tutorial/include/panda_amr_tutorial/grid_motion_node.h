#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "panda_amr_tutorial/joint_grid_generator.h"
#include "panda_amr_tutorial/panda_arm_controller.h"

class GridMotionNode
{
public:
  explicit GridMotionNode(ros::NodeHandle& nh);

private:
  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void move();

  PandaArmController arm_;
  JointGrid grid_;

  size_t x_, y_;
  ros::Subscriber sub_;
};
