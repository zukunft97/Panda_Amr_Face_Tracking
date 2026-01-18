#include <ros/ros.h>
#include "panda_amr_tutorial/grid_motion_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_motion_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  GridMotionNode node(nh);

  ros::waitForShutdown();
  return 0;
}
