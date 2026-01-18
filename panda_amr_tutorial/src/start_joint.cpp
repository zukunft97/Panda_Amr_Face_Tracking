#include <ros/ros.h>
#include "panda_amr_tutorial/panda_arm_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_joint_array_control");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start(); 

  PandaArmController arm("panda_arm");

  PandaArmController::JointArray target_joints = {
    7.322118496522308e-05, -0.7853116683289186, 9.577233898453416e-05, -2.356122405050535, 2.6367573672905566e-05, 3.1454202351145444, 0.7853191739090496
  };

  // ros::Duration(1.0).sleep();
  arm.moveToJointPositions(target_joints);

  // ros::waitForShutdown();

  ros::Duration(0.5).sleep();
  return 0;
}
