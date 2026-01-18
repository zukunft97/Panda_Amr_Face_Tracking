#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <shape_msgs/SolidPrimitive.h>

const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_cpp");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 잠깐 대기 (중요!)
    ros::Duration(1.0).sleep();

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box1";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.2, 0.2};

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.1;

    // collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;

    // planning_scene_interface.applyCollisionObject(collision_object);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose1);
    move_group_interface.move();

    ROS_INFO("Obstacle added to planning scene");

    // ros::spin();
    return 0;
}
