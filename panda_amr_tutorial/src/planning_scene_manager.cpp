#include "panda_amr_tutorial/planning_scene_manager.h"
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

PlanningSceneManager::PlanningSceneManager()
{
  ros::Duration(1.0).sleep();
}

void PlanningSceneManager::addBox(const std::string& id,
                                  const std::string& frame_id,
                                  double x, double y, double z,
                                  double sx, double sy, double sz)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {sx, sy, sz};

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface_.applyCollisionObject(collision_object);
}

void PlanningSceneManager::removeBox(const std::string& id)
{
  planning_scene_interface_.removeCollisionObjects({id});
}
