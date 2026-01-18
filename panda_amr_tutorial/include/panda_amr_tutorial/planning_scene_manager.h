#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PlanningSceneManager
{
public:
  PlanningSceneManager();

  void addBox(const std::string& id,
              const std::string& frame_id,
              double x, double y, double z,
              double sx, double sy, double sz);

  void removeBox(const std::string& id);

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};
