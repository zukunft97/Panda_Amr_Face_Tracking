cmake_minimum_required(VERSION 3.0.2)
project(panda_amr_tutorial)

find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  roscpp
  rospy
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_group_cpp src/move_group_cpp.cpp)
target_link_libraries(move_group_cpp ${catkin_LIBRARIES})

add_library(panda_arm_lib
  src/planning_scene_manager.cpp
  src/panda_arm_controller.cpp
  src/panda_arm_controller_pose.cpp
)

target_link_libraries(panda_arm_lib
  ${catkin_LIBRARIES}
)

add_executable(panda_tutorial
  src/panda_tutorial.cpp
)

target_link_libraries(panda_tutorial
  panda_arm_lib
  ${catkin_LIBRARIES}
)

add_dependencies(panda_tutorial
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(start_joint
  src/start_joint.cpp
)

target_link_libraries(start_joint
  panda_arm_lib
  ${catkin_LIBRARIES}
)

add_executable(amr_controller
  src/amr_controller.cpp
  src/grid_motion_node.cpp
  src/joint_grid_generator.cpp
)

target_link_libraries(amr_controller
  panda_arm_lib
  ${catkin_LIBRARIES}
)

add_dependencies(start_joint
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(eef_relative_move src/eef_relative_move.cpp)
target_link_libraries(eef_relative_move ${catkin_LIBRARIES})