cmake_minimum_required(VERSION 2.8.3)
project(temoto_robot_manager)

add_compile_options(-std=c++14)
option(TEMOTO_ENABLE_TRACING "Use tracer" OFF)

if(TEMOTO_ENABLE_TRACING)
  add_compile_options(-Denable_tracing)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cmake_modules
  moveit_ros_planning_interface
  move_base_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  temoto_core
  temoto_er_manager
  temoto_resource_registrar
  yaml-cpp
)

add_service_files(
  FILES
  
  RobotLoad.srv
  RobotPlanManipulation.srv
  RobotExecutePlan.srv
  RobotSetTarget.srv
  RobotSetMode.srv
  RobotGetVizInfo.srv
  RobotGetTarget.srv
  RobotGetNamedTargets.srv
  RobotGetConfig.srv
  RobotNavigationGoal.srv
  RobotGripperControlPosition.srv
  GripperControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  temoto_core
  temoto_resource_registrar
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    std_msgs 
    geometry_msgs
    temoto_core
    temoto_er_manager
    yaml-cpp
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_robot_manager 
  src/robot_manager.cpp
  src/robot_manager_node.cpp
  src/robot.cpp
  src/robot_config.cpp
  src/robot_features.cpp
)
add_dependencies(temoto_robot_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(temoto_robot_manager ${catkin_LIBRARIES})
