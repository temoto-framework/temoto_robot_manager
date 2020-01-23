cmake_minimum_required(VERSION 2.8.3)
project(temoto_robot_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cmake_modules
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  temoto_core
  temoto_action_engine
  temoto_context_manager
  temoto_er_manager
)

add_service_files(
  FILES
  
  RobotLoad.srv
  RobotPlan.srv
  RobotExecute.srv
  RobotSetTarget.srv
  RobotSetMode.srv
  RobotGetVizInfo.srv
  RobotGetTarget.srv
  RobotGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  temoto_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp
  CATKIN_DEPENDS 
    roscpp
    std_msgs 
    geometry_msgs
    visualization_msgs
    temoto_core
    temoto_action_engine
    temoto_context_manager
    temoto_er_manager
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
target_link_libraries(temoto_robot_manager ${catkin_LIBRARIES} yaml-cpp)
