cmake_minimum_required(VERSION 3.8)
project(temoto_robot_manager)

add_compile_options(-std=c++17)
option(TEMOTO_ENABLE_TRACING "Use tracer" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(temoto_process_manager REQUIRED)
find_package(temoto_resource_registrar REQUIRED)
find_package(rosidl_default_generators REQUIRED)

  # temoto_core  
  # yaml-cpp

find_package(Boost REQUIRED COMPONENTS filesystem serialization program_options)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomFeedback.msg"
  "msg/NavigationFeedback.msg"
  "srv/CancelNavigationGoal.srv"
  "srv/CustomRequest.srv"
  "srv/CustomRequestPreempt.srv"
  "srv/GripperControl.srv"
  "srv/NavigationGoal.srv"
  "srv/RobotExecutePlan.srv"
  "srv/RobotGetConfig.srv"
  "srv/RobotGetNamedTargets.srv"
  "srv/RobotGetTarget.srv"
  "srv/RobotGetVizInfo.srv"
  "srv/RobotGripperControlPosition.srv"
  "srv/RobotLoad.srv"
  "srv/RobotPlanManipulation.srv"
  "srv/RobotSetMode.srv"
  "srv/RobotSetTarget.srv"
  DEPENDENCIES rr_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)


# Include directories
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${rr_interfaces_INCLUDE_DIRS}
    ${temoto_resource_registrar_INCLUDE_DIRS}
    ${temoto_resource_registrar_INCLUDE_DIRS}/rr
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # ${CMAKE_CURRENT_SOURCE_DIR}/temoto_robot_manager/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_process_manager/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_resource_registrar/include
)


#
# Robot Manager
#
# add_executable(temoto_robot_manager
#   src/robot_manager.cpp
#   src/robot_manager_node.cpp
#   src/robot.cpp
#   src/robot_config.cpp
#   src/robot_features.cpp
#   src/robot_common_procedures.cpp
#   src/custom_plugin_helper.cpp
#   src/navigation_plugin_helper.cpp
# )

# target_include_directories(temoto_robot_manager PUBLIC
#   ${rclcpp_INCLUDE_DIRS}
#   ${temoto_resource_registrar_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/temoto_process_manager/include
# )

# ament_target_dependencies(temoto_robot_manager
#   rclcpp
#   std_msgs
#   geometry_msgs
#   rr_interfaces
#   temoto_resource_registrar
# )

# target_link_libraries(temoto_robot_manager
#   rr_core 
#   Boost::filesystem
#   Boost::serialization
#   Boost::program_options
# )

# rosidl_target_interfaces(temoto_robot_manager
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

  
# install(TARGETS temoto_robot_manager
#   DESTINATION lib/${PROJECT_NAME}
#   INCLUDES DESTINATION include/${PROJECT_NAME})


# Install .srv files
install(DIRECTORY srv
DESTINATION share/${PROJECT_NAME})

  
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()






# add_dependencies(temoto_robot_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(temoto_robot_manager 
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${class_loader_LIBRARIES}
# )
