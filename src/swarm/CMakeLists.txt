cmake_minimum_required(VERSION 3.8)
project(swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# create executables
add_executable(artifact_manager src/artifact_manager.cpp)
ament_target_dependencies(artifact_manager 
  rclcpp
  ros_gz_interfaces
  custom_interfaces
  geometry_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
  gz-transport13
  gz-msgs10
)

# install needed file directories
install(DIRECTORY 
  launch  #launch files
  worlds  #gz_sim worlds
  models  #gz_sim models
  config  #config files
  DESTINATION share/${PROJECT_NAME}/
)

# install cpp executables
install(TARGETS artifact_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
