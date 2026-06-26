cmake_minimum_required(VERSION 3.8)
project(swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(artifact_manager src/artifact_manager.cpp)
ament_target_dependencies(artifact_manager rclcpp)


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
