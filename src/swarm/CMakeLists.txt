cmake_minimum_required(VERSION 3.8)
project(swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For generating service interfaces

# generate service interfaces
# without this, the service definitions won't be generated and the code won't compile
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RemoveArtifact.srv"
)

# create executables
add_executable(artifact_manager src/artifact_manager.cpp)
ament_target_dependencies(artifact_manager rclcpp ros_gz_interfaces)

# link the generated service interfaces
# this approach is used because both service definition and the nodes using it are in the same package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(artifact_manager ${cpp_typesupport_target})

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
