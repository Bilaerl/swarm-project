cmake_minimum_required(VERSION 3.8)
project(rover_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For generating service interfaces
find_package(swarm REQUIRED) # For using the service interfaces defined in the swarm package
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# generate service interfaces
# without this, the service definitions won't be generated and the code won't compile
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PickArtifact.srv"
  "srv/DropArtifact.srv"
)

# create executables
add_executable(picker src/picker.cpp)
ament_target_dependencies(picker rclcpp swarm)

add_executable(rover_core src/rover_core.cpp)
ament_target_dependencies(rover_core rclcpp)

add_executable(image_processor src/image_processor.cpp)
ament_target_dependencies(image_processor
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# link the generated service interfaces
# this approach is used because both service definition and the nodes using it are in the same package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(picker ${cpp_typesupport_target})
target_link_libraries(rover_core ${cpp_typesupport_target})

# install needed file directories
install(DIRECTORY 
  launch  #launch files
  config  #config files
  DESTINATION share/${PROJECT_NAME}/
)

# install cpp executables
install(TARGETS
  picker
  rover_core
  image_processor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
