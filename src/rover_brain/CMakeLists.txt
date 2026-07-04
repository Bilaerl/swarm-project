cmake_minimum_required(VERSION 3.8)
project(rover_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For generating service interfaces

# generate service interfaces
# without this, the service definitions won't be generated and the code won't compile
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PickArtifact.srv"
)

# create executables
add_executable(picker src/picker.cpp)
ament_target_dependencies(picker rclcpp)

add_executable(rover_core src/rover_core.cpp)
ament_target_dependencies(rover_core rclcpp)

# link the generated service interfaces
# this approach is used because both service definition and the nodes using it are in the same package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(picker ${cpp_typesupport_target})
target_link_libraries(rover_core ${cpp_typesupport_target})

# install cpp executables
install(TARGETS
  picker
  rover_core
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
