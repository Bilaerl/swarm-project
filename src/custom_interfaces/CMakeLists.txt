cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) # for generating service interfaces

# generate service interfaces
# without this, the service definitions won't be generated and the code won't compile
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PickArtifact.srv"
  "srv/DropArtifact.srv"
  "srv/RemoveArtifact.srv"
  "srv/SpawnArtifact.srv"
)

ament_package()
