cmake_minimum_required(VERSION 3.5)
project(hv_handler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(scene_srv REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(hv_handler_node src/hv_handler_node.cpp)
ament_target_dependencies(hv_handler_node 
                          rclcpp
                          geometry_msgs
                          ament_index_cpp
                          nlohmann_json
                          scene_srv)
target_link_libraries(hv_handler_node
  yaml-cpp
)

install(TARGETS
  hv_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
