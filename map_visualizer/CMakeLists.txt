cmake_minimum_required(VERSION 3.8)
project(map_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(map_visualizer_node src/map_visualizer_node.cpp)
target_link_libraries(map_visualizer_node
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)
ament_target_dependencies(map_visualizer_node
  rclcpp
  visualization_msgs
  geometry_msgs
  ament_index_cpp
)



add_executable(cav_visualizer_node src/cav_visualizer_node.cpp)
target_link_libraries(cav_visualizer_node
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)
ament_target_dependencies(cav_visualizer_node
  rclcpp
  visualization_msgs
  geometry_msgs
  ament_index_cpp
  tf2
  tf2_geometry_msgs
)

install(TARGETS map_visualizer_node cav_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
