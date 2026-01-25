cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
target_link_libraries(pure_pursuit_node
  nlohmann_json::nlohmann_json
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

add_executable(quiz2_node src/quiz2.cpp)
target_link_libraries(quiz2_node
  nlohmann_json::nlohmann_json
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
add_executable(quiz3_node src/quiz3.cpp)
target_include_directories(quiz3_node PRIVATE
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(quiz3_node
  nlohmann_json::nlohmann_json
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

add_executable(pure_pursuit_viz_node src/pure_pursuit_viz_node.cpp)
target_link_libraries(pure_pursuit_viz_node
  nlohmann_json::nlohmann_json
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

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

install(TARGETS
  pure_pursuit_node
  pure_pursuit_viz_node
  quiz2_node
  quiz3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
