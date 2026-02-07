cmake_minimum_required(VERSION 3.16)
project(simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

option(USE_ROS2 "Enable ROS2 integration" ON)

if(USE_ROS2)
    # ROS2 패키지 찾기
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(scene_srv REQUIRED)

    # 사용자 메시지 생성
    # rosidl_generate_interfaces(${PROJECT_NAME}
    #     "msg/CustomMessage.msg"
    #     DEPENDENCIES std_msgs
    # )

    add_compile_definitions(USE_ROS2)
endif()

include(FetchContent)

# GLM
find_package(glm CONFIG QUIET)
if(NOT glm_FOUND)
    if(EXISTS "${CMAKE_BINARY_DIR}/_deps/glm-src/CMakeLists.txt")
        message(STATUS "Using existing glm in build folder")
        add_subdirectory(${CMAKE_BINARY_DIR}/_deps/glm-src ${CMAKE_BINARY_DIR}/_deps/glm-build EXCLUDE_FROM_ALL)
    else()
        message(STATUS "GLM not found, downloading via FetchContent...")
        FetchContent_Declare(
            glm
            GIT_REPOSITORY https://github.com/g-truc/glm.git
            GIT_TAG master
        )
        FetchContent_MakeAvailable(glm)
    endif()
endif()

# SDL3
find_package(SDL3 CONFIG REQUIRED)
message(STATUS "Found SDL3: ${SDL3_DIR}")

# 소스 파일
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 실행 파일
add_executable(simulation_node ${SOURCES})

# Precompiled header
target_precompile_headers(simulation_node PRIVATE src/pch.hpp)

# 인클루드 디렉토리
target_include_directories(simulation_node PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../scene_srv/include> 
)

# 디버그 정의
target_compile_definitions(simulation_node PRIVATE 
    $<$<CONFIG:Debug>:DEBUG>
)

target_link_libraries(simulation_node
    glm::glm
    SDL3::SDL3
)
# ROS2 패키지만 ament_target_dependencies()로 처리
ament_target_dependencies(simulation_node
    rclcpp
    std_msgs
    geometry_msgs
    ament_index_cpp
    scene_srv
)

# 리소스 폴더 복사 (개발용)
if(EXISTS ${CMAKE_SOURCE_DIR}/resource)
    add_custom_command(TARGET simulation_node POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/resource
            ${CMAKE_BINARY_DIR}/bin/resource
        COMMENT "Copying resource folder to build directory"
    )
endif()

# 실행 파일 출력 위치 및 RPATH 설정
set_target_properties(simulation_node PROPERTIES
    BUILD_RPATH "$ORIGIN"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

if(USE_ROS2)
    # 설치 설정
    install(TARGETS simulation_node
        DESTINATION lib/${PROJECT_NAME}
    )
    
    if(EXISTS ${CMAKE_SOURCE_DIR}/resource)
        install(DIRECTORY resource/
            DESTINATION share/${PROJECT_NAME}/resource
        )
    endif()
    
    ament_package()
endif()

