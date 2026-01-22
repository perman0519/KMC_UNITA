#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts may reference variables that are unset; avoid `set -u` failures while sourcing.
set +u
source /opt/ros/foxy/setup.bash
source /ws/install/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0

: "${RUN_MODE:=algorithm}"  # algorithm|sim (default: algorithm)

run_simulator() {
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
  exec ros2 launch simulator_launch simulator_launch.py
}

run_algorithm() {
  : "${PROBLEM_ID:=1-1}" # 기본값을 1-1로 설정

  export ROS_DOMAIN_ID=100

  case "${PROBLEM_ID}" in
    "1-1")
      echo "실행: 문제 1-1 (pure_pursuit_node)"
      exec ros2 launch pure_pursuit_controller competition.launch.py problem_id:="1-1"
      ;;
    "1-2")
      echo "실행: 문제 1-2 (Domain Bridges + Nodes)"
      exec ros2 launch pure_pursuit_controller competition.launch.py problem_id:="1-2"
      ;;
    "2")
      echo "실행: 문제 2 (quiz2_node)"
      exec ros2 launch pure_pursuit_controller competition.launch.py problem_id:="2"
      ;;
    "3")
      echo "실행: 문제 3 (quiz3_node)"
      exec ros2 launch pure_pursuit_controller competition.launch.py problem_id:="3"
      ;;
    *)
      echo "잘못된 PROBLEM_ID: ${PROBLEM_ID} (1-1, 1-2, 2, 3 중 하나를 입력하세요)" >&2
      exit 2
      ;;
  esac
}

case "${RUN_MODE}" in
  sim|simulator)
    run_simulator
    ;;
  algorithm)
    run_algorithm
    ;;
  team) # backward-compatible alias
    run_algorithm
    ;;
  *)
    echo "Invalid RUN_MODE=${RUN_MODE} (expected algorithm|sim)" >&2
    exit 2
    ;;
esac
