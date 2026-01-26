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
: "${SINGLE_CAV:=false}" # 기본값은 false (전체 실행)
: "${TARGET_CAV:=all}" # all | cav1 | cav2 | cav3 | cav4
run_simulator() {
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
  exec ros2 launch simulator_launch simulator_launch.py
}

run_algorithm() {
  : "${PROBLEM_ID:=3}" # 기본값을 1-1로 설정
  # 기본값 설정 (변수가 없을 경우 대비)
  : "${CAV1_ID:=1}"
  : "${CAV2_ID:=2}"
  : "${CAV3_ID:=3}"
  : "${CAV4_ID:=4}"

  export ROS_DOMAIN_ID=100

  case "${PROBLEM_ID}" in
    "3")
      echo "실행: 문제 3 (Mode: ${SINGLE_CAV}, Target: ${TARGET_CAV})"
      exec ros2 launch pure_pursuit_controller competition.launch.py \
        problem_id:="3" \
        cav1_id:="${CAV1_ID:-1}" \
        cav2_id:="${CAV2_ID:-2}" \
        cav3_id:="${CAV3_ID:-3}" \
        cav4_id:="${CAV4_ID:-4}" \
        target_cav:="${TARGET_CAV}"
      ;;
    "viz")
      echo "실행: map rviz"
      exec ros2 launch map_visualizer visualize.launch.py \
        cav1_id:="${CAV1_ID:-1}" \
        cav2_id:="${CAV2_ID:-2}" \
        cav3_id:="${CAV3_ID:-3}" \
        cav4_id:="${CAV4_ID:-4}" \
      ;;
    *)
      echo "잘못된 PROBLEM_ID: ${PROBLEM_ID} (3, viz 중 하나를 입력하세요)" >&2
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
