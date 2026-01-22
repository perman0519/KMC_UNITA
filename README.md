# 요약

## 환경구성

### OS
- Ubuntu 20.04 LTS (Focal Fossa)

### ROS 2
- ROS 2 Foxy Fitzroy
- 미들웨어(DDS): 기본 RMW 사용 (FastDDS)

## 폴더구조

```
/ (root)
├── Dockerfile
├── entrypoint.sh
├── README.md  # - Dependency, 코드 구조에 대한 간략한 설명
├── Mobility_Challenge_Simulator/
├── pure_pursuit_controller/
│   ├── config/
│   ├── src/
│   ├── launch/
│   │   └── competition.launch.py
│   └── package.xml
└── ...
```

## 제출 파일 형식

- 제출 폴더 이름: `TEAM_UNITA`
- 최종 압축 파일 이름: `TEAM_UNITA.tar.gz`

## 3-1) docker 빌드

```bash
cd TEAM_UNITA
docker build -t team_unita:latest .
```
---
## 컨테이너 실행 형식

```bash
sudo docker run --rm -it --net=host --ipc=host   -e RUN_MODE=sim   -e ROS_DOMAIN_ID=100   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   --name unita_sim   team_unita:latest

```
### 문제 1-1에 사용하는 알고리즘 (PROBLEM_ID=1-1)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=1-1 team_unita:latest
```

### 문제 1-2에 사용하는 알고리즘 (PROBLEM_ID=1-2)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=1-2 team_unita:latest
```

### 문제 2에 사용하는 알고리즘 (PROBLEM_ID=2)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=2 team_unita:latest
```


### 문제 2에 사용하는 알고리즘 (PROBLEM_ID=3)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=3 team_unita:latest
```

## 문제 3 (PROBLEM_ID=3)

### 0. 목표/접근

문제 3은 “사지 교차로 + HV 혼재” 상황에서 단순 거리 기반 충돌 회피만으로는 불안정해질 수 있으므로,
(A) 교차로를 ‘세그먼트(Path)’ 단위로 분해
(B) 각 세그먼트별 실제 충돌 가능한 상대 경로만 선택
(C) 누적거리 기반으로 ‘누가 먼저 충돌 지점에 도달하는지’ 예측
(D) HV는 최우선 양보(정지)
(E) 최종 속도는 Pure Pursuit 기반 조향과 결합 하는 구조로 설계했다.

1. 런타임/실행 전제
* ROS_DOMAIN_ID를 CAV id(1~4)로 사용
* Timer 기반 20Hz(50ms) 반복 제어
* QoS: SensorDataQoS()를 이용하여 pose 스트림 수신

2. 노드 인터페이스(토픽)와 상태 변수
    * Subscribe
      1. /Ego_pose
         * Ego 상태:
            * curr_x_, curr_y_, curr_yaw_
            * pose_received_ 플래그
      2. /CAV{i}_pose
         * 타 CAV 상태:
           * others_pos_[i] = {x,y}
           * others_received_[i]=true
      3. /HV_19, /HV_20
         * HV 상태:
           * others_pos_[19/20], others_received_[19/20]
    * Publish
      * /Accel:
        * linear.x: 목표 속도(target_v)
        * angular.z: target_v * curvature에 기반한 각속도(±π 제한)

3. 설정/경로 데이터 로딩(초기화)
    3.1 YAML: cav_params.yaml
    problem_three.cav{domain_id_}에서 다음을 읽는다
    * init_speed → v_
    * lookahead → ld_
    * path → my_path_ (Ego의 메인 주행 경로)

    3.2 JSON: Path 로딩 방식
    * 각 경로 파일은 다음 구조를 가정한다.
    * X: double 배열
    * Y: double 배열
    이를 Path{X,Y}로 저장한다.

    3.3 로드되는 경로 종류
      * Ego 메인 경로: my_path_
      * 타 CAV 경로: others_paths_[1..4] (자기 자신 제외)
      * HV 경로(고정): others_paths_[19], others_paths_[20] = roundabout_lane_two.json
      * 교차로 판별 기준: circle_path_ = circle.json
      * 사지교차로 세그먼트 경로(룰 구성용): path21_51, path51_46(=21_51_46), path22_25, path48_58, path49_55, path60_52, path61_47, path8_11, path9_56, path56_59, path52_24, path55_12, **path60_52_exit**

4. 교차로 “세그먼트 기반 충돌 룰” 설계(핵심)
   * 전체 모든 조합을 매 tick마다 검사하지 않고, **현재 내가 위치한 구간에서만 충돌 가능한 상대**를 검사하도록 룰 테이블을 구성한다.

    4.1 자료구조
    * Path: 경로 점열
    * ConflictRule: (other_id, other_path_ptr)
    * SegmentRule: (my_segment_ptr, conflicts[])
    * my_active_rules_: 현재 CAV의 활성 룰 벡터

    4.2 세그먼트 판별
    * is_on_path(path, x, y, threshold=0.17)로
    * 내 위치가 해당 세그먼트에 올라간 경우에만 해당 세그먼트 룰을 실행한다.
    1) 사지교차로 룰 매핑
    setup_conflict_rules()에서 domain_id_별로 다음을 정의한다.

        CAV1
        * 21_51 구간: (CAV3, 22_25)
        * 51_46 구간: (CAV2, 60_52), (CAV2, 49_55), (CAV3, 48_58)
        * 9_56 구간: (CAV4, 8_11)
        * 56_59 구간: (CAV2, 49_55), (CAV2, 60_52), (CAV4, 61_47)

        CAV2
        * 60_52 구간: (CAV1, 56_59), (CAV1, 51_46), (CAV3, 48_58)
        * 52_24 구간: (CAV3, 22_25)
        * 49_55 구간: (CAV1, 51_46), (CAV1, 56_59), (CAV4, 61_47)
        * 55_12 구간: (CAV4, 8_11)

        CAV3
        * 22_25 구간: (CAV1, 21_51), (CAV2, 52_24)
        * 48_58 구간: (CAV1, 51_46), (CAV2, 60_52)

        CAV4
        * 61_47 구간: (CAV1, 56_59), (CAV2, 49_55)
        * 8_11 구간: (CAV1, 9_56), (CAV2, 55_12)
        의도: 사지 교차로에서 실제 교차/합류 가능한 조합만 “명시적 규칙”으로 제한하여 안정적으로 양보 로직을 적용한다.

1.  충돌 예측 로직(누적거리 기반)

    6.1 공통 유틸
    * find_closest_index(path, x, y): 현재 위치에서 가장 가까운 경로 인덱스 탐색

    6.2 CAV 간 충돌(세그먼트 룰 기반)
    * check_hardcoded_collision(other_id, other_path):

    1. 상대 차량이 수신되지 않았으면 skip
    2. 상대 차량이 해당 other_path 위에 있는지 is_on_path()로 확인
    3.Ego와 상대 각각
    * 현재 인덱스를 기준으로 전방 누적거리로 스캔 (Ego≈1.0m, Other≈1.5m)
    1. 두 경로 점이 충분히 가까우면(충돌 후보)
    * 충돌 지점까지의 누적거리 비교 + deadzone(0.15m)로 우선순위 판단
    * 거의 동시 도달이면 ID가 큰 쪽이 양보

    6.3 HV 충돌(최우선)
    check_hv_collision_path_based():

    *  HV(19,20)에 대해
    * Ego 경로/ HV 경로를 전방으로 동시에 훑어 충돌 후보를 찾고,
    * HV가 더 먼저(더 가까이) 충돌 지점에 도달하면 즉시 정지

2) control_loop() 의사결정 우선순위
3. 세그먼트 룰 기반 CAV 충돌 검사
* 충돌 판단 시 target_v를 감속(0.5) 또는 정지(0.0)로 설정
* (코드 예외) CAV2가 path60_52_exit이면 특정 상황에서 2.0으로 재가속

1. HV 충돌 검사 (우선순위 1)
* true면 무조건 target_v=0.0
(1) 보조 충돌정보 기반 양보
 * deadzone + ID 비교로 정지 여부 결정

(2) Circle 구간 속도 정책
 * circle_path_ 근처(0.5m 이내)면 정지 상태가 아니면 target_v=2.0

8) Pure Pursuit 출력
compute_curvature():

* lookahead 거리 ld_ 이상 앞의 목표점을 찾고,
* 로컬 좌표계 횡오차 기반 곡률 κ를 계산한다.

최종 퍼블리시:

* /Accel.linear.x = target_v
* /Accel.angular.z = clamp(target_v * κ, -π, π)

9) 디렉터리 구조(README용)
pure_pursuit_controller/
├── src/
│   └── quiz3.cpp
├── config/
│   ├── cav_params.yaml
│   ├── circle.json
│   ├── roundabout_lane_two.json
│   ├── 8_11_fixed.json
│   ├── 9_56.json
│   ├── 21_51.json
│   ├── 22_25_fixed.json
│   ├── 48_58_fixed.json
│   ├── 49_55_fixed.json
│   ├── 52_24.json
│   ├── 55_12.json
│   ├── 56_59.json
│   ├── 60_52.json
│   ├── 60_52_fixed.json
│   └── 61_47_fixed.json
└── launch/
    └── quiz3_launch.py
    └── bridge_launch.py
