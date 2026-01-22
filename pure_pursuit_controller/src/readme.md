
# Pure Pursuit Controller for Autonomous Driving (ROS2)

이 프로젝트는 **ROS2 Foxy** 환경에서 **Pure Pursuit** 알고리즘을 기반으로 한 자율주행 제어 시스템입니다. 단일 경로 추종부터 시작하여 다중 차량(CAV) 간 충돌 회피, 차선 변경 전략, 그리고 복잡한 사지/원형 교차로에서의 V2X 협력 주행 로직을 포함합니다.

## 🛠 Tech Stack

* **OS**: Ubuntu 20.04 LTS
* **Middleware**: ROS2 Foxy Fitzroy
* **Language**: C++ (14/17)
* **Dependencies**:
* `rclcpp`, `geometry_msgs`
* `nlohmann-json-dev` (JSON 경로 로딩)
* `libyaml-cpp-dev` (설정 파라미터 관리)



## 📂 Directory Structure

```text
pure_pursuit_controller/
├── config/                 # 주행 경로(JSON) 및 파라미터(YAML)
│   ├── cav_params.yaml     # 속도, Look-ahead 거리 등 설정
│   ├── highway_line1~3.json
│   ├── circle.json         # 원형 교차로 경로
│   └── ... (기타 세그먼트 경로)
├── launch/                 # 실행 스크립트
│   ├── quiz3_launch.py
│   └── bridge_launch.py
├── src/                    # 소스 코드
│   ├── pure_pursuit_node.cpp (Problem 1)
│   ├── quiz2.cpp             (Problem 2)
│   └── quiz3.cpp             (Problem 3)
├── CMakeLists.txt
└── package.xml

```

---

## 🚀 Core Algorithms

### 1. Longitudinal & Lateral Control (Pure Pursuit)

차량의 현재 위치에서 전방의 목표점(Look-ahead Point)을 실시간으로 추적하여 조향 곡률을 계산합니다.

* **곡률 계산 공식**:



*(여기서 는 Look-ahead Distance, $y_{local}$은 로컬 좌표계에서의 횡방향 오차)*
* **최종 출력**: 선속도 와 각속도 를 통해 차량을 제어합니다.

### 2. Collision Avoidance & Yielding Logic

* **누적 거리 기반 예측**: 자차와 상대 차량의 경로 점을 전방 1.5m까지 스캔하여, 경로 간 거리가 0.2m 이하가 되는 '충돌 지점'을 탐색합니다.
* **우선순위 결정**:
* 충돌 지점까지의 남은 거리가 더 먼 차량이 양보(정지)합니다.
* 거리 차이가 임계값(0.15m) 이내일 경우 `ROS_DOMAIN_ID`가 낮은 차량이 우선권을 가집니다.



---

## 🚦 Problem Specific Features

### [Problem 1] 기초 경로 추종 및 2대 간 교차

* **목표**: 정해진 체크포인트(노드)를 순서대로 통과하며 완주.
* **기능**: 두 대의 CAV가 동일한 공간을 공유할 때 경로 스캔을 통한 상호 양보 주행.

### [Problem 2] 차선 변경 및 장애물(HV) 대응

* **차선 점유 인식**: 1·2·3차선 경로상의 HV(Human Vehicle) 존재 여부를 실시간 판단.
* **회피 전략**:
* 현재 차선 차단 시 한 단계씩 차선 변경().
* **28~31번 구간**: 장애물 발견 시 1차선으로 강제 회피하는 특수 구간 로직 적용.


* **안전 로직**: 합류 구간 일시 정지 및 후방 차량 근접 시 가속 회피().

### [Problem 3] 복합 교차로 및 다중 V2X 협력

* **세그먼트(Segment) 기반 충돌 룰**: 교차로의 특정 구간(Path)에 진입했을 때만 관련된 상대 차량을 검사하여 연산 효율 극대화.
* **HV 최우선 양보**: 통신이 되지 않는 HV(19, 20번) 발견 시 무조건 양보 및 안전거리 확보.
* **교착 상태(Deadlock) 방지**: 양보 대상 차량이 구간을 완전히 통과할 때까지 상태를 유지(Latching)하는 로직 포함.
* **원형 교차로**: 진입/탈출 시 별도 속도 정책()을 적용하여 교통 흐름 최적화.

---

## 🔧 Installation & Build

1. **의존성 설치**
```bash
sudo apt update
sudo apt install ros-foxy-yaml-cpp nlohmann-json3-dev

```


2. **워크스페이스 빌드**
```bash
cd ~/ros2_ws
colcon build --packages-select pure_pursuit_controller
source install/setup.bash

```


3. **실행 (예: 문제 3)**
```bash
# 터미널 1: 브릿지 및 시뮬레이터 연결
ros2 launch pure_pursuit_controller bridge_launch.py

# 터미널 2: 제어 노드 실행
ros2 launch pure_pursuit_controller quiz3_launch.py

```



---

## 📝 Authors

* **Team**: UNITA (Incheon National University AI Autonomous Driving Club)
* **Developer**: 송준상, 정우진, 한다인, 조재민, 이다빈, 이기현

---
