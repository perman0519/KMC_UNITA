# Mobility Challenge Simulator

## 개요

**Mobility Challenge Simulator**는 2025 모빌리티 챌린지 경진대회를 위해 개발된 시뮬레이션 플랫폼이다. 대회장 환경을 재현하고, 다수의 **CAV(Connected and Autonomous Vehicle)** 및 **HV(Human-driven Vehicle)** 가 공존하는 상황에서 CAV 주행 알고리즘을 개발할 수 있도록 지원한다.

본 시뮬레이터는 ROS 2 기반으로 동작하며, 개발된 알고리즘이 실제 대회 운영 환경에 추가 수정이 거의 없이 적용될 수 있도록 높은 호환성을 목표로 제작되었다. 참가자는 이를 통해 대회장을 방문하지 않더라도 멀티 에이전트 주행 알고리즘을 개발할 수 있다.

![Simulator Overview](images/Simulator_gif.gif)

---

## 튜토리얼

### 1. 설치 가이드

아래 절차에 따라 시뮬레이터를 설치한다.

```bash
# 1) 리포지토리 복제
$ cd ~
$ git clone https://github.com/cislab-kaist/Mobility_Challenge_Simulator.git
$ cd ~/Mobility_Challenge_Simulator

# 2) rosdep 초기화 및 의존성 설치
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y

# 3) 시뮬레이션 엔진 의존성 빌드 (SDL3, X11, OpenGL 등)
$ cd ~/Mobility_Challenge_Simulator/src/simulation
$ chmod +x build.sh
$ ./build.sh

# 4) ROS 2 패키지 빌드
$ cd ~/Mobility_Challenge_Simulator
$ colcon build --symlink-install
$ source install/setup.bash
```

---

### 2. 시뮬레이터 실행

ROS 2 Domain ID를 설정한 후 시뮬레이터를 실행한다.

```bash
# Domain ID 100에서 시뮬레이터 실행
$ export ROS_DOMAIN_ID=100
$ ros2 launch simulator_launch simulator_launch.py
```

---

### 3. 간단한 CAV 주행 검증

제어하려는 CAV에 해당하는 ROS 2 Domain ID를 설정한 뒤, 아래 명령어를 통해 주행 명령을 송신한다.

```bash
# CAV_01을 제어할 경우 Domain ID 1로 설정
$ export ROS_DOMAIN_ID=1
$ ros2 topic pub -r 10 /Accel geometry_msgs/msg/Accel \
    "{linear: {x: ${LIN_VEL}, \
               y: 0.0, \
               z: 0.0}, \
      angular: {x: 0.0, \
                y: 0.0, \
                z: ${ANG_VEL}}}"

# 예시
$ ros2 topic pub -r 10 /Accel geometry_msgs/msg/Accel \
    "{linear: {x: 0.2, \
               y: 0.0, \
               z: 0.0}, \
      angular: {x: 0.0, \
                y: 0.0, \
                z: 0.0}}"
```
  
- **`LIN_VEL` (m/s)**  
    차량 중심 속도 $v_x$

- **`ANG_VEL` (rad/s)**  
    차량 중심 각속도 $\omega_z$


### 4. HV 주행 파라미터 설정

HV 주행 파라미터는 `src/hv_handler/config/hv_params.yaml` 파일에서 관리된다.

> `colcon build --symlink-install` 옵션으로 빌드한 경우, YAML 파일 수정 후 **재빌드 없이 노드 재실행만으로** 변경 사항이 반영된다.

- **`problem_two`, `problem_three`**  
  시뮬레이션 문제 번호 실행 시 선택된 문제에 따라 해당 블록이 활성화됨

- **`lane_three`, `lane_two`, `roundabout`**  
  HV가 주행하는 도로 환경 또는 경로 유형

- **`init_speed` (m/s)**  
  HV의 초기 주행 속도 값이 클수록 공격적인 주행 특성

- **`lookahead` (L_d, m)**  
  경로 추종 시 참조하는 전방 거리 값이 클수록 부드럽고 완만한 주행


---

## 추가 자료

- 📘 **시뮬레이터 매뉴얼**: 시뮬레이터 조작 방법, 토픽 구조, 사용 방법 등에 대한 상세 설명을 포함한다.
  
  [매뉴얼 다운로드](https://drive.google.com/drive/folders/18i4JVSah0wi0DM1WJ_bzrPLkgHcR89dE?usp=sharing)


