// EKF removed - using direct vehicle_speed
#include "pure_pursuit_controller/collision_checker.hpp"
#include "pure_pursuit_controller/controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using json = nlohmann::json;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode()
      : Node("pure_pursuit_node"), brake_active_(false), waiting_(true) {
    const char *env_domain = std::getenv("ROS_DOMAIN_ID");
    domain_id_ = (env_domain != nullptr) ? std::atoi(env_domain) : 1;

    // 클래스 인스턴스 생성 (EKF 제거)
    controller_ = std::make_unique<PurePursuitController>();
    checker_ = std::make_unique<CollisionChecker>();

    // 하드웨어 파라미터 셋업
    this->declare_parameter("wheelbase", 0.211);
    this->declare_parameter("max_v", 2.0);
    this->declare_parameter("max_kappa", 3.0);
    L_ = this->get_parameter("wheelbase").as_double();
    max_v_ = this->get_parameter("max_v").as_double();
    max_kappa_ = this->get_parameter("max_kappa").as_double();

    if (!init_config_and_paths()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize config or load paths.");
      return;
    }
    setup_conflict_rules();

    auto qos = rclcpp::SensorDataQoS();
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/Ego_pose", qos,
        std::bind(&PurePursuitNode::pose_callback, this,
                  std::placeholders::_1));

    // 엔코더 속도 구독 (20Hz) - 모션 캡처 보완용
    encoder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/vehicle_speed", qos,
        std::bind(&PurePursuitNode::encoder_callback, this,
                  std::placeholders::_1));

    accel_pub_ =
        this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    init_other_subs(qos);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&PurePursuitNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "Pure Pursuit Node Initialized for Domain ID: %zu", domain_id_);

    wait_starttime_ = rclcpp::Time(0); // Initialize to 0
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 모션 캡처는 차량 중심을 측정하지만, Pure Pursuit은 뒷바퀴 축 기준으로
    // 동작 wheelbase/2 = 0.1055m 만큼 뒤로 보정
    double center_x = msg->pose.position.x;
    double center_y = msg->pose.position.y;
    curr_yaw_ = msg->pose.orientation.z;

    // 뒷바퀴 축 위치 계산 (중심에서 wheelbase/2만큼 뒤로)
    double rear_offset = L_ / 8.0; // 0.1055m
    curr_x_ = center_x + rear_offset * std::cos(curr_yaw_);
    curr_y_ = center_y + rear_offset * std::sin(curr_yaw_);

    pose_received_ = true;
  }

  void encoder_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // 직접 vehicle_speed 사용 (EKF 제거)
    encoder_v_ = msg->data;
    encoder_received_ = true;
  }

  void control_loop() {
    if (waiting_) {
      if (wait_starttime_.nanoseconds() == 0) {
        wait_starttime_ = this->now();
      }

      // If time is still 0 (e.g. valid clock not received yet), wait
      if (wait_starttime_.nanoseconds() == 0) {
        return;
      }

      double elapsed = (this->now() - wait_starttime_).seconds();

      if (elapsed < 3.0) {
        geometry_msgs::msg::Accel stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        accel_pub_->publish(stop_msg);

        geometry_msgs::msg::Twist stop_twist;
        stop_twist.linear.x = 0.0;
        stop_twist.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_twist);

        return;
      } else {
        waiting_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Startup delay complete. Starting control.");
      }
    }

    if (!pose_received_)
      return;
    my_idx_ = controller_->find_closest_index(my_path_, curr_x_, curr_y_);

    double target_v = v_;
    int logical_id = domain_ids[domain_id_];

    // 1. 사지 교차로 충돌 체크
    for (const auto &segment : my_active_rules_) {
      if (checker_->is_on_path(*(segment.my_segment_ptr), curr_x_, curr_y_)) {
        for (const auto &conflict : segment.conflicts) {
          if (checker_->check_hardcoded(my_path_, my_idx_, conflict.other_id,
                                        *(conflict.other_path_ptr), others_pos_,
                                        others_received_, logical_id)) {
            target_v =
                (conflict.other_id < logical_id) ? 0.0 : 0.5; // 0.5 에서 테스트
            if (logical_id == 2 &&
                checker_->is_on_path(path60_52_exit, curr_x_, curr_y_))
              target_v = v_; // cav_param 에서 받은 v
            break;
          }
        }
      }

      if (target_v == 0.0)
        break;
    }

    // 2. HV 충돌 체크
    if (checker_->check_hv_collision(my_path_, my_idx_, others_pos_,
                                     others_paths_, others_received_)) {
      target_v = 0.0;
    }

    if (checker_->is_on_path(circle_path_, curr_x_, curr_y_) && target_v > 0.01)
      target_v = 1.5; // todo: hv 속도를 받아서 처리 해야됨

    // 3. 브레이크 로직 판별
    bool should_brake = (target_v == 0.0); // target_v가 0이면 브레이크 true
    if (should_brake && !brake_active_)
      brake_active_ = true;
    else if (!should_brake && brake_active_)
      brake_active_ = false;

    // === P 제어 ===
    const double Kp = 1.1;

    // 속도 제어 - vehicle_speed 직접 사용
    double current_v = encoder_received_ ? encoder_v_ : 0.0;
    double v_error = current_v - target_v;

    // 수정 전 P 제어 공식 (원래 공식이 맞음)
    double control_v = current_v - Kp * v_error;

    // 디버깅 로그 (5초마다 한 번씩만 출력)
    static auto last_log_time = this->now();
    auto current_time = this->now();
    if ((current_time - last_log_time).seconds() > 5.0) {
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] idx=%d/%zu, curr_v=%.2f, target_v=%.2f, "
                  "control_v=%.2f, brake=%d, encoder_rx=%d",
                  my_idx_, my_path_.X.size(), current_v, target_v, control_v,
                  brake_active_, encoder_received_);
      last_log_time = current_time;
    }

    // 횡방향 제어
    double raw_k = controller_->compute_curvature(my_path_, my_idx_, curr_x_,
                                                  curr_y_, curr_yaw_, ld_);
    double raw_omega = target_v * raw_k;
    double max_omega_limit = target_v * max_kappa_;
    double final_omega =
        std::clamp(raw_omega, -max_omega_limit, max_omega_limit);

    // 브레이크 시 각속도 0
    if (brake_active_) {
      final_omega = 0.0;
    }

    // 메시지 구성
    geometry_msgs::msg::Accel accel_msg;
    geometry_msgs::msg::Twist twist_msg;

    // 시뮬레이터용 Accel
    accel_msg.linear.x = brake_active_ ? 0.0 : target_v;
    accel_msg.angular.z =
        brake_active_ ? 0.0 : std::clamp(target_v * raw_k, -M_PI, M_PI);

    // 실제 주행용 cmd_vel (P 제어 적용)
    twist_msg.linear.x = control_v;
    twist_msg.angular.z = final_omega;

    accel_pub_->publish(accel_msg);
    cmd_vel_pub_->publish(twist_msg);
  }

  bool init_config_and_paths() {
    std::string pkg_path =
        ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
    try {
      // Declare and get CAV ID parameters
      this->declare_parameter("cav1_id", 1);
      this->declare_parameter("cav2_id", 2);
      this->declare_parameter("cav3_id", 3);
      this->declare_parameter("cav4_id", 4);

      domain_ids[this->get_parameter("cav1_id").as_int()] = 1;
      domain_ids[this->get_parameter("cav2_id").as_int()] = 2;
      domain_ids[this->get_parameter("cav3_id").as_int()] = 3;
      domain_ids[this->get_parameter("cav4_id").as_int()] = 4;

      YAML::Node config = YAML::LoadFile(pkg_path + "/config/cav_params.yaml");

      int my_logical_id = domain_ids[domain_id_];
      std::string my_key = "cav" + std::to_string(my_logical_id);
      v_ = config["problem_three"][my_key]["init_speed"].as<double>(1.0);
      ld_ = config["problem_three"][my_key]["lookahead"].as<double>(0.5);
      my_path_ = load_path_json(
          pkg_path + "/config/" +
          config["problem_three"][my_key]["path"].as<std::string>());

      // 모든 경로 파일 로드
      circle_path_ = load_path_json(pkg_path + "/config/circle.json");
      path8_11 = load_path_json(pkg_path + "/config/8_11_fixed.json");
      path9_56 = load_path_json(pkg_path + "/config/9_56.json");
      path21_51 = load_path_json(pkg_path + "/config/21_51.json");
      path22_25 = load_path_json(pkg_path + "/config/22_25.json");
      path48_58 = load_path_json(pkg_path + "/config/48_58_fixed.json");
      path49_55 = load_path_json(pkg_path + "/config/49_55_fixed.json");
      path51_46 = load_path_json(pkg_path + "/config/21_51_46.json");
      path52_24 = load_path_json(pkg_path + "/config/52_24.json");
      path55_12 = load_path_json(pkg_path + "/config/55_12.json");
      path56_59 = load_path_json(pkg_path + "/config/56_59.json");
      path60_52 = load_path_json(pkg_path + "/config/60_52.json");
      path61_47 = load_path_json(pkg_path + "/config/61_47_fixed.json");
      path60_52_exit = load_path_json(pkg_path + "/config/60_52_fixed.json");

      others_paths_[19] =
          load_path_json(pkg_path + "/config/roundabout_lane_two.json");
      others_paths_[20] =
          load_path_json(pkg_path + "/config/roundabout_lane_two.json");

      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML/JSON Loading Error: %s", e.what());
      return false;
    }
  }

  void init_other_subs(rclcpp::SensorDataQoS qos) {
    for (int i = 1; i <= 4; ++i) {
      if (i == domain_ids[domain_id_])
        continue;
      int tid = this->get_parameter("cav" + std::to_string(i) + "_id").as_int();

      other_subs_[i] =
          this->create_subscription<geometry_msgs::msg::PoseStamped>(
              "/CAV" + std::to_string(tid) + "_pose", qos,
              [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->others_pos_[i] = {msg->pose.position.x,
                                        msg->pose.position.y};
                this->others_received_[i] = true;
              });
    }

    other_subs_[19] =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/HV_19", qos,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              this->others_pos_[19] = {msg->pose.position.x,
                                       msg->pose.position.y};
              this->others_received_[19] = true;
            });
    other_subs_[20] =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/HV_20", qos,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              this->others_pos_[20] = {msg->pose.position.x,
                                       msg->pose.position.y};
              this->others_received_[20] = true;
            });
  }

  void setup_conflict_rules() { // 사지 교차로에서 rule base
    my_active_rules_.clear();
    int tid = domain_ids[domain_id_];
    if (tid == 1) {
      my_active_rules_.push_back({&path21_51, {{3, &path22_25}}});
      my_active_rules_.push_back(
          {&path51_46, {{2, &path60_52}, {2, &path49_55}, {3, &path48_58}}});
      my_active_rules_.push_back({&path9_56, {{4, &path8_11}}});
      my_active_rules_.push_back(
          {&path56_59, {{2, &path49_55}, {2, &path60_52}, {4, &path61_47}}});
    } else if (tid == 2) {
      my_active_rules_.push_back(
          {&path60_52, {{1, &path56_59}, {1, &path51_46}, {3, &path48_58}}});
      my_active_rules_.push_back({&path52_24, {{3, &path22_25}}});
      my_active_rules_.push_back(
          {&path49_55, {{1, &path51_46}, {1, &path56_59}, {4, &path61_47}}});
      my_active_rules_.push_back({&path55_12, {{4, &path8_11}}});
    } else if (tid == 3) {
      my_active_rules_.push_back(
          {&path22_25, {{1, &path21_51}, {2, &path52_24}}});
      my_active_rules_.push_back(
          {&path48_58, {{1, &path51_46}, {2, &path60_52}}});
    } else if (tid == 4) {
      my_active_rules_.push_back(
          {&path61_47, {{1, &path56_59}, {2, &path49_55}}});
      my_active_rules_.push_back(
          {&path8_11, {{1, &path9_56}, {2, &path55_12}}});
    }
  }

  Path load_path_json(const std::string &path) {
    Path p;
    std::ifstream f(path);
    if (f.is_open()) {
      json j;
      f >> j;
      p.X = j["X"].get<std::vector<double>>();
      p.Y = j["Y"].get<std::vector<double>>();
    }
    return p;
  }

  // 멤버 변수 (EKF 제거)
  std::unique_ptr<PurePursuitController> controller_;
  std::unique_ptr<CollisionChecker> checker_;

  size_t domain_id_;
  int my_idx_ = 0;
  double curr_x_, curr_y_, curr_yaw_, v_, ld_, L_, max_v_, max_kappa_;
  double encoder_v_ = 0.0; // vehicle_speed 직접 사용
  bool pose_received_ = false, brake_active_, encoder_received_ = false;

  Path my_path_, circle_path_, path8_11, path9_56, path21_51, path22_25,
      path48_58, path49_55, path51_46, path52_24, path55_12, path56_59,
      path60_52, path61_47, path60_52_exit;

  std::map<int, Point> others_pos_;
  std::map<int, bool> others_received_;
  std::map<int, Path> others_paths_;
  std::vector<SegmentRule> my_active_rules_;
  std::map<int, int> domain_ids;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_;
  std::map<int,
           rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      other_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time wait_starttime_;
  bool waiting_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
