#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath> 
#include <algorithm> 

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

enum class RobotState {
  MOVE_AND_AVOID 
};

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_; 
  
  RobotState current_state_; 
  
  // --- 튜닝 가능한 상수 ---
  const double OBSTACLE_DISTANCE_THRESHOLD = 0.6; // 장애물 회피 시작 거리
  const double COLLISION_DISTANCE = 0.4;          // 충돌 직전, 정지해야 하는 거리
  const double MAX_LINEAR_VELOCITY = 0.2;         // 최대 직진 속도
  const double AVOIDANCE_LINEAR_VELOCITY = 0.1;   // 회피 중 유지할 직진 속도
  const double ANGULAR_SPEED = 0.6;               // 회전 속도 (Rad/s)

public:
  SelfDrive() : rclcpp::Node("self_drive"), 
                current_state_(RobotState::MOVE_AND_AVOID) 
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile,
                                                                       callback);
    
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);

    RCLCPP_INFO(this->get_logger(), "SelfDrive Node initialized for stable moving avoidance.");
  }

  // 전방 약 30도 범위(좌우 약 15도)의 최소 유효 거리를 계산합니다.
  double get_front_min_range(const std::vector<float>& ranges)
  {
      if (ranges.empty()) return 100.0; 
      
      double min_val = 100.0;
      int n = ranges.size();
      
      // 전방 약 ±15도 범위 (대략 idx 345 ~ 15)에 대한 거리 탐색
      for (int i = 0; i <= 15; ++i) { 
          int right_idx = (n - i) % n; 
          int left_idx = i % n;
          
          if (i == 0) right_idx = 0; 
          
          if (std::isfinite(ranges[right_idx]) && ranges[right_idx] > 0.01) {
              min_val = std::min(min_val, (double)ranges[right_idx]);
          }
          if (i != 0 && std::isfinite(ranges[left_idx]) && ranges[left_idx] > 0.01) {
              min_val = std::min(min_val, (double)ranges[left_idx]);
          }
      }
      return min_val;
  }
  
  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist vel;
    
    // --- 1. 주요 거리 측정 및 유효성 확인 ---
    double front_min_range = get_front_min_range(scan->ranges); 
    
    int n = scan->ranges.size();
    
    // LiDAR 인덱스 튜닝 지점:
    // 터틀봇3는 보통 0도가 정면, 40도가 좌측, 320도가 우측입니다.
    const int LEFT_IDX = 40;
    const int RIGHT_IDX = 320;
    
    double left_range = (n > LEFT_IDX) ? (double)scan->ranges[LEFT_IDX] : 100.0; 
    double right_range = (n > RIGHT_IDX) ? (double)scan->ranges[RIGHT_IDX] : 100.0;
    
   

    // --- 2. 속도 제어 로직 (MOVE_AND_AVOID 상태) ---

    // 2-1. 충돌 회피 (가장 높은 우선 순위)
    if (front_min_range < COLLISION_DISTANCE) {
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;
      RCLCPP_FATAL(this->get_logger(), "!!! COLLISION IMMINENT (F: %1.2f). STOPPED !!!", front_min_range);
      
      // 충돌 방지 후 강제 회전하여 탈출
      if (left_range > right_range) {
          vel.angular.z = ANGULAR_SPEED; // 좌회전
      } else {
          vel.angular.z = -ANGULAR_SPEED; // 우회전
      }
      // 속도를 발행하고 함수 종료
      vel_pub_->publish(vel);
      return; 
    }

    // 2-2. 회피 기동 (직진하며 회피)
    double angular_z = 0.0;
    double linear_x = MAX_LINEAR_VELOCITY;
    
    if (front_min_range < OBSTACLE_DISTANCE_THRESHOLD) {
      // 회피가 필요한 상황: 직진 속도를 낮춤
      linear_x = AVOIDANCE_LINEAR_VELOCITY; 

      // 좌우 거리 비교를 통한 회피 방향 결정
      if (left_range > right_range) {
        // 좌측이 더 멀면 (안전) -> 좌회전 (양수 각속도)
        angular_z = ANGULAR_SPEED;
        RCLCPP_WARN(this->get_logger(), "-> A VOIDING LEFT (L: %1.2f > R: %1.2f)", left_range, right_range);
      } else if (right_range > left_range) {
        // 우측이 더 멀면 (안전) -> 우회전 (음수 각속도)
        angular_z = -ANGULAR_SPEED; 
        RCLCPP_WARN(this->get_logger(), "-> AVOIDING RIGHT (R: %1.2f > L: %1.2f)", right_range, left_range);
      }
      
    } else {
      // 장애물로부터 충분히 멀 때는 직진
      linear_x = MAX_LINEAR_VELOCITY;
      angular_z = 0.0;
    }

    vel.linear.x = linear_x;
    vel.angular.z = angular_z;

    // --- 3. 속도 발행 및 로그 출력 ---
    RCLCPP_INFO(this->get_logger(),
                "State=MOVE, F=%1.2f, L=%1.2f (Idx 90), R=%1.2f (Idx 270), linear=%1.2f, angular=%1.2f", 
                front_min_range, left_range, right_range,
                vel.linear.x, vel.angular.z);
    
    vel_pub_->publish(vel);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}