#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {

    // Subscribers
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::laserscan_callback, this, _1));

    // Publishers
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for control loop  (10Hz)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol Node initialized and ready!");
  }

  void stop() {
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // We have lasercan from -pi to pi with 100 ranges on Simulation

    size_t front_idx = std::round(
        (-msg->angle_min) / msg->angle_increment); // Front IDx (angle ~0.0 rad)
    const double front_con_rad = 20.0 * M_PI / 180; // 20 degrees front
    size_t cone_rays = std::round(front_con_rad / msg->angle_increment);
    size_t front_start_idx = front_idx - cone_rays / 2;
    size_t front_end_idx = front_idx + cone_rays / 2 + 1;

    // Begin and End of the range;
    auto begin = msg->ranges.begin();

    // Check for obstalc in front cone
    float front_min =
        *std::min_element(begin + front_start_idx, begin + front_end_idx);

    if (front_min > min_detection_) {
      direction_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "No obstacles, moving forward");
    } else {
      // analyze 180 degrees area -pi/2 to pi/2
      size_t left_idx =
          std::round((-M_PI / 2 - msg->angle_min) / msg->angle_increment);
      size_t right_idx =
          std::round((M_PI / 2 - msg->angle_min) / msg->angle_increment);

      auto first = begin + left_idx;
      auto last = begin + right_idx;
      float max_val = 0.0f;
      int max_rel_idx = 0;
      int rel_idx = 0;
      for (auto it = first; it != last; ++it, ++rel_idx) {
        float val = *it;
        if (!std::isinf(val) && val > max_val) {
          max_val = val;
          max_rel_idx = rel_idx;
        }
      } // max_rel_id from -pi/2 to pi/2
      size_t abs_idx = left_idx + max_rel_idx;
      direction_ = msg->angle_min +
                   abs_idx * msg->angle_increment; // direction for the robot

      RCLCPP_INFO(this->get_logger(), "Obstacle detected, new direction: %0.2f",
                  direction_);
    }
  }

  void control_loop() {
    twist_.linear.x = linear_vel_;
    twist_.angular.z = direction_ / 2;
    cmd_vel_publisher_->publish(twist_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  geometry_msgs::msg::Twist twist_;
  double direction_ = 0.0;
  const double min_detection_ = 0.35; // 35 cm to detect obstacle
  const float linear_vel_ = 0.1;      // linear velocity 0.1 m/sec
};

std::shared_ptr<Patrol> patrol_node;

void signal_handler(int signum) {
  if (patrol_node) {
    patrol_node->stop();
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  patrol_node = std::make_shared<Patrol>();
  std::signal(SIGINT, signal_handler);
  //   std::signal(SIGTERM, signal_handler); // System term (e.g., kill)
  rclcpp::spin(patrol_node);
  rclcpp::shutdown();
  return 0;
}