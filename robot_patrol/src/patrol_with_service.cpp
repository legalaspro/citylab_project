#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  Patrol() : Node("patrol_node") {
    // Subscribers
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::laserscan_callback, this, _1));

    // Publishers
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Services
    std::string name_service = "/direction_service";
    direction_client_ = this->create_client<GetDirection>(name_service);

    // Wait for the service to be available (check every second)
    while (!direction_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }

    // Timer for control loop  (10Hz)
    control_timer_ =
        this->create_wall_timer(100ms, std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol Node initialized and ready!");
  }

  void stop() {
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Client<GetDirection>::SharedPtr direction_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  geometry_msgs::msg::Twist twist_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
  const double min_detection_ = 0.55; // 55 cm to detect obstacle
  const float linear_vel_ = 0.1;      // linear velocity 0.1 m/sec
  const float angular_vel_ = 0.5;     // angular velocity 0.5 rad/sec

  //  use UniquePtr to allow moving the big LaserScan
  void laserscan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg) {
    // move the whole message into our shared buffer (no copy)
    scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>(std::move(*msg));
  }

  void control_loop() {
    if (!scan_msg_)
      return;

    // no obstacle detected
    if (min_front(*scan_msg_) > min_detection_) {
      twist_.linear.x = linear_vel_;
      twist_.angular.z = 0.0;
      cmd_vel_publisher_->publish(twist_);
      RCLCPP_INFO(this->get_logger(), "No obstacles, moving forward");
      return;
    }

    // Create an empty GetDirection request
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = std::move(*scan_msg_); // move message with zero copy
    scan_msg_.reset();                           // moved from, drop it

    RCLCPP_INFO(this->get_logger(), "Obstacle determined, calling service...");
    // Send the request asynchronously
    direction_client_->async_send_request(
        request,
        [this](rclcpp::Client<GetDirection>::SharedFuture result_future) {
          try {
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Direction Service Response: %s ",
                        response->direction.c_str());
            if (response->direction == "forward") {
              twist_.linear.x = linear_vel_;
              twist_.angular.z = 0.0;
            } else if (response->direction == "left") {
              twist_.linear.x = linear_vel_;
              twist_.angular.z = angular_vel_;
            } else {
              twist_.linear.x = linear_vel_;
              twist_.angular.z = -angular_vel_;
            }
            cmd_vel_publisher_->publish(twist_);
          } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Direction Service call failed: %s",
                         e.what());
          }
        });
  }

  float min_front(const sensor_msgs::msg::LaserScan &scan,
                  double half_deg = 10.0) {
    const double half = half_deg * M_PI / 180.0;
    const size_t right_idx =
        std::ceil((-half - scan.angle_min) / scan.angle_increment);
    const size_t left_idx =
        std::floor((half - scan.angle_min) / scan.angle_increment) + 1;
    float min_val = std::numeric_limits<float>::infinity();
    for (auto i = right_idx; i < left_idx; ++i) {
      min_val = std::min(min_val, scan.ranges[i]);
    }
    return min_val;
  }
};

std::shared_ptr<Patrol> patrol_node;

void signal_handler(int signum) {
  (void)signum;
  if (patrol_node) {
    patrol_node->stop();
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  patrol_node = std::make_shared<Patrol>();
  std::signal(SIGINT, signal_handler);
  rclcpp::spin(patrol_node);
  rclcpp::shutdown();
  return 0;
}