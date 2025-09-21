#include <exception>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  TestService() : Node("test_service_node") {

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TestService::laserscan_callback, this,
                  std::placeholders::_1));

    std::string name_service = "/direction_service";
    client_ = this->create_client<GetDirection>(name_service);

    // Wait for the service to be available (check every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Test Client started...");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<GetDirection>::SharedPtr client_;

  // subscriber (prefer UniquePtr to allow moving the big LaserScan)
  void laserscan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg) {
    // Create an empty GetDirection request
    auto request = std::make_shared<GetDirection::Request>();

    request->laser_data = std::move(*msg); // move msg, zero extra copy
    // Send the request asynchronously
    client_->async_send_request(
        request,
        [this](rclcpp::Client<GetDirection>::SharedFuture result_future) {
          try {
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Direction Service Response: %s ",
                        response->direction.c_str());
          } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Direction Service call failed: %s",
                         e.what());
          }
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto test_node = std::make_shared<TestService>();

  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}