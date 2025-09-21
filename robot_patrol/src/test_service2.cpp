#include <exception>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread>

using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  TestService() : Node("test_service_node") {
    sub_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = sub_group_;
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TestService::laserscan_callback_blocking, this,
                  std::placeholders::_1),
        opts);

    std::string name_service = "/direction_service";
    client_ = this->create_client<GetDirection>(
        name_service, rmw_qos_profile_services_default, client_group_);

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
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr sub_group_, client_group_;

  void laserscan_callback_blocking(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto req = std::make_shared<GetDirection::Request>();
    req->laser_data = *msg;

    auto fut = client_->async_send_request(req);
    using namespace std::chrono_literals;
    if (fut.wait_for(2s) == std::future_status::ready) {
      try {
        auto resp = fut.get();
        RCLCPP_INFO(get_logger(), "Direction: %s", resp->direction.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Service failed: %s", e.what());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Service timeout");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto test_node = std::make_shared<TestService>();
  // Use MultiThreadedExecutor with 2 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(test_node);

  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(test_node->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}