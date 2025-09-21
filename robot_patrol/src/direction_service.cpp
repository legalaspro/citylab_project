#include <cmath>
#include <cstddef>
#include <functional>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>

class GetDirectionService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  GetDirectionService() : Node("get_direction_service") {
    using namespace std::placeholders;
    // Create a service that will handle direction queries
    std::string name_service = "/direction_service";
    service_ = this->create_service<GetDirection>(
        name_service,
        std::bind(&GetDirectionService::get_direction_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr service_;
  const double min_detection_ = 0.35; // 35 cm to detect obstacle

  void
  get_direction_callback(const std::shared_ptr<GetDirection::Request> request,
                         std::shared_ptr<GetDirection::Response> response) {

    const auto &scan = request->laser_data;
    const size_t N = scan.ranges.size();

    if (N == 0 || scan.angle_increment == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Empty laser data recieved!");
      response->direction = "ERROR";
      return;
    }

    const size_t front_idx = std::round((scan.angle_max - scan.angle_min) /
                                        (2 * scan.angle_increment));

    if (scan.ranges[front_idx] > min_detection_) {
      RCLCPP_DEBUG(this->get_logger(), "Send forward");
      response->direction = "forward";
    }

    // Indices covering [-pi/2, pi/2]
    const size_t right_idx =
        std::round((-M_PI_2 - scan.angle_min) / scan.angle_increment);
    const size_t left_idx =
        std::round((M_PI_2 - scan.angle_min) / scan.angle_increment);
    // Split in 3 sections
    size_t total = left_idx - right_idx;
    size_t section = total / 3;

    auto right_begin = scan.ranges.begin() + right_idx; // start right
    auto right_end = right_begin + section;             // end of right
    auto front_end = right_end + section;               // end of front
    auto left_end = scan.ranges.begin() + left_idx + 1; // left

    float total_dist_sec_right =
        std::accumulate(right_begin, right_end, 0.0f); // right section
    float total_dist_sec_front =
        std::accumulate(right_end, front_end, 0.0f); // front section
    float total_dist_sec_left =
        std::accumulate(front_end, left_end, 0.0f); // left section

    if (total_dist_sec_front >= total_dist_sec_right &&
        total_dist_sec_front >= total_dist_sec_left) {
      RCLCPP_DEBUG(this->get_logger(), "Send  forward");
      response->direction = "forward";
    } else if (total_dist_sec_left >= total_dist_sec_right) {
      RCLCPP_DEBUG(this->get_logger(), "Send  left");
      response->direction = "left";
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Send  right");
      response->direction = "right";
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto service_node = std::make_shared<GetDirectionService>();

  rclcpp::spin(service_node);
  rclcpp::shutdown();
  return 0;
}