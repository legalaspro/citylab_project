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
  const double min_detection_ = 0.55; // 55 cm to detect obstacle

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

    if (min_front(scan, 25.0) > min_detection_) {
      RCLCPP_INFO(this->get_logger(), "Send forward");
      response->direction = "forward";
      return;
    }

    // Indices covering [-pi/2, pi/2]
    const size_t right_idx =
        std::ceil((-M_PI_2 - scan.angle_min) / scan.angle_increment);
    const size_t left_idx =
        std::floor((M_PI_2 - scan.angle_min) / scan.angle_increment) + 1;
    // Split in 3 sections
    size_t total = left_idx - right_idx;
    size_t section = total / 3;

    auto right_begin = scan.ranges.begin() + right_idx; // start right
    auto right_end = right_begin + section;             // end of right
    auto front_end = right_end + section;               // end of front
    auto left_end = scan.ranges.begin() + left_idx + 1; // left

    float total_dist_sec_right = sum_finite(right_begin, right_end);
    float total_dist_sec_front = sum_finite(right_end, front_end);
    float total_dist_sec_left = sum_finite(front_end, left_end);

    if (total_dist_sec_front >= total_dist_sec_right &&
        total_dist_sec_front >= total_dist_sec_left) {
      RCLCPP_INFO(this->get_logger(), "Send  forward");
      response->direction = "forward";
    } else if (total_dist_sec_left >= total_dist_sec_right) {
      RCLCPP_INFO(this->get_logger(), "Send  left");
      response->direction = "left";
    } else {
      RCLCPP_INFO(this->get_logger(), "Send  right");
      response->direction = "right";
    }
  }

  template <typename Iter> static float sum_finite(Iter first, Iter last) {
    float sum = 0.0f;
    for (; first != last; ++first) {
      const float v = *first;
      if (std::isfinite(v) && v > 0.0f)
        sum += v;
    }
    return sum;
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto service_node = std::make_shared<GetDirectionService>();

  rclcpp::spin(service_node);
  rclcpp::shutdown();
  return 0;
}