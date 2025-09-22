#include <chrono>
#include <cmath>
#include <memory>
// #include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <thread>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_patrol/action/go_to_pose.hpp>

class GoToPoseServer : public rclcpp::Node {
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

  explicit GoToPoseServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_server", options) {
    using namespace std::placeholders;

    // Action Server to accept goals
    action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "go_to_pose",
        std::bind(&GoToPoseServer::handle_goal, this, _1, _2),
        std::bind(&GoToPoseServer::handle_cancel, this, _1),
        std::bind(&GoToPoseServer::handle_accepted, this, _1));

    // Subscribe to odometry to get the robot position
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPoseServer::odom_callback, this, _1));

    // Create cmd_vel publisher
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "GoToPose Action Server Ready...");
  }

  void stop() {
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing stop message...");
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // Calculate the theta(yaw- orientation around the z-axis)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_pos_.theta = tf2::getYaw(
        q); // directly get yaw without matrix construction in radians
    // tf2::Matrix3x3 m(q);
    // double roll, pitch;
    // m.getRPY(roll, pitch, current_pos_.theta);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {
    (void)uuid;
    desired_pos_ = goal->goal_pos;
    const double degree = desired_pos_.theta;
    desired_pos_.theta = degree * (M_PI / 180);
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x: %.2f, y: %.2f, theta: %.2f "
                "(degree) %.2f (radians)",
                desired_pos_.x, desired_pos_.y, degree, desired_pos_.theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up
    // a new thread
    std::thread{std::bind(&GoToPoseServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    rclcpp::WallRate rate(10.0); // 10 Hz control

    // Control Loop 10 Hz here
    geometry_msgs::msg::Twist twist;
    bool reached_dist = false;
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "Goal canceled by client");
        result->status = false;
        goal_handle->canceled(result);
        return;
      }

      // Calculate the desired angle
      double dx = desired_pos_.x - current_pos_.x;
      double dy = desired_pos_.y - current_pos_.y;
      if (!reached_dist) {
        double distance = std::hypot(dx, dy);
        if (distance > 0.01) {
          double heading = std::atan2(dy, dx);
          double heading_err = normalize_angle(heading - current_pos_.theta);
          twist.linear.x = std::min(0.2, distance); // apply fixed speed 0.2
          twist.angular.z = std::min(heading_err, 0.5);
          cmd_vel_publisher_->publish(twist);
        } else {
          reached_dist = true;
        }
      }

      // When reached destination do rotations for the desired pose
      if (reached_dist) {
        double dtheta =
            normalize_angle(desired_pos_.theta - current_pos_.theta);
        if (abs(dtheta) > 0.01) {
          twist.linear.x = 0.0;
          twist.angular.z = std::min(dtheta, 0.5);
          cmd_vel_publisher_->publish(twist);
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
          stop(); // stop the robot
          result->status = true;
          goal_handle->succeed(result);
          return;
        }
      }

      // feedback - current position
      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stop();
    result->status = false;
    goal_handle->abort(result);
  }

  double normalize_angle(double a) {
    const double tau = 2.0 * M_PI;
    double mod = std::fmod(a, tau);
    if (mod < 0.0) {
      mod += tau;
    }
    if (mod > M_PI) {
      mod -= tau;
    }
    return mod;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}