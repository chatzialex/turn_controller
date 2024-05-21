#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>

using Pose = geometry_msgs::msg::Pose;
using PoseArray = geometry_msgs::msg::PoseArray;
using Quaternion = geometry_msgs::msg::Quaternion;
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

namespace TurnController {

double getYaw(const Quaternion &quaternion) {
  tf2::Quaternion q{quaternion.x, quaternion.y, quaternion.z, quaternion.w};
  double pitch{};
  double roll{};
  double yaw{};
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

class TurnController : public rclcpp::Node {
public:
  TurnController(const std::string &node_name = kNodeName,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node{node_name, options}, odom_sub_{this->create_subscription<Odometry>(
                                      kOdometryTopicName, 1,
                                      std::bind(&TurnController::odomSubCb,
                                                this, std::placeholders::_1))},
        twist_pub_{
            this->create_publisher<Twist>(kCommandVelocityTopicName, 1)} {

    RCLCPP_INFO(this->get_logger(), "%s node started.", node_name.c_str());
  }

  bool followTrajectory(const PoseArray &goals);

private:
  struct State {
    double theta;
    double x;
    double y;
    double dtheta;
  };

  constexpr static char kCommandVelocityTopicName[]{"cmd_vel"};
  constexpr static char kNodeName[]{"distance_controller"};
  constexpr static char kOdometryTopicName[]{"/odometry/filtered"};

  constexpr static double kAngleTolerance{0.02}; // [rad]
  constexpr static auto kControlCycle{100ms};
  constexpr static double kIGain{0.01};
  constexpr static double kPGain{2.0};
  constexpr static double kDGain{0.3};

  void odomSubCb(const std::shared_ptr<const Odometry> msg);
  bool goToPoint(const Pose &goal);

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_{};
  rclcpp::Publisher<Twist>::SharedPtr twist_pub_{};

  std::optional<struct State> state_cur_{std::nullopt};
};

void TurnController::odomSubCb(const std::shared_ptr<const Odometry> msg) {
  state_cur_ = {getYaw(msg->pose.pose.orientation), msg->pose.pose.position.x,
                msg->pose.pose.position.y, msg->twist.twist.angular.z};
}

bool TurnController::goToPoint(const Pose &goal) {
  const auto pid_step{[this](auto e, auto de, auto ie) {
    return kPGain * e + kDGain * de + kIGain * ie;
  }};

  while (!state_cur_) {
    RCLCPP_WARN(this->get_logger(), "Odometry not received yet, waiting...");
    rclcpp::sleep_for(1s);
  }

  const auto theta_goal{std::atan2(goal.position.y - state_cur_.value().y,
                                   goal.position.x - state_cur_.value().x)};

  RCLCPP_INFO(this->get_logger(), "Received new goal: theta=%f [rad]",
              theta_goal);

  double e_theta{}, de_theta, ie_theta{0.0};
  Twist v_d{};
  while (true) {
    e_theta = std::atan2(std::sin(theta_goal - state_cur_.value().theta),
                         std::cos(theta_goal - state_cur_.value().theta));
    de_theta = /* dtheta_goal = 0 */ -state_cur_.value().dtheta;

    if (std::abs(e_theta) <= kAngleTolerance) {
      twist_pub_->publish(Twist{});
      RCLCPP_INFO(this->get_logger(), "Reached waypoint.");
      return true;
    }

    v_d.angular.z = pid_step(e_theta, de_theta, ie_theta);
    ie_theta += std::chrono::duration<double>{kControlCycle}.count() * e_theta;
    twist_pub_->publish(v_d);

    rclcpp::sleep_for(kControlCycle);
  }
}

bool TurnController::followTrajectory(const PoseArray &goals) {
  RCLCPP_INFO(this->get_logger(), "Received new goal trajectory.");

  for (const auto &goal : goals.poses) {
    if (!goToPoint(goal)) {
      RCLCPP_WARN(this->get_logger(), "Failed to follow goal trajectory.");
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Reached goal.");
  return true;
}

} // namespace TurnController

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node{std::make_shared<TurnController::TurnController>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto executor_thread{std::thread([&executor]() { executor.spin(); })};

  const auto goals{[] {
    PoseArray goal_poses;
    Pose goal_pose{};

    // w3
    goal_pose.position.x = 0.574;
    goal_pose.position.y = -1.454;
    goal_poses.poses.push_back(goal_pose);

    // w7
    goal_pose.position.x = 1.767;
    goal_pose.position.y = -0.432;
    goal_poses.poses.push_back(goal_pose);

    goal_pose.position.x = 0.698;
    goal_pose.position.y = 0.548;
    goal_poses.poses.push_back(goal_pose);

    return goal_poses;
  }()};
  node->followTrajectory(goals);

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
}