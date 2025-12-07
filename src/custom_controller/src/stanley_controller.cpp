#include "custom_controller/stanley_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "angles/angles.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace custom_controller
{

using nav2_core::GoalChecker;

void StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in configure()");
  }
  logger_ = node->get_logger();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  // Load parameters with sensible defaults.
  node->declare_parameter(name + ".k_gain", k_gain_);
  node->declare_parameter(name + ".softening_speed", softening_speed_);
  node->declare_parameter(name + ".max_steer_rad", max_steer_rad_);
  node->declare_parameter(name + ".max_angular_speed", max_angular_speed_);
  node->declare_parameter(name + ".min_linear_speed", min_linear_speed_);
  node->declare_parameter(name + ".max_linear_speed", max_linear_speed_);
  node->declare_parameter(name + ".slow_down_gain", slow_down_gain_);
  node->declare_parameter(name + ".lookahead_dist", lookahead_dist_);
  node->declare_parameter(name + ".goal_tolerance_lin", goal_tolerance_lin_);
  node->declare_parameter(name + ".goal_tolerance_yaw", goal_tolerance_yaw_);

  node->get_parameter(name + ".k_gain", k_gain_);
  node->get_parameter(name + ".softening_speed", softening_speed_);
  node->get_parameter(name + ".max_steer_rad", max_steer_rad_);
  node->get_parameter(name + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(name + ".min_linear_speed", min_linear_speed_);
  node->get_parameter(name + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(name + ".slow_down_gain", slow_down_gain_);
  node->get_parameter(name + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name + ".goal_tolerance_lin", goal_tolerance_lin_);
  node->get_parameter(name + ".goal_tolerance_yaw", goal_tolerance_yaw_);

  RCLCPP_INFO(logger_, "StanleyController configured with k=%.2f, max_lin=%.2f, max_ang=%.2f",
              k_gain_, max_linear_speed_, max_angular_speed_);
}

void StanleyController::cleanup()
{
  global_plan_.poses.clear();
}

void StanleyController::activate() {}

void StanleyController::deactivate() {}

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const Pose & pose, const geometry_msgs::msg::Twist & velocity, GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("Global plan is empty");
  }

  const auto & goal_pose = global_plan_.poses.back();
  const double dx_goal = goal_pose.pose.position.x - pose.pose.position.x;
  const double dy_goal = goal_pose.pose.position.y - pose.pose.position.y;
  const double dist_goal = std::hypot(dx_goal, dy_goal);
  const double yaw_goal = computeYaw(goal_pose.pose.orientation);
  const double yaw_robot = computeYaw(pose.pose.orientation);
  const double yaw_err = normalizeAngle(yaw_goal - yaw_robot);

  if (dist_goal < goal_tolerance_lin_ && std::abs(yaw_err) < goal_tolerance_yaw_) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = robot_base_frame_;
    cmd.header.stamp = pose.header.stamp;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // Transform plan to global_frame_ if needed (assumes pose in same frame).
  nav_msgs::msg::Path transformed_plan;
  if (!transformGlobalPlan(pose, transformed_plan)) {
    throw std::runtime_error("Failed to transform global plan");
  }

  geometry_msgs::msg::PoseStamped target;
  size_t target_idx = 0;
  if (!findTargetPoint(transformed_plan, pose, lookahead_dist_, target, target_idx)) {
    throw std::runtime_error("Failed to find target point on path");
  }

  const double path_heading = headingToTarget(
    (target_idx > 0) ? transformed_plan.poses[target_idx - 1] : pose, target);

  // Cross-track error signed relative to path heading.
  const double dx = target.pose.position.x - pose.pose.position.x;
  const double dy = target.pose.position.y - pose.pose.position.y;
  const double cross_track = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;

  const double heading_err = normalizeAngle(path_heading - yaw_robot);
  const double v = std::max(std::abs(velocity.linear.x), softening_speed_);
  double steer = heading_err + std::atan2(k_gain_ * cross_track, v);

  // Clamp steering to physical limits.
  steer = std::clamp(steer, -max_steer_rad_, max_steer_rad_);
  double angular_z = std::clamp(steer, -max_angular_speed_, max_angular_speed_);

  double linear_x = max_linear_speed_ * std::exp(-slow_down_gain_ * std::abs(steer));
  linear_x = std::clamp(linear_x, min_linear_speed_, max_linear_speed_);

  // If very close to goal, slow further.
  if (dist_goal < lookahead_dist_) {
    linear_x = std::max(min_linear_speed_, linear_x * 0.5);
    angular_z = std::clamp(angular_z * 1.2, -max_angular_speed_, max_angular_speed_);
  }

  // Apply external speed limit if setSpeedLimit was called.
  if (speed_limit_linear_ > 0.0) {
    double limit = speed_limit_linear_;
    if (speed_limit_is_ratio_) {
      limit = max_linear_speed_ * speed_limit_linear_;
    }
    linear_x = std::min(linear_x, limit);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = robot_base_frame_;
  cmd.header.stamp = pose.header.stamp;
  cmd.twist.linear.x = linear_x;
  cmd.twist.angular.z = angular_z;
  return cmd;
}

void StanleyController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
}

void StanleyController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_linear_ = speed_limit;
  speed_limit_is_ratio_ = percentage;
}

bool StanleyController::transformGlobalPlan(const Pose & pose, nav_msgs::msg::Path & transformed)
{
  // In most Nav2 setups, global plan and pose are already in the global frame. We keep it simple.
  (void)pose;
  transformed = global_plan_;
  return true;
}

bool StanleyController::findTargetPoint(const nav_msgs::msg::Path & path, const Pose & robot_pose,
                                        double lookahead, geometry_msgs::msg::PoseStamped & target,
                                        size_t & idx) const
{
  if (path.poses.empty()) {
    return false;
  }

  const auto & poses = path.poses;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < poses.size(); ++i) {
    const double dx = poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = poses[i].pose.position.y - robot_pose.pose.position.y;
    const double d = std::hypot(dx, dy);
    if (d < min_dist) {
      min_dist = d;
      // nearest point seen so far (not used later, kept for clarity)
    }
    if (d >= lookahead) {
      target = poses[i];
      idx = i;
      return true;
    }
  }

  // If no point beyond lookahead, use the farthest (goal).
  target = poses.back();
  idx = poses.size() - 1;
  return true;
}

double StanleyController::headingToTarget(const Pose & from, const Pose & to) const
{
  const double dx = to.pose.position.x - from.pose.position.x;
  const double dy = to.pose.position.y - from.pose.position.y;
  return std::atan2(dy, dx);
}

double StanleyController::normalizeAngle(const double angle) const
{
  return angles::normalize_angle(angle);
}

double StanleyController::computeYaw(const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace custom_controller

// Export plugin
PLUGINLIB_EXPORT_CLASS(custom_controller::StanleyController, nav2_core::Controller)
