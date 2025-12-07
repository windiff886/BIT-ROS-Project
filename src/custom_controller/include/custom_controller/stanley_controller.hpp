#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace custom_controller
{

class StanleyController : public nav2_core::Controller
{
public:
  StanleyController() = default;
  ~StanleyController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using Pose = geometry_msgs::msg::PoseStamped;

  bool transformGlobalPlan(const Pose & pose, nav_msgs::msg::Path & transformed);
  bool findTargetPoint(const nav_msgs::msg::Path & path, const Pose & robot_pose,
                       double lookahead, geometry_msgs::msg::PoseStamped & target, size_t & idx) const;
  double headingToTarget(const geometry_msgs::msg::PoseStamped & from,
                         const geometry_msgs::msg::PoseStamped & to) const;
  double normalizeAngle(const double angle) const;
  double computeYaw(const geometry_msgs::msg::Quaternion & q) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("StanleyController")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;
  std::string robot_base_frame_;

  nav_msgs::msg::Path global_plan_;
  std::mutex plan_mutex_;

  // Parameters
  double k_gain_{1.5};
  double softening_speed_{0.1};
  double max_steer_rad_{0.6};
  double max_angular_speed_{1.0};
  double min_linear_speed_{0.05};
  double max_linear_speed_{0.35};
  double slow_down_gain_{2.0};
  double lookahead_dist_{0.6};
  double goal_tolerance_lin_{0.05};
  double goal_tolerance_yaw_{0.08};
  double speed_limit_linear_{0.0};
  bool speed_limit_is_ratio_{false};
};

}  // namespace custom_controller
