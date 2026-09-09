// Copyright (c) 2026, David Grbac
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CONTROLLER__PLUGINS__ADAPTIVE_TOLERANCE_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__ADAPTIVE_TOLERANCE_GOAL_CHECKER_HPP_

/**
 * @file adaptive_tolerance_goal_checker.hpp
 * @brief 使用精细/粗略两级容差和停滞状态判断机器人是否可接受地到达目标。
 */

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**
 * @class AdaptiveToleranceGoalChecker
 * @brief Goal Checker plugin with two tolerance tiers: a tight desired tolerance
 * and a looser coarse tolerance. The robot is considered to have reached the goal if:
 *   (1) it reaches within the desired (tight) tolerance, OR
 *   (2) it is within the coarse tolerance AND the robot's velocity is below
 *       a stopped threshold for a configurable number of consecutive cycles,
 *       indicating it is no longer making useful progress toward the goal.
 * 中文：优先要求精确到达；若只能进入粗略容差区，则在穿过终点线、停车停滞或距离
 * 不再改善时接受目标，从而避免机器人在狭窄或受限环境中长期振荡。
 */
class AdaptiveToleranceGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * @brief Construct a new Progress Goal Checker object
    * 中文：构造自适应容差目标检查器。
    * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  AdaptiveToleranceGoalChecker();
  /**
   * @brief Destroy the Progress Goal Checker object
    * 中文：注销动态参数回调。
    * 调用方：ControllerServer 清空 goal_checkers_、销毁插件实例时调用。
   */
  ~AdaptiveToleranceGoalChecker();

  /**
   * @brief Initialize the goal checker
    * 中文：读取两级位置容差、航向、停车阈值和连续停滞周期等参数。
    * 调用方：ControllerServer::on_activate() 通过 GoalChecker 基类接口调用。
   * @param parent Weak pointer to the lifecycle node
   * @param plugin_name Name of the plugin
   * @param costmap_ros Shared pointer to the costmap
   */
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Reset the goal checker state
    * 中文：清除容差区、最佳距离、停滞计数、接近方向和接受原因等状态。
    * 调用方：ControllerServer::setPlannerPath() 通过 GoalChecker 基类接口调用。
   */
  void reset() override;

  /**
   * @brief Check if the goal is reached
    * 中文：XY 被接受后，再检查普通或前后对称的航向误差。
    * 调用方：ControllerServer::isGoalReached() 通过 GoalChecker 基类接口调用。
   * @param query_pose Current pose of the robot
   * @param goal_pose Target goal pose
   * @param velocity Current velocity of the robot
   * @param transformed_global_plan The transformed global plan
   * @return true if goal is reached, false otherwise
   */
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity,
    const nav_msgs::msg::Path & transformed_global_plan) override;

  /**
   * @brief Check if XY goal position has been reached (without considering yaw)
    * 中文：精细容差内立即通过；粗略容差内根据终点线和停滞条件决定是否通过。
    * 调用方：AdaptiveToleranceGoalChecker::isGoalReached()。
   * @param query_pose The pose to check
   * @param goal_pose The pose to check against
   * @param velocity The robot's current velocity
   * @param transformed_global_plan The global plan after being processed by the path handler
   * @return True if XY goal is reached (position within tolerance, yaw ignored)
   */
  bool isGoalXYReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity,
    const nav_msgs::msg::Path & transformed_global_plan) override;

  /**
   * @brief Get the position and velocity tolerances
    * 中文：向控制器报告最宽的粗略位置容差、停车速度、航向和路径长度容差。
    * 调用方：当前 nav2_controller 生产代码没有直接调用；供外部通过 GoalChecker 接口查询。
   * @param pose_tolerance Output parameter for pose tolerance
   * @param vel_tolerance Output parameter for velocity tolerance
   * @param path_length_tolerance Output parameter for path length tolerance
   * @return true if tolerances are available, false otherwise
   */
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance,
    double & path_length_tolerance) override;

protected:
  /// @brief Reason that the XY component of the goal was accepted.
  enum class XyAcceptanceReason
  {
    NONE,
    FINE_TOLERANCE,
    COARSE_TOLERANCE_FINISH_LINE,
    COARSE_TOLERANCE_STOPPED_STAGNATION,
    COARSE_TOLERANCE_DISTANCE_STAGNATION
  };

  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("adaptive_tolerance_goal_checker")};

  // Fine (desired) tolerance
  double fine_xy_goal_tolerance_;
  double fine_xy_goal_tolerance_sq_;
  // Coarse (fallback) tolerance
  double coarse_xy_goal_tolerance_;
  double coarse_xy_goal_tolerance_sq_;
  // Hysteresis buffer used when stateful to reset the
  // check_xy_ when the robot drifts outside the accepted region.
  double xy_goal_tolerance_buffer_;
  double fine_xy_goal_tolerance_reset_sq_, coarse_xy_goal_tolerance_reset_sq_;

  double yaw_goal_tolerance_;
  double path_length_tolerance_;
  bool stateful_;
  bool symmetric_yaw_tolerance_;

  // Velocity thresholds for detecting a stopped/stalled robot
  double trans_stopped_velocity_;
  double rot_stopped_velocity_;

  // Number of consecutive stopped cycles before accepting at coarse tolerance
  int required_stagnation_cycles_;

  // Stateful tracking
  bool check_xy_;
  bool in_tolerance_zone_;
  int stopped_stagnation_count_;
  int distance_stagnation_count_;
  double best_distance_sq_;
  double approach_dx_;
  double approach_dy_;
  XyAcceptanceReason xy_acceptance_reason_;

  // Dynamic parameters
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;
  std::string plugin_name_;

  /**
    * @brief Convert XY acceptance reason to string
    * 中文：把 XY 接受原因转换为日志可读字符串。
    * 调用方：AdaptiveToleranceGoalChecker::isGoalReached()。
    * @param reason The XY acceptance reason
    * @return String representation of the XY acceptance reason
    */
  static std::string toString(XyAcceptanceReason reason);

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
  * 中文：拒绝负数容差，并要求连续停滞周期至少为 1。
    * 调用方：initialize() 注册后，由 rclcpp 参数框架在设置参数前间接调用。
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
  * 中文：线程安全地更新两级容差、停车阈值、停滞周期和状态配置。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__ADAPTIVE_TOLERANCE_GOAL_CHECKER_HPP_
