// Copyright (c) 2025 Prabhav Saxena
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

#ifndef NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_

/**
 * @file position_goal_checker.hpp
 * @brief 只使用 XY 位置和剩余路径长度判断是否到达目标，忽略目标朝向。
 */

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_controller
{

/**
 * @class PositionGoalChecker
 * @brief Goal Checker plugin that only checks XY position, ignoring orientation
 * 中文：适合终点朝向不重要，或机器人无需在终点原地调整方向的场景。
 */
class PositionGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * @brief Construct a new Position Goal Checker object
    * 中文：构造位置目标检查器。
    * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  PositionGoalChecker();

  /**
   * @brief Destroy the Position Goal Checker object
    * 中文：注销动态参数回调。
    * 调用方：ControllerServer 清空 goal_checkers_、销毁插件实例时调用。
   */
  ~PositionGoalChecker();

  /**
   * @brief Initialize the goal checker
    * 中文：读取 XY 容差、滞回缓冲、路径长度阈值和状态锁存参数。
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
    * 中文：清除“位置已到达”状态，供新路径重新判断。
    * 调用方：ControllerServer::setPlannerPath() 通过 GoalChecker 基类接口调用。
   */
  void reset() override;

  /**
   * @brief Check if the goal is reached
    * 中文：直接调用 XY 到达判定，不检查目标 yaw。
    * 调用方：ControllerServer::isGoalReached() 通过 GoalChecker 基类接口调用。
   * @param query_pose Current pose of the robot
   * @param goal_pose Target goal pose
   * @param velocity Current velocity of the robot
   * @param transformed_global_plan The transformed global plan
   * @return true if goal is reached, false otherwise
   */
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity,
    const nav_msgs::msg::Path & transformed_global_plan) override;

  /**
   * @brief Get the position and velocity tolerances
    * 中文：报告 XY 和路径长度容差；速度与朝向不构成到达约束。
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

  /**
   * @brief Check if XY goal position has been reached (without considering yaw)
    * 中文：检查剩余路径长度和 XY 距离，并处理锁存与滞回复位。
    * 调用方：PositionGoalChecker::isGoalReached()。
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

protected:
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("position_goal_checker")};
  double xy_goal_tolerance_, xy_goal_tolerance_buffer_;
  double xy_goal_tolerance_sq_, xy_goal_tolerance_reset_sq_;
  double path_length_tolerance_;
  bool stateful_;
  bool position_reached_;
  std::string plugin_name_;
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
  * 中文：拒绝本插件命名空间内的负数参数。
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
  * 中文：线程安全地更新位置、路径长度及状态锁存配置。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
