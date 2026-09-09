// Copyright (c) 2025 Dexory
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

#ifndef NAV2_CONTROLLER__PLUGINS__AXIS_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__AXIS_GOAL_CHECKER_HPP_

/**
 * @file axis_goal_checker.hpp
 * @brief 沿路径末段方向分别检查纵向误差和横向误差。
 */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**
  * @class AxisGoalChecker
  * @brief Goal Checker plugin that checks progress along the axis defined by the last segment
  * of the path to the goal.
  *
  * This class can be configured to allow overshoot past the goal if the is_overshoot_valid
  *  parameter is set to true (which is false by default).
  * 中文：以路径最后一个有效线段建立局部坐标轴，可配置为允许机器人越过终点。
  */
class AxisGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * @brief Construct a new Axis Goal Checker object
    * 中文：构造轴向目标检查器。
    * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  AxisGoalChecker();

  /**
   * @brief Destroy the Axis Goal Checker object
    * 中文：注销动态参数回调。
    * 调用方：ControllerServer 清空 goal_checkers_、销毁插件实例时调用。
   */
  ~AxisGoalChecker();

  // Standard GoalChecker Interface
  /**
   * @brief Initialize the goal checker
    * 中文：读取纵向、横向、剩余路径长度容差以及越过终点开关。
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
    * 中文：本插件没有跨周期状态，因此无需执行复位操作。
    * 调用方：ControllerServer::setPlannerPath() 通过 GoalChecker 基类接口调用。
   */
  void reset() override;

  /**
   * @brief Check if the goal is reached
    * 中文：忽略目标朝向，直接执行路径轴向的位置判定。
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
    * 中文：以接口可表达的保守形式报告轴向位置和路径长度容差。
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
    * 中文：把目标误差投影到路径末段的纵向和横向，分别与容差比较。
    * 调用方：AxisGoalChecker::isGoalReached()。
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
  double along_path_tolerance_;
  double cross_track_tolerance_;
  double path_length_tolerance_;
  bool is_overshoot_valid_;
  // Dynamic parameters handler
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;
  std::string plugin_name_;
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("AxisGoalChecker")};

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
  * 中文：拒绝负数的轴向和路径长度容差。
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
  * 中文：线程安全地更新容差和越过终点开关。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__AXIS_GOAL_CHECKER_HPP_
