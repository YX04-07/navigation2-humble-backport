/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_

/**
 * @file simple_goal_checker.hpp
 * @brief 使用位置、朝向和剩余路径长度判断机器人是否到达目标。
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
 * @class SimpleGoalChecker
 * @brief Goal Checker plugin that only checks the position difference
 *
 * This class can be stateful if the stateful parameter is set to true (which it is by default).
 * This means that the goal checker will not check if the xy position matches again once it is found to be true.
 * 中文：支持有状态位置锁存、滞回缓冲以及前后方向对称的航向容差。
 */
class SimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * @brief Construct a new Simple Goal Checker object
    * 中文：构造简单目标检查器。
  * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  SimpleGoalChecker();

  /**
   * @brief Destroy the Simple Goal Checker object
    * 中文：注销动态参数回调。
  * 调用方：ControllerServer 清空 goal_checkers_、销毁插件实例时调用。
   */
  ~SimpleGoalChecker();

  /**
   * @brief Initialize the goal checker
    * 中文：读取位置、航向、路径长度、状态锁存和对称朝向等参数。
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
    * 中文：清除 XY 已到达的锁存状态，供新路径重新判断。
  * 调用方：ControllerServer::setPlannerPath() 通过 GoalChecker 基类接口调用。
   */
  void reset() override;

  /**
   * @brief Check if the goal is reached
    * 中文：先检查 XY 和剩余路径长度，再检查普通或对称航向误差。
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
    * 中文：向控制器报告本检查器使用的位置、航向和路径长度容差。
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
    * 中文：检查剩余路径长度和 XY 距离，并处理 stateful 锁存及滞回复位。
  * 调用方：SimpleGoalChecker::isGoalReached()；也可由派生类或外部接口调用。
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
  rclcpp::Logger logger_{rclcpp::get_logger("simple_goal_checker")};
  double xy_goal_tolerance_, xy_goal_tolerance_buffer_, yaw_goal_tolerance_, path_length_tolerance_;
  bool stateful_, check_xy_;
  bool symmetric_yaw_tolerance_;
  // Cached squared xy_goal_tolerance_ and xy_goal_tolerance_reset_
  double xy_goal_tolerance_sq_, xy_goal_tolerance_reset_sq_;
  // Dynamic parameters handler
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;
  std::string plugin_name_;

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
  * 中文：线程安全地更新各容差、状态锁存和对称朝向配置。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
