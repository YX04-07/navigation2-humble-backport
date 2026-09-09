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

#ifndef NAV2_CONTROLLER__PLUGINS__STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STOPPED_GOAL_CHECKER_HPP_

/**
 * @file stopped_goal_checker.hpp
 * @brief 在位姿到达条件之外，额外要求机器人速度接近停止。
 */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"

namespace nav2_controller
{

/**
 * @class StoppedGoalChecker
 * @brief Goal Checker plugin that checks the position difference and velocity
 * 中文：继承 SimpleGoalChecker，并增加线速度和角速度停止阈值。
 */
class StoppedGoalChecker : public SimpleGoalChecker
{
public:
  /**
   * @brief Construct a new Stopped Goal Checker object
    * 中文：构造停车目标检查器。
  * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  StoppedGoalChecker();

  /**
   * @brief Destroy the Stopped Goal Checker object
    * 中文：注销本类注册的动态参数回调。
  * 调用方：ControllerServer 清空 goal_checkers_、销毁插件实例时调用。
   */
  ~StoppedGoalChecker();

  /**
   * @brief Initialize the goal checker
    * 中文：初始化父类位姿容差，并读取平移和旋转停止速度阈值。
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
   * @brief Check if the goal is reached
    * 中文：父类位姿条件通过后，再要求当前线速度和角速度低于阈值。
  * 调用方：ControllerServer::isGoalReached() 通过 GoalChecker 基类接口动态分派调用。
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
    * 中文：复用父类位姿容差，并补充本类的停止速度容差。
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
    * 中文：直接复用父类的 XY 到达判定，不在此阶段检查速度。
  * 调用方：当前 nav2_controller 生产代码没有直接调用；供外部 GoalChecker 接口使用。
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
  rclcpp::Logger logger_{rclcpp::get_logger("stopped_goal_checker")};
  double rot_stopped_velocity_, trans_stopped_velocity_;
  // Dynamic parameters handler
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;
  std::string plugin_name_;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
  * 中文：拒绝负数的停车速度阈值。
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
  * 中文：线程安全地更新平移和旋转停止速度阈值。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__STOPPED_GOAL_CHECKER_HPP_
