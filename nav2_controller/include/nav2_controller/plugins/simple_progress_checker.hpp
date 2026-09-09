// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_

/**
 * @file simple_progress_checker.hpp
 * @brief 基于移动距离和时间窗口判断机器人是否持续取得进展。
 */

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"
#include "nav2_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav2_controller
{
/**
* @class SimpleProgressChecker
* @brief This plugin is used to check the position of the robot to make sure
* that it is actually progressing towards a goal.
* 中文：若机器人在允许时间内未离开基准位置指定半径，则认为导航卡住。
*/

class SimpleProgressChecker : public nav2_core::ProgressChecker
{
public:
  /**
   * @brief Construct a new Simple Progress Checker object
    * 中文：构造简单进度检查器，实际参数在 initialize() 中读取。
  * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  SimpleProgressChecker() = default;

  /**
   * @brief Destroy the Simple Progress Checker object
    * 中文：注销本插件注册的动态参数回调。
  * 调用方：ControllerServer 清空 progress_checkers_、销毁插件实例时调用。
   */
  ~SimpleProgressChecker();

  /**
   * @brief Initialize the goal checker
    * 中文：读取最小移动半径和允许停滞时间，并注册动态参数回调。
  * 调用方：ControllerServer::on_activate() 通过 ProgressChecker 基类接口调用。
   * @param parent Weak pointer to the lifecycle node
   * @param plugin_name Name of the plugin
   */
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  /**
   * @brief Checks if the robot has moved compare to previous
    * 中文：机器人移动足够远时刷新基准；否则检查是否仍处于允许时间内。
  * 调用方：ControllerServer::computeAndPublishVelocity() 通过基类接口调用。
   * @param current_pose Current pose of the robot
   * @return true, if the robot has moved enough, false otherwise
   */
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  /**
   * @brief Reset the progress checker state
    * 中文：清除基准位姿，使下一次 check() 重新建立基准。
  * 调用方：ControllerServer::computeControl() 和 updateGlobalPath() 通过基类接口调用。
   */
  void reset() override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
    * 中文：比较当前位姿与基准位姿的二维平移距离。
  * 调用方：SimpleProgressChecker::check()。
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose & pose);
  /**
   * @brief Resets baseline pose with the current pose of the robot
    * 中文：把当前位姿和当前时间保存为新的进度检查基准。
  * 调用方：SimpleProgressChecker::check()；派生类 PoseProgressChecker::check() 也会调用。
   * @param pose Current pose of the robot
   */
  void resetBaselinePose(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Calculates distance between two poses
    * 中文：计算两个位姿在 XY 平面上的欧氏距离。
  * 调用方：SimpleProgressChecker::isRobotMovedEnough()；派生类 PoseProgressChecker 也会调用。
   * @param pose1 First pose
   * @param pose2 Second pose
   * @return Distance between the two poses
   */
  static double pose_distance(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &);

  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("simple_progress_checker")};

  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose baseline_pose_;
  rclcpp::Time baseline_time_;

  bool baseline_pose_set_{false};
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
  * 中文：线程安全地更新最小移动半径和允许停滞时间。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
