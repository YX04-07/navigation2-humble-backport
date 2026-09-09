// Copyright (c) 2023 Dexory
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

#ifndef NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_

/**
 * @file pose_progress_checker.hpp
 * @brief 同时使用平移距离和旋转角度判断机器人是否取得进展。
 */

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"

namespace nav2_controller
{
/**
* @class PoseProgressChecker
* @brief This plugin is used to check the position and the angle of the robot to make sure
* that it is actually progressing or rotating towards a goal.
* 中文：机器人只要平移或旋转达到阈值，就被认为取得了有效进展。
*/

class PoseProgressChecker : public SimpleProgressChecker
{
public:
  /**
   * @brief Construct a new Pose Progress Checker object
    * 中文：构造位姿进度检查器。
  * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  PoseProgressChecker() = default;

  /**
   * @brief Destroy the Pose Progress Checker object
    * 中文：注销派生类注册的动态参数回调。
  * 调用方：ControllerServer 清空 progress_checkers_、销毁插件实例时调用。
   */
  ~PoseProgressChecker();

  /**
   * @brief Initialize the goal checker
    * 中文：先初始化距离检查参数，再读取最小旋转角并注册回调。
  * 调用方：ControllerServer::on_activate() 通过 ProgressChecker 基类接口调用。
   * @param parent Weak pointer to the lifecycle node
   * @param plugin_name Name of the plugin
   */
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  /**
   * @brief Checks if the robot has moved compare to previous
    * 中文：平移或旋转任一达到要求时刷新基准，否则检查是否超时。
  * 调用方：ControllerServer::computeAndPublishVelocity() 通过基类接口动态分派调用。
   * @param current_pose Current pose of the robot
   * @return true, if the robot has moved enough, false otherwise
   */
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
    * 中文：组合父类的平移判定和本类的航向角变化判定。
  * 调用方：PoseProgressChecker::check()。
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Calculates angle difference between two poses
    * 中文：计算两个姿态 yaw 的最短角距离绝对值。
  * 调用方：PoseProgressChecker::isRobotMovedEnough()。
   * @param pose1 First pose
   * @param pose2 Second pose
   * @return Angle difference in radians
   */
  static double poseAngleDistance(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &);

  double required_movement_angle_;

  // Dynamic parameters handler
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;
  std::string plugin_name_;
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("pose_progress_checker")};

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
  * 中文：线程安全地更新最小旋转角阈值。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
