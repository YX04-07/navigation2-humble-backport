// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_

/**
 * @file parameter_handler.hpp
 * @brief Controller Server 参数结构和动态参数处理器的声明。
 *
 * 参数分为服务器级参数和插件级参数。服务器级参数由 ParameterHandler 管理；名称中
 * 带点号的插件私有参数（例如 goal_checker.xy_goal_tolerance）由对应插件自行管理。
 */

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_controller
{

/**
 * @brief Controller Server 运行参数及四类插件的“逻辑 ID -> C++ 类型”映射。
 *
 * *_ids 是配置和 Action 中使用的实例名称，*_types 是 pluginlib 创建实例时使用的
 * 完整 C++ 类型名，两者按相同下标一一对应。
 */
struct Parameters
{
  double controller_frequency;
  double min_x_velocity_threshold;
  double min_y_velocity_threshold;
  double min_theta_velocity_threshold;
  std::string speed_limit_topic;
  double failure_tolerance;
  bool use_realtime_priority;
  bool publish_zero_velocity;
  rclcpp::Duration costmap_update_timeout{0, 0};
  std::string odom_topic;
  double odom_duration;
  double search_window;
  std::vector<std::string> progress_checker_ids;
  std::vector<std::string> progress_checker_types;
  std::vector<std::string> goal_checker_ids;
  std::vector<std::string> goal_checker_types;
  std::vector<std::string> controller_ids;
  std::vector<std::string> controller_types;
  std::vector<std::string> path_handler_ids;
  std::vector<std::string> path_handler_types;
};

/**
 * @class nav2_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for Controller Server
 * 中文：负责声明、读取、校验并更新 Controller Server 参数，同时解析插件配置。
 */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for nav2_controller::ParameterHandler
    * 中文：加载服务器参数和四类插件列表，并解析每个插件 ID 对应的 pluginlib 类型。
    * 调用方：ControllerServer::on_configure() 通过 make_unique 创建。
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    const rclcpp::Logger & logger);

protected:
  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
  * 中文：插件私有参数交给插件校验；服务器级 double 参数通常必须非负，
  * failure_tolerance 例外，因为 -1 表示无限等待。
  * 调用方：ParameterHandler 激活后，由 rclcpp 参数框架在设置参数前间接调用。
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
  * 中文：仅更新允许运行时修改的服务器参数；若控制循环正持有互斥锁则拒绝本次更新。
  * 调用方：ParameterHandler 激活后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  const std::vector<std::string> default_progress_checker_ids_{"progress_checker"};
  const std::vector<std::string> default_progress_checker_types_{
    "nav2_controller::SimpleProgressChecker"};
  const std::vector<std::string> default_goal_checker_ids_{"goal_checker"};
  const std::vector<std::string> default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"};
  const std::vector<std::string> default_controller_ids_{"FollowPath"};
  const std::vector<std::string> default_controller_types_{"nav2_mppi_controller::MPPIController"};
  const std::vector<std::string> default_path_handler_ids_{"PathHandler"};
  const std::vector<std::string> default_path_handler_types_{
    "nav2_controller::FeasiblePathHandler"};
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
