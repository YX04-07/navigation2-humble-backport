// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_CONTROLLER__PLUGINS__FEASIBLE_PATH_HANDLER_HPP_
#define NAV2_CONTROLLER__PLUGINS__FEASIBLE_PATH_HANDLER_HPP_

/**
 * @file feasible_path_handler.hpp
 * @brief 从全局路径中提取机器人当前可执行的局部路径段，并完成坐标变换。
 */

#include <string>
#include <vector>
#include <memory>
#include <utility>
#include "nav2_core/path_handler.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/parameter_callbacks.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace nav2_controller
{
/**
* @class FeasiblePathHandler
* @brief This plugin manages the global plan by clipping it to the local
* segment, typically bounded by the local costmap size
* and transforming the resulting path into the odom frame.
* 中文：维护完整路径和当前约束段，裁剪已通过部分，并限制输出路径不超出局部代价地图。
*/

class FeasiblePathHandler : public nav2_core::PathHandler
{
public:
  /**
   * @brief Construct a new Feasible Path Handler object
    * 中文：构造可行路径处理器。
    * 调用方：pluginlib 在 ControllerServer::on_configure() 创建该插件实例时调用。
   */
  FeasiblePathHandler() = default;

  /**
   * @brief Destroy the Feasible Path Handler object
    * 中文：注销动态参数回调。
    * 调用方：ControllerServer 清空 path_handlers_、销毁插件实例时调用。
   */
  ~FeasiblePathHandler();

  /**
   * @brief Initialize parameters
    * 中文：保存节点、TF 和代价地图，并读取路径搜索、裁剪和约束参数。
    * 调用方：ControllerServer::on_activate() 通过 PathHandler 基类接口调用。
   * @param parent Lifecycle node pointer
   * @param logger Node logging interface
   * @param plugin_name Name of the plugin
   * @param costmap_ros Costmap2DROS object
   * @param tf Shared ptr of TF2 buffer
   */
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const rclcpp::Logger & logger,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    nav2::TransformBuffer::SharedPtr tf) override;

  /**
   * @brief Set new reference plan
    * 中文：保存完整路径，并按倒车点或大角度旋转点确定当前可执行段。
    * 调用方：ControllerServer::setPlannerPath() 通过 PathHandler 基类接口调用。
   * @param Path Path to use
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Determines the portion of the global plan to be used for local control.
   * This function locates the start and end iterators of the global plan segment
   * that is relevant for controller computation based on the robot's current pose and local costmap.
  * 中文：在限定搜索距离内寻找机器人最近路径点，再按裁剪距离确定局部段终点。
    * 调用方：ControllerServer::computeAndPublishVelocity() 通过 PathHandler 基类接口调用。
   * @param pose Robot pose in odom frame
   * @return PathSegment A pair of iterators defining the start and end of the
   *         selected plan segment.
   */
  nav2_core::PathSegment findPlanSegment(
    const geometry_msgs::msg::PoseStamped & pose) override;

  /**
    * @brief Transforms a predefined segment of the global plan into the costmap global frame.
    * 中文：将选中路径段变换到代价地图坐标系，越界时截断，并剪除已经通过的路径。
    * 调用方：ControllerServer::computeAndPublishVelocity() 通过 PathHandler 基类接口调用。
    * @param closest_point Iterator to the starting pose of the path segment.
    * @param pruned_plan_end Iterator to the ending pose of the path segment.
    * @return nav_msgs::msg::Path The transformed local plan segment in the costmap global frame.
    */
  nav_msgs::msg::Path transformLocalPlan(
    const nav2_core::PathIterator & closest_point,
    const nav2_core::PathIterator & pruned_plan_end) override;

  /**
   * @brief Get the global goal pose transformed to the costmap global frame
    * 中文：把完整全局路径的最终目标变换到代价地图全局坐标系。
  * 调用方：ControllerServer::computeAndPublishVelocity() 通过 PathHandler 基类接口调用。
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  geometry_msgs::msg::PoseStamped getTransformedGoal(
    const builtin_interfaces::msg::Time & stamp) override;

  /**
   * @brief Gets the global plan
    * 中文：返回当前保存的完整全局路径副本。
  * 调用方：当前 nav2_controller 生产代码没有直接调用；供外部通过 PathHandler 接口查询。
   * @return The global plan
   */
  nav_msgs::msg::Path getPlan() {return global_plan_;}

protected:
  /**
    * @brief Transform a pose to the global reference frame
    * 中文：把机器人位姿变换到全局路径自身的坐标系，供最近点搜索使用。
    * 调用方：FeasiblePathHandler::findPlanSegment()。
    * @param pose Current pose
    * @return output poose in global reference frame
    */
  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrame(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * Get the greatest extent of the costmap in meters from the center.
    * 中文：计算局部代价地图中心到最远边界的距离，作为默认搜索范围。
  * 调用方：FeasiblePathHandler::initialize()。
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * 中文：检查机器人是否已同时满足当前倒车或旋转约束点的 XY 与 yaw 容差。
    * 调用方：FeasiblePathHandler::transformLocalPlan()。
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
    * @brief Prune a path to only interesting portions
    * 中文：删除路径中指定迭代器之前的前缀，保留尚未执行的部分。
    * 调用方：FeasiblePathHandler::transformLocalPlan()。
    * @param plan Plan to prune
    * @param end Final path iterator
    */
  void prunePlan(nav_msgs::msg::Path & plan, const nav2_core::PathIterator end);

  // Dynamic parameters handler
  std::mutex mutex_;
  nav2::ParameterCallbacks parameter_callbacks_;
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_ {rclcpp::get_logger("FeasiblePathHandler")};
  std::string plugin_name_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_constraint_;
  geometry_msgs::msg::PoseStamped global_pose_;
  unsigned int constraint_locale_{0u};
  bool reject_unit_path_, enforce_path_inversion_, enforce_path_rotation_;
  double max_robot_pose_search_dist_, transform_tolerance_, prune_distance_;
  float inversion_xy_tolerance_, inversion_yaw_tolerance_, minimum_rotation_angle_;

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
  * 中文：线程安全地更新搜索、裁剪、约束容差和约束开关。
  * 调用方：initialize() 注册后，由 rclcpp 参数框架在参数验证成功后间接调用。
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__FEASIBLE_PATH_HANDLER_HPP_
