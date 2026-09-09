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

#ifndef NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_

/**
 * @file controller_server.hpp
 * @brief Controller Server 的核心声明。
 *
 * ControllerServer 本身不是具体的路径跟踪算法，而是 FollowPath Action 的执行器和
 * 多类插件的宿主。它管理局部代价地图、选择插件、运行固定频率控制循环，最终发布
 * 速度命令及跟踪反馈。具体算法由 Controller、GoalChecker、ProgressChecker 和
 * PathHandler 四类 pluginlib 插件提供。
 */

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/path_handler.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/twist_publisher.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_controller/parameter_handler.hpp"

namespace nav2_controller
{

class ProgressChecker;
/**
 * @class nav2_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 *
 * 中文说明：接收全局路径后，本类按“路径处理 -> 进度检查 -> 到达检查 -> 控制器算速”
 * 的顺序执行控制任务，并把底层异常转换为 FollowPath Action 的标准错误码。
 */
class ControllerServer : public nav2::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;
  using ProgressCheckerMap = std::unordered_map<std::string, nav2_core::ProgressChecker::Ptr>;
  using PathHandlerMap = std::unordered_map<std::string, nav2_core::PathHandler::Ptr>;

  /**
   * @brief Constructor for nav2_controller::ControllerServer
    * 中文：创建生命周期节点、四类插件加载器以及控制器使用的局部代价地图节点。
  * 调用方：独立运行时由 main() 创建；组件模式下由 ROS 2 组件容器创建。
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_controller::ControllerServer
    * 中文：先销毁插件实例，再停止代价地图线程，避免卸载插件库后仍持有插件对象。
  * 调用方：由 shared_ptr 或 ROS 2 组件容器在节点对象生命周期结束时调用。
   */
  ~ControllerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
    * 中文：读取参数、创建四类插件、启动局部代价地图线程，并建立 Action、话题和里程计接口。
    * 调用方：由 ROS 2 生命周期框架在 configure 转换时调用；业务代码不直接调用。
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
    * 中文：激活代价地图、控制器、发布器和 Action Server，并初始化其余三类插件。
    * 调用方：由 ROS 2 生命周期框架在 activate 转换时调用。
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
    * 中文：停止接收控制任务，停用各资源，并发布零速度确保机器人停车。
    * 调用方：由 ROS 2 生命周期框架在 deactivate 转换时调用。
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
    * 中文：清理控制器并释放插件、Action、线程、订阅器和发布器等运行资源。
    * 调用方：由 ROS 2 生命周期框架调用；on_configure() 失败时也会直接调用。
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
    * 中文：处理生命周期节点关闭事件；资源释放主要由 deactivate 和 cleanup 完成。
  * 调用方：由 ROS 2 生命周期框架在 shutdown 转换时调用。
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2::SimpleActionServer<Action>;

  /**
   * @brief Goal received callback to validate a new goal before acceptance
    * 中文：在接受目标前检查四类插件 ID 是否有效，并拒绝空路径。
  * 调用方：注册到 SimpleActionServer，由其收到 FollowPath 目标时调用。
   * @param goal The incoming goal to validate
   * @return true if goal should be accepted, false to reject
   */
  bool goalReceived(std::shared_ptr<const Action::Goal> goal);

  // Our action server implements the FollowPath action
  typename ActionServer::SharedPtr action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
  * 中文：FollowPath 主循环；支持取消和抢占，等待代价地图更新，周期性检查目标并计算速度，
  * 同时负责将控制异常映射为 Action 结果。
  * 调用方：注册为 SimpleActionServer 的执行回调，由其执行线程调用。
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
    * 中文：选择局部控制器；仅配置一个插件且请求为空时自动使用该插件。
  * 调用方：goalReceived()、computeControl() 和 updateGlobalPath()。
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
    * 中文：选择目标检查器；该插件决定机器人何时被视为到达终点。
  * 调用方：goalReceived()、computeControl() 和 updateGlobalPath()。
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid progress checker ID name for the specified parameter
    * 中文：选择进度检查器；未配置且请求为空时允许关闭进度检查。
  * 调用方：goalReceived()、computeControl() 和 updateGlobalPath()。
   *
   * @param c_name The progress checker name
   * @param name Reference to the name to use for progress checking if any valid available
   * @return bool Whether it found a valid progress checker to use
   */
  bool findProgressCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid path handler ID name for the specified parameter
    * 中文：选择路径处理器；该插件负责裁剪并变换控制器当前需要的局部路径。
  * 调用方：goalReceived()、computeControl() 和 updateGlobalPath()。
   *
   * @param c_name The path handler name
   * @param name Reference to the name to use for path handling if any valid available
   * @return bool Whether it found a valid path handler to use
   */
  bool findPathHandlerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
    * 中文：把新路径同时交给控制器和路径处理器，保存终点并重置目标检查状态。
  * 调用方：computeControl() 接收初始目标时，以及 updateGlobalPath() 接受抢占目标时。
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
    * 中文：执行一个控制周期，完成位姿获取、进度检查、路径处理、控制器算速和反馈发布。
  * 调用方：computeControl() 的周期控制循环。
   */
  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
    * 中文：处理 Action 抢占请求，可切换四类插件并用新路径替换当前路径。
  * 调用方：computeControl() 的周期控制循环。
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
    * 中文：校验速度有限性后，在生命周期发布器已激活且存在订阅者时发布速度。
  * 调用方：computeAndPublishVelocity() 和 publishZeroVelocity()。
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
    * 中文：构造并发布带当前时间戳和机器人基坐标系的全零速度命令。
  * 调用方：on_deactivate() 和 onGoalExit()。
   */
  void publishZeroVelocity();
  /**
   * @brief Called on goal exit
    * 中文：任务结束时按需停车，并重置所有控制器的内部状态。
  * 调用方：computeControl() 的取消、异常和成功结束分支。
   */
  void onGoalExit(bool force_stop);
  /**
   * @brief Wait for costmap to become current, with timeout
    * 中文：等待局部代价地图更新完成；超时会终止当前控制任务。
  * 调用方：computeControl() 的周期控制循环。
   * @return Duration in seconds spent waiting for the costmap (0.0 if already current)
   * @throw nav2_core::ControllerTimedOut if costmap update times out
   */
  double waitForCostmap();
  /**
   * @brief Checks if goal is reached
    * 中文：将终点变换到代价地图坐标系，再委托当前 GoalChecker 判断是否到达。
  * 调用方：computeControl() 的周期控制循环。
   * @return true or false
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot in costmap's frame
    * 中文：从局部代价地图节点查询机器人在其全局坐标系中的当前位姿。
  * 调用方：computeAndPublishVelocity() 和 isGoalReached()。
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
    * 中文：对单个速度分量应用死区，小于等于阈值的噪声被置零。
  * 调用方：getThresholdedTwist() 分别处理三个速度分量时调用。
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
    * 中文：分别对 x、y 线速度和 z 角速度应用配置的里程计死区。
  * 调用方：computeAndPublishVelocity() 和 isGoalReached()。
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  geometry_msgs::msg::Twist getThresholdedTwist(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::Twist twist_thresh;
    twist_thresh.linear.x = getThresholdedVelocity(twist.linear.x,
      params_->min_x_velocity_threshold);
    twist_thresh.linear.y = getThresholdedVelocity(twist.linear.y,
      params_->min_y_velocity_threshold);
    twist_thresh.angular.z = getThresholdedVelocity(twist.angular.z,
      params_->min_theta_velocity_threshold);
    return twist_thresh;
  }

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav2_util::OdomSmoother> odom_sub_;
  std::unique_ptr<nav2_util::TwistPublisher> vel_publisher_;
  nav2::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
  nav2::Publisher<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_pub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  ProgressCheckerMap progress_checkers_;
  std::string progress_checker_ids_concat_, current_progress_checker_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::string controller_ids_concat_, current_controller_;

  // Path Handler Plugins
  pluginlib::ClassLoader<nav2_core::PathHandler> path_handler_loader_;
  PathHandlerMap path_handlers_;
  std::string path_handler_ids_concat_, current_path_handler_;

  size_t start_index_;
  geometry_msgs::msg::PoseStamped end_pose_;
  geometry_msgs::msg::PoseStamped transformed_end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;
  nav_msgs::msg::Path transformed_global_plan_;
  std::unique_ptr<nav2_controller::ParameterHandler> param_handler_;
  Parameters * params_;
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr transformed_plan_pub_;
  double transform_tolerance_;

private:
  /**
    * @brief Callback for speed limiting messages
    * 中文：把收到的绝对值或百分比限速转发给所有已加载控制器。
    * 调用方：注册为 ROS 2 订阅回调，由 executor 收到 speed_limit 消息时调用。
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::ConstSharedPtr & msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
