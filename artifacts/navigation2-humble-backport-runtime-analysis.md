# navigation2-humble-backport：NavigateToPose 源码运行时分析

## 分析基线

- 仓库：`https://github.com/YX04-07/navigation2-humble-backport.git`
- 当前实际 checkout：`main`
- commit：`fc28f71efd7350da4ffdf806118c2f61d068cf4b`
- 本地 refs：只有 `main`、`origin/main`、`origin/HEAD -> origin/main`；不存在 `humble-backport-devel`。
- 因而本文和架构图严格描述当前工作区 `main@fc28f71`。不能把结果表述成对一个本地不存在的 `humble-backport-devel` ref 的分析。

本文中的证据分三类：

- **源码确认**：由当前 checkout 的 C++、BT XML 或 launch 代码直接决定。
- **默认配置**：由仓库内 `nav2_bringup/params/nav2_params.yaml` 选择；部署时可被其他参数文件、remap 或 goal 中的 `behavior_tree` 覆盖。
- **集成契约**：Nav2 对机器人平台的要求，但实际订阅者、驱动和硬件实现不在本仓库。

## 一句话主链

`NavigateToPose` goal 进入 `BtActionServer<NavigateToPose>` 后，`NavigateToPoseNavigator::goalReceived()` 将经 TF 变换的 goal 和空 path 写入 blackboard；`BtActionServer::executeCallback()` 加载并运行默认 BT；树中的 `ComputePathToPose` 通过 ROS 2 Action 调用 `PlannerServer::computePlan()`，后者经 pluginlib 调用 `GlobalPlanner::createPlan()`；返回的 `nav_msgs/Path` 写回 blackboard 后，`FollowPath` 通过 Action 进入 `ControllerServer::computeControl()`，控制循环经 PathHandler 和 local costmap 调用 `Controller::computeVelocityCommands()`，再发布 `cmd_vel_nav`，依次经过 Velocity Smoother 和 Collision Monitor，最终以 `geometry_msgs/Twist` 发布到 `cmd_vel`，交给仓库外的底盘接口。

## 按调用顺序展开

| # | 运行时步骤 | 实际入口 / 调用 | 跨组件协议 |
|---:|---|---|---|
| 1 | 生命周期 bringup | `LifecycleManager::startup()` 对 managed nodes 先 `CONFIGURE`、再 `ACTIVATE` | `/<node>/change_state`、`/<node>/get_state` Services；`bond` 心跳 |
| 2 | 创建导航入口 | `BtNavigator::on_configure()` 经 pluginlib 创建 `NavigateToPoseNavigator`，后者构造 `BtActionServer` | pluginlib `NavigatorBase`；Action server 名 `navigate_to_pose` |
| 3 | 接收目标 | `NavigateToPoseNavigator::goalReceived()` → `initializeGoalPose()` | `NavigateToPose` Action；TF 将 goal 转到 global frame，并取得当前机器人位姿 |
| 4 | 准备 blackboard | 写入 `{goal}`、空 `{path}`、`number_recoveries=0` | BT blackboard，进程内调用 |
| 5 | 加载并 tick BT | `BtActionServer::executeCallback()` → `loadBehaviorTree()` → `BehaviorTreeEngine::run()` → `tree->tickOnce()` | BT.CPP plugin shared libraries；goal 可选择 XML/BT ID |
| 6 | 规划 BT 节点 | 默认树 1 Hz tick `ComputePathToPose`；通用 `BtActionNode` 使用 `async_send_goal()` | Action `compute_path_to_pose` |
| 7 | Planner Server | `PlannerServer::computePlan()` 等待 global costmap、取得/变换 start 与 goal、调用 `getPlan()` | TF；owned child `global_costmap`；结果 `nav_msgs/Path` |
| 8 | GlobalPlanner plugin | `planners_[planner_id]->createPlan(start, goal, viapoints, cancel_checker)` | pluginlib `nav2_core::GlobalPlanner`；默认配置 `GridBased = NavfnPlanner` |
| 9 | 控制 BT 节点 | `ComputePathToPoseAction::on_success()` 写回 `{path}`；`FollowPath` 读取并发送 path，重规划时可更新 goal | Action `follow_path` |
| 10 | Controller Server | `ControllerServer::computeControl()` 选择 controller/checkers/PathHandler，设置 path，并按频率循环 | owned child `local_costmap`；TF；`odom` |
| 11 | Controller plugin | `computeAndPublishVelocity()` 取得 pose/odom、变换和裁剪 plan，再调用 `Controller::computeVelocityCommands()` | pluginlib `nav2_core::Controller`；默认配置 `FollowPath = MPPIController` |
| 12 | 速度输出 | `publishVelocity()` → controller `cmd_vel`，launch remap 为 `cmd_vel_nav` | 当前默认 `enable_stamped_cmd_vel=false`，线上类型为 `geometry_msgs/Twist` |
| 13 | 安全处理 | Velocity Smoother：`cmd_vel_nav` → `cmd_vel_smoothed`；Collision Monitor：`cmd_vel_smoothed` → `cmd_vel` | ROS 2 Topics |
| 14 | 底盘执行 | 仓库外底盘订阅 `cmd_vel`，并向 Nav2 提供 odom、传感器和 TF | 集成契约，不是本仓库内实现 |

## 1. NavigateToPose Action Server 如何出现

`BtNavigator::on_configure()` 先创建自己的 TF buffer/listener，并读取 `global_frame`、`robot_base_frame` 和 `odom_topic`。随后它构造默认 navigator ID/type 列表，其中 `navigate_to_pose` 对应 `nav2_bt_navigator::NavigateToPoseNavigator`，再调用 `class_loader_.createUniqueInstance(navigator_type)`。见 [`bt_navigator.cpp:48`](../nav2_bt_navigator/src/bt_navigator.cpp#L48) 和 [`bt_navigator.cpp:83`](../nav2_bt_navigator/src/bt_navigator.cpp#L83)。

通用的 `BehaviorTreeNavigator<ActionT>::on_configure()` 取得默认 BT 文件，随后用 `getName()` 创建 `BtActionServer<ActionT>`。`NavigateToPoseNavigator::getName()` 返回字面值 `navigate_to_pose`，所以 Action Server 的真实名字来自这里，而不是图或文档中的约定。见 [`behavior_tree_navigator.hpp:199`](../nav2_core/include/nav2_core/behavior_tree_navigator.hpp#L199) 和 [`navigate_to_pose.hpp:75`](../nav2_bt_navigator/include/nav2_bt_navigator/navigators/navigate_to_pose.hpp#L75)。

`BtActionServer::on_configure()` 最终调用 `create_action_server<ActionT>(node, action_name_, executeCallback, ...)`，然后创建 `BehaviorTreeEngine` 和共享 blackboard。见 [`bt_action_server_impl.hpp:115`](../nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp#L115)。

## 2. Goal 接收、TF 与 blackboard

`NavigateToPoseNavigator::goalReceived()` 直接进入 `initializeGoalPose()`。该函数：

1. 用 `nav2_util::getCurrentPose()` 从 `global_frame` 到 `robot_frame` 的 TF 取得当前位姿。
2. 用 `transformPoseInTargetFrame()` 把请求中的目标位姿变换到 global frame。
3. 重置反馈状态和旧 path。
4. 将目标写到 `goal_blackboard_id_`，将空 `nav_msgs::Path` 写到 `path_blackboard_id_`。

任何一项 TF 获取失败都会把 `NavigateToPose::Result::TF_ERROR` 写入内部错误并拒绝执行。见 [`navigate_to_pose.cpp:93`](../nav2_bt_navigator/src/navigators/navigate_to_pose.cpp#L93) 和 [`navigate_to_pose.cpp:224`](../nav2_bt_navigator/src/navigators/navigate_to_pose.cpp#L224)。

默认 BT 文件由 `default_nav_to_pose_bt_xml` 参数决定；未设置时指向 `navigate_to_pose_w_replanning_and_recovery.xml`。见 [`navigate_to_pose.cpp:69`](../nav2_bt_navigator/src/navigators/navigate_to_pose.cpp#L69)。Action goal 的 `behavior_tree` 字段仍可选择另一个 XML 或已注册 BT ID，因此“默认树”不是不可变硬编码。

## 3. BT 的加载和执行

`BehaviorTreeEngine` 构造时遍历 BT node shared libraries，通过 BT.CPP 的 `factory_.registerFromPlugin()` 注册节点。见 [`behavior_tree_engine.cpp:33`](../nav2_behavior_tree/src/behavior_tree_engine.cpp#L33)。

收到 Action goal 后，`BtActionServer::executeCallback()`：

1. 读取当前 goal 的 `behavior_tree` 字段。
2. 调用 `loadBehaviorTree()`，空字段回退到默认 XML/ID；该函数会注册搜索目录中的 XML/BT ID 并创建 tree。
3. 调用 `BehaviorTreeEngine::run()`。
4. 在 run loop 中反复执行 `tree->tickOnce()`，同时处理 cancel、preemption、feedback 和日志。
5. 结束后 `haltAllActions()` 并把 BT 状态转换成 Action result。

关键位置见 [`bt_action_server_impl.hpp:253`](../nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp#L253)、[`bt_action_server_impl.hpp:411`](../nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp#L411) 和 [`behavior_tree_engine.cpp:45`](../nav2_behavior_tree/src/behavior_tree_engine.cpp#L45)。

当前默认 NavigateToPose 树是 BT.CPP format 4。主体为 `RecoveryNode → PipelineSequence`，其中 planner selector 默认 `GridBased`、controller selector 默认 `FollowPath`；规划分支受 `RateController hz="1.0"` 控制，之后进入 `FollowPath`。见 [`navigate_to_pose_w_replanning_and_recovery.xml:7`](../nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml#L7)。

一个容易误读的细节是：`PipelineSequence` 让规划分支继续以 1 Hz 重新 tick，而 `FollowPathAction::on_wait_for_result()` 会检测 blackboard 中的新 path，并用 `goal_updated_` 更新正在运行的 FollowPath goal。见 [`follow_path_action.cpp:71`](../nav2_behavior_tree/plugins/action/follow_path_action.cpp#L71)。因此规划和跟踪并不是只串行各运行一次。

## 4. ComputePathToPose 到 Planner plugin

BT plugin 注册 `ComputePathToPose` 时，把 ROS Action 名固定为 `compute_path_to_pose`。它从 blackboard 读取 goal、planner ID、可选 start/viapoints；成功后把 Action result 的 `path` 写回 blackboard。见 [`compute_path_to_pose_action.cpp:31`](../nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp#L31) 和 [`compute_path_to_pose_action.cpp:101`](../nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp#L101)。通用 `BtActionNode` 最终用 `action_client_->async_send_goal()` 发出 goal，见 [`bt_action_node.hpp:476`](../nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp#L476)。

`PlannerServer` 在构造函数内创建名为 `global_costmap` 的 `Costmap2DROS` child node；configure 时取得 `Costmap2D*` 和 TF buffer，再按参数中的 plugin types 通过 `gp_loader_.createUniqueInstance()` 加载 `nav2_core::GlobalPlanner`。它在 `compute_path_to_pose` 上创建 Action Server，回调为 `PlannerServer::computePlan()`。见 [`planner_server.cpp:46`](../nav2_planner/src/planner_server.cpp#L46) 和 [`planner_server.cpp:68`](../nav2_planner/src/planner_server.cpp#L68)。

`computePlan()` 的关键顺序是：

1. `waitForCostmap()`。
2. 未显式提供 start 时，从 global costmap/TF 取得当前机器人位姿。
3. 将 start、goal 变换到 global frame。
4. `getPlan(start, goal_pose, viapoints, planner_id, cancel_checker)`。
5. 校验 path，发布 `plan` topic 供可视化，并完成 Action。

`getPlan()` 的动态分派点是 `planners_[planner_id]->createPlan(start, goal, viapoints, cancel_checker)`。见 [`planner_server.cpp:536`](../nav2_planner/src/planner_server.cpp#L536) 和 [`planner_server.cpp:640`](../nav2_planner/src/planner_server.cpp#L640)。这就是 server 到具体 GlobalPlanner 的实际 pluginlib 调用边界。

当前 checked-in 参数选择 `planner_plugins: ["GridBased"]` 和 `GridBased.plugin: nav2_navfn_planner::NavfnPlanner`，见 [`nav2_params.yaml:393`](../nav2_bringup/params/nav2_params.yaml#L393)。`NavfnPlanner::createPlan()` 是当前默认实现入口，见 [`navfn_planner.cpp:121`](../nav2_navfn_planner/src/navfn_planner.cpp#L121)。这只是仓库默认配置；部署参数可以选其他 GlobalPlanner。

当前本地 `nav2_core::GlobalPlanner` 接口的 `createPlan()` 明确包含 `viapoints` 与 `cancel_checker`，见 [`global_planner.hpp:78`](../nav2_core/include/nav2_core/global_planner.hpp#L78)。分析该 backport 时应以这个本地签名为准。

## 5. FollowPath 到 Controller plugin

BT plugin 注册 `FollowPath` 时把 ROS Action 名固定为 `follow_path`。`on_tick()` 从 blackboard 读取 path、controller ID、goal/progress checker ID 和 PathHandler ID。见 [`follow_path_action.cpp:31`](../nav2_behavior_tree/plugins/action/follow_path_action.cpp#L31) 和 [`follow_path_action.cpp:130`](../nav2_behavior_tree/plugins/action/follow_path_action.cpp#L130)。

`ControllerServer` 在构造函数内创建 `local_costmap` child node。configure 时分别通过 pluginlib 加载 ProgressChecker、GoalChecker、PathHandler 和 `nav2_core::Controller`；controller 的加载点是 `lp_loader_.createUniqueInstance()`。随后创建 `follow_path` Action Server，执行回调为 `ControllerServer::computeControl()`。见 [`controller_server.cpp:38`](../nav2_controller/src/controller_server.cpp#L38) 和 [`controller_server.cpp:162`](../nav2_controller/src/controller_server.cpp#L162)。

`computeControl()` 从 Action goal 选择具体 plugin/checker/PathHandler，调用 `setPlannerPath(goal->path)`，然后按 `controller_frequency` 循环：等待 local costmap current、接受更新 path、检查 goal、调用 `computeAndPublishVelocity()`。见 [`controller_server.cpp:491`](../nav2_controller/src/controller_server.cpp#L491)。

`computeAndPublishVelocity()` 的顺序是：

1. `getRobotPose()` 从 local costmap TF 获取当前 pose。
2. 从 `OdomSmoother` 取得当前 `odom` twist。
3. `PathHandler::getTransformedGoal()`、`findPlanSegment()`、`transformLocalPlan()` 对 global path 做定位、裁剪和局部坐标变换。
4. 调用 `controllers_[current_controller_]->computeVelocityCommands(pose, twist, goal_checker, transformed_global_plan, goal)`。
5. 补齐 base frame 与时间戳，再进入 `publishVelocity()`。

见 [`controller_server.cpp:708`](../nav2_controller/src/controller_server.cpp#L708) 和 [`controller_server.cpp:903`](../nav2_controller/src/controller_server.cpp#L903)。当前本地 `nav2_core::Controller` 接口直接接收 transformed plan 和 global goal，见 [`controller.hpp:118`](../nav2_core/include/nav2_core/controller.hpp#L118)。

checked-in 参数选择 `controller_plugins: ["FollowPath"]` 和 `FollowPath.plugin: nav2_mppi_controller::MPPIController`，见 [`nav2_params.yaml:92`](../nav2_bringup/params/nav2_params.yaml#L92)。`MPPIController::computeVelocityCommands()` 会锁住 local `Costmap2D`，调用 `optimizer_.evalControl(...)` 并返回 `TwistStamped`，见 [`controller.cpp:103`](../nav2_mppi_controller/src/controller.cpp#L103)。同样，这个具体算法是默认配置选择，不是 Controller Server 的硬编码。

## 6. cmd_vel 到机器人

Controller Server 内部 publisher 名为 `cmd_vel`，但默认 navigation launch 将 controller server 的 `cmd_vel` remap 为 `cmd_vel_nav`。Behavior Server 的恢复动作也被 remap 到同一输入。见 [`navigation_launch.py:163`](../nav2_bringup/launch/navigation_launch.py#L163) 和 [`navigation_launch.py:206`](../nav2_bringup/launch/navigation_launch.py#L206)。

接下来的实际 topic 链为：

`controller/behavior cmd_vel` → **remap** `cmd_vel_nav` → Velocity Smoother 订阅其本地 `cmd_vel` → 发布 `cmd_vel_smoothed` → Collision Monitor 订阅 `cmd_vel_smoothed` → 发布最终 `cmd_vel`。

Velocity Smoother 的 publisher/subscriber 创建点见 [`velocity_smoother.cpp:142`](../nav2_velocity_smoother/src/velocity_smoother.cpp#L142)；launch 中的输入 remap 见 [`navigation_launch.py:238`](../nav2_bringup/launch/navigation_launch.py#L238)。Collision Monitor 用参数决定输入和输出 topic，代码见 [`collision_monitor_node.cpp:65`](../nav2_collision_monitor/src/collision_monitor_node.cpp#L65)，checked-in 值见 [`nav2_params.yaml:510`](../nav2_bringup/params/nav2_params.yaml#L510)。

`TwistPublisher` 根据 `enable_stamped_cmd_vel` 选择 `geometry_msgs/Twist` 或 `TwistStamped`。当前 controller、Behavior Server、Velocity Smoother 与 Collision Monitor 的 checked-in 参数均为 `false`，所以图中最终链路标为 `geometry_msgs/Twist`。这仍是配置相关结论，不是类型永远固定。

本仓库没有实现最终底盘 subscriber，也不能从本仓库证明某一具体驱动节点名。能够确定的边界仅是：底盘侧应订阅最终 `cmd_vel`，并提供 Nav2 需要的 `odom`、传感器数据以及 `odom → base_link/base_footprint` TF。

## 7. Global/Local costmap、地图、AMCL 与 TF

Planner Server 和 Controller Server 分别**拥有** `global_costmap`、`local_costmap` child node，不只是远程订阅一个 costmap topic。`Costmap2DROS::on_configure()` 创建 `LayeredCostmap` 和 TF buffer，再通过 pluginlib `createSharedInstance()` 加载 layers/filters；更新循环从 TF 取机器人位姿并调用 `layered_costmap_->updateMap()`。见 [`costmap_2d_ros.cpp:129`](../nav2_costmap_2d/src/costmap_2d_ros.cpp#L129) 和 [`costmap_2d_ros.cpp:599`](../nav2_costmap_2d/src/costmap_2d_ros.cpp#L599)。

checked-in local costmap 使用 `odom` frame、rolling window、VoxelLayer + InflationLayer，并从 `scan` 获取障碍物；global costmap 使用 `map` frame、StaticLayer + ObstacleLayer + InflationLayer。见 [`nav2_params.yaml:240`](../nav2_bringup/params/nav2_params.yaml#L240) 和 [`nav2_params.yaml:297`](../nav2_bringup/params/nav2_params.yaml#L297)。filters 也由同一 YAML 决定。

Map Server 在 lifecycle activate 后以 transient-local/latched QoS 发布 `map` OccupancyGrid。见 [`map_server.cpp:114`](../nav2_map_server/src/map_server/map_server.cpp#L114)。AMCL 以 LaserScan 与 odometry motion update 粒子滤波，发布 `amcl_pose`，并计算/广播 `map → odom`。见 [`amcl_node.cpp:570`](../nav2_amcl/src/amcl_node.cpp#L570)、[`amcl_node.cpp:835`](../nav2_amcl/src/amcl_node.cpp#L835) 和 [`amcl_node.cpp:869`](../nav2_amcl/src/amcl_node.cpp#L869)。

完整 TF 链的责任边界为：AMCL 提供 `map → odom`；机器人平台/里程计侧提供 `odom → base_link`（当前 AMCL 默认 base frame 参数可能为 `base_footprint`，具体由部署参数统一）；Nav2 的 BT Navigator、costmaps、planner 和 controller 消费 TF。平台侧那一半不是本仓库实现。

## 8. 恢复行为与 Smoother Server

默认 BT 在 planner/controller 局部恢复中分别调用：

- Service `global_costmap/clear_entirely_global_costmap`
- Service `local_costmap/clear_entirely_local_costmap`

系统级恢复的 RoundRobin 还会依次尝试清图、`Spin`、`Wait`、`BackUp`。见 [`navigate_to_pose_w_replanning_and_recovery.xml:29`](../nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml#L29) 和 [`navigate_to_pose_w_replanning_and_recovery.xml:48`](../nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml#L48)。

Behavior Server 按 `behavior_plugins` 经 pluginlib 创建 `nav2_core::Behavior`。`TimedBehavior::configure()` 以 behavior plugin ID 为 Action Server 名，并创建速度 publisher；当前默认 ID/Action 包括 `spin`、`backup`、`wait`。它还可按 plugin 需要订阅 local/global `costmap_raw` 与 `published_footprint`。见 [`behavior_server.cpp:75`](../nav2_behaviors/src/behavior_server.cpp#L75)、[`timed_behavior.hpp:138`](../nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp#L138) 和 [`nav2_params.yaml:426`](../nav2_bringup/params/nav2_params.yaml#L426)。

Smoother Server 确实存在：它创建 `smooth_path` Action Server，经 pluginlib 加载 `nav2_core::Smoother`，并订阅 global costmap/footprint 做碰撞检查。见 [`nav2_smoother.cpp:47`](../nav2_smoother/src/nav2_smoother.cpp#L47) 和 [`nav2_smoother.cpp:93`](../nav2_smoother/src/nav2_smoother.cpp#L93)。但是当前默认 `navigate_to_pose_w_replanning_and_recovery.xml` **没有** `SmoothPath` 节点。因此它不是这条默认 NavigateToPose 主链的必经步骤；只有 goal 选择了包含 `SmoothPath` 的其他 BT，或者部署替换默认 BT 时才参与。

## 9. Lifecycle 的真实控制关系

`navigation_launch.py` 把 controller、smoother、planner、behavior、velocity_smoother、collision_monitor、bt_navigator 等列入 `lifecycle_nodes`，并启动 `lifecycle_manager_navigation`。见 [`navigation_launch.py:45`](../nav2_bringup/launch/navigation_launch.py#L45) 和 [`navigation_launch.py:283`](../nav2_bringup/launch/navigation_launch.py#L283)。Map Server 和 AMCL 由 localization launch 中独立的 lifecycle manager 管理。

`LifecycleServiceClient` 对每个 managed node 构造 `<node>/change_state` 和 `<node>/get_state` Service client。`LifecycleManager::startup()` 对列表顺序先发 CONFIGURE，再发 ACTIVATE；每次 ACTIVATE 成功后建立 bond，监控节点存活。见 [`lifecycle_service_client.hpp:42`](../nav2_util/include/nav2_util/lifecycle_service_client.hpp#L42) 和 [`lifecycle_manager.cpp:297`](../nav2_lifecycle_manager/src/lifecycle_manager.cpp#L297)。

Lifecycle Manager 自身是普通 `rclcpp::Node`，不是图中 managed LifecycleNode 边界的一员；图里的安全色连线表示它通过 services/transitions 管理其他节点。

## 10. 协议与 pluginlib 汇总

| 类型 | 名称 / 接口 | 发送方 → 接收方 | 证据性质 |
|---|---|---|---|
| Action | `navigate_to_pose` / `nav2_msgs::action::NavigateToPose` | client → BT Navigator | 源码确认 |
| Action | `compute_path_to_pose` | BT `ComputePathToPoseAction` → Planner Server | 源码确认 |
| Action | `follow_path` | BT `FollowPathAction` → Controller Server | 源码确认 |
| Action | `spin`、`backup`、`wait` | 默认 BT → Behavior plugins | XML + checked-in plugin IDs |
| Action | `smooth_path` | 可选 BT → Smoother Server | server 源码确认；默认树未调用 |
| Service | `global_costmap/clear_entirely_global_costmap` | 默认 BT → global costmap clear service | 默认 XML |
| Service | `local_costmap/clear_entirely_local_costmap` | 默认 BT → local costmap clear service | 默认 XML |
| Service | `/<node>/change_state`、`/<node>/get_state` | Lifecycle Manager → managed node | 源码确认 |
| Topic | `map` | Map Server → AMCL / global costmap StaticLayer | 源码 + 默认配置 |
| Topic | `scan` | robot sensors → AMCL / costmap obstacle layers | 默认配置 + 集成契约 |
| Topic | `odom` | robot platform → BT/controller velocity feedback | 默认参数 + 集成契约 |
| Topic | `plan` | Planner Server → 可视化消费者 | 源码确认，非控制主数据通路 |
| Topic | `cmd_vel_nav` → `cmd_vel_smoothed` → `cmd_vel` | Controller/Behavior → Smoother → Collision Monitor → chassis | launch + 默认配置 |
| Topic | `local/global_costmap/costmap_raw`、`published_footprint` | costmaps → Behavior/Smoother | 源码 + 默认参数 |
| TF | `map → odom` | AMCL → TF graph | 源码确认 |
| TF | `odom → base_link/base_footprint` | robot localization/driver → TF graph | 仓库外集成契约 |
| pluginlib | `NavigatorBase` | BtNavigator → NavigateToPoseNavigator | 源码确认 |
| BT.CPP plugin | BT node shared libraries | BehaviorTreeEngine → ComputePath/FollowPath/... nodes | 源码确认 |
| pluginlib | `GlobalPlanner` | Planner Server → configured planner | 源码确认；Navfn 为默认配置 |
| pluginlib | costmap `Layer` / filters | Costmap2DROS → configured layers | 源码确认；具体 layers 为默认配置 |
| pluginlib | `Controller` + checker/PathHandler interfaces | Controller Server → configured plugins | 源码确认；MPPI 为默认配置 |
| pluginlib | `Behavior` | Behavior Server → spin/backup/wait/... | 源码 + 默认配置 |
| pluginlib | `Smoother` | Smoother Server → configured smoother | 源码 + 默认配置；默认 NavigateToPose 不调用 |

## 结论

从源码角度，当前 checkout 中一次默认 `NavigateToPose` 的关键函数序列可以压缩为：

`NavigateToPoseNavigator::goalReceived / initializeGoalPose`
→ `BtActionServer<NavigateToPose>::executeCallback`
→ `BtActionServer::loadBehaviorTree`
→ `BehaviorTreeEngine::run / BT::Tree::tickOnce`
→ `ComputePathToPoseAction` / `BtActionNode::async_send_goal`
→ `PlannerServer::computePlan / getPlan`
→ `GlobalPlanner::createPlan`（默认 `NavfnPlanner::createPlan`）
→ `ComputePathToPoseAction::on_success` 写回 `{path}`
→ `FollowPathAction` / `BtActionNode::async_send_goal`
→ `ControllerServer::computeControl / computeAndPublishVelocity`
→ `PathHandler` 变换与裁剪 plan
→ `Controller::computeVelocityCommands`（默认 `MPPIController::computeVelocityCommands`）
→ `ControllerServer::publishVelocity`
→ `cmd_vel_nav`
→ `VelocitySmoother`
→ `cmd_vel_smoothed`
→ `CollisionMonitor`
→ `cmd_vel`
→ 仓库外机器人底盘。

其中 Smoother Server（path smoother）与 Velocity Smoother（velocity smoother）是两个不同组件：前者提供可选的 `smooth_path` Action，当前默认 NavigateToPose BT 不调用；后者位于默认 `cmd_vel` 输出链上。
