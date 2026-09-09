# Nav2 Controller

The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://docs.nav2.org/plugins/index.html) for a list of the currently known and available controller plugins.

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

The `ControllerServer` makes use of a [nav2_util::TwistPublisher](../nav2_util/README.md#twist-publisher-and-twist-subscriber-for-commanded-velocities).

## 插件目录说明

`ControllerServer` 是插件宿主和调度器，并不固定使用某一种控制算法。一次 `FollowPath`
任务会组合四类插件：

- `nav2_core::Controller`：根据机器人位姿、速度和局部路径计算速度命令。默认配置使用
	`nav2_mppi_controller::MPPIController`，其实现不在本包的 `plugins/` 目录中。
- `nav2_core::ProgressChecker`：判断机器人是否持续移动，长时间没有有效进展时终止任务。
- `nav2_core::GoalChecker`：根据位置、朝向、速度或路径长度判断机器人是否到达终点。
- `nav2_core::PathHandler`：从全局路径中选择当前相关部分，裁剪并变换到局部控制坐标系。

本包的 `plugins/` 目录保存后三类插件的 `.cpp` 实现。每个插件通过
`PLUGINLIB_EXPORT_CLASS` 导出工厂，由 `plugins.xml` 描述共享库、具体类型和基接口，
再由 `ControllerServer` 中对应的 `pluginlib::ClassLoader` 在运行时创建。配置中的插件
列表给出逻辑 ID（例如 `goal_checker`），而 `<ID>.plugin` 参数给出完整 C++ 类型名
（例如 `nav2_controller::SimpleGoalChecker`）。因此可以不修改服务器代码，只改参数就
能替换或同时加载多个算法实现。
