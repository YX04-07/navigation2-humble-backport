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

#include <memory>

#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @file main.cpp
 * @brief Controller Server 的独立进程入口。
 *
 * 该入口直接创建并 spin ControllerServer；若使用组件容器，则通过
 * controller_server.cpp 末尾的 RCLCPP_COMPONENTS_REGISTER_NODE 加载同一个节点类。
 */

/**
 * @brief 初始化 ROS 2、运行 ControllerServer，节点退出后关闭 ROS 2。
 * 调用方：由操作系统的 C/C++ 运行时在启动 nav2_controller 可执行程序时调用。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_controller::ControllerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
