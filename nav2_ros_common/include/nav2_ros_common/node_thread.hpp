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

#ifndef NAV2_ROS_COMMON__NODE_THREAD_HPP_
#define NAV2_ROS_COMMON__NODE_THREAD_HPP_

// 本文件提供一个 RAII 后台执行线程，用于持续 spin 节点或执行器。
// 析构时会取消执行器并等待线程退出，因此析构可能短暂阻塞。

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace nav2
{
/**
 * @class nav2::NodeThread
 * @brief A background thread to process node/executor callbacks
 */
class NodeThread
{
public:
  /**
   * @brief A background thread to process node callbacks constructor
   * @param node_base Interface to Node to spin in thread
   */
  explicit NodeThread(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  : node_(node_base)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // 线程捕获 this；NodeThread 对象必须在线程停止前保持有效。
    thread_ = std::make_unique<std::thread>(
      [&]()
      {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
      });
  }

  /**
   * @brief A background thread to process executor's callbacks constructor
   * @param executor Interface to executor to spin in thread
   */
  explicit NodeThread(
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
  : executor_(executor)
  {
    thread_ = std::make_unique<std::thread>(
      [&]() {
        executor_->spin();
      });
  }

  /**
   * @brief A background thread to process node callbacks constructor
   * @param node Node pointer to spin in thread
   */
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  /**
   * @brief A destructor
   */
  ~NodeThread()
  {
    // 先解除 spin() 的阻塞，再 join，确保成员销毁前后台线程已退出。
    executor_->cancel();
    thread_->join();
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__NODE_THREAD_HPP_
