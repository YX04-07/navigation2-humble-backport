// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROS_COMMON__ACTION_CLIENT_HPP_
#define NAV2_ROS_COMMON__ACTION_CLIENT_HPP_

// 本文件统一 Nav2 中 Action 客户端的类型名称。
// 当前仅提供类型别名，不额外增加生命周期、线程或错误处理逻辑。

#include <memory>
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2
{
/**
  * @brief A ROS 2 action client for Nav2
  * This is a convenience type alias to simplify the use of action clients in Nav2
  * which may be further built up on in the future with custom APIs.
  */
template<typename ActionT>
using ActionClient = rclcpp_action::Client<ActionT>;

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__ACTION_CLIENT_HPP_
