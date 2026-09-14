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

#ifndef NAV2_ROS_COMMON__PUBLISHER_HPP_
#define NAV2_ROS_COMMON__PUBLISHER_HPP_

// 本文件统一 Nav2 中发布者的类型名称。
// Publisher 是生命周期发布者，只有激活后才能正常发布消息。

#include <memory>
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace nav2
{
/**
  * @brief A ROS 2 publisher for Nav2
  * This is a convenience type alias to simplify the use of publishers in Nav2
  * which may be further built up on in the future with custom APIs.
  */
template<typename MessageT>
using Publisher = rclcpp_lifecycle::LifecyclePublisher<MessageT>;

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__PUBLISHER_HPP_
