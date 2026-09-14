#ifndef NAV2_ROS_COMMON__PARAMETER_CALLBACKS_HPP_
#define NAV2_ROS_COMMON__PARAMETER_CALLBACKS_HPP_

// 本文件管理动态参数回调的注册句柄，将“参数合法性验证”和
// “参数生效后的业务更新”封装为成对的 activate/deactivate 操作。

#include <functional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

namespace nav2
{

/**
 * @class nav2::ParameterCallbacks
 * @brief 管理节点参数验证回调与更新回调的生命周期
 *
 * 新版 rclcpp 在参数提交后执行更新回调；旧版没有 post-set 接口，
 * 因而在验证成功后立即执行更新回调，两种版本的回调阶段略有差异。
 */
class ParameterCallbacks
{
public:
  /// 返回 successful=false 可拒绝整批参数更新。
  using ValidationCallback = std::function<rcl_interfaces::msg::SetParametersResult(
        const std::vector<rclcpp::Parameter> &)>;
  /// 在参数通过验证后同步业务状态，不负责决定参数是否可接受。
  using UpdateCallback = std::function<void(const std::vector<rclcpp::Parameter> &)>;

  /** @brief 在给定节点上注册验证和更新回调，并持有其注册句柄。 */
  template<typename NodeT>
  void activate(
    NodeT node, ValidationCallback validation_callback, UpdateCallback update_callback)
  {
#if RCLCPP_VERSION_GTE(17, 0, 0)
    post_set_params_handler_ = node->add_post_set_parameters_callback(
      std::move(update_callback));
    on_set_params_handler_ = node->add_on_set_parameters_callback(
      std::move(validation_callback));
#else
    on_set_params_handler_ = node->add_on_set_parameters_callback(
      [validation_callback = std::move(validation_callback),
        update_callback = std::move(update_callback)](
        const std::vector<rclcpp::Parameter> & parameters)
      {
        auto result = validation_callback(parameters);
        if (result.successful) {
          update_callback(parameters);
        }
        return result;
      });
#endif
  }

  /** @brief 从注册时使用的节点移除回调并释放句柄。 */
  template<typename NodeT>
  void deactivate(NodeT node)
  {
#if RCLCPP_VERSION_GTE(17, 0, 0)
    if (post_set_params_handler_ && node) {
      node->remove_post_set_parameters_callback(post_set_params_handler_.get());
    }
    post_set_params_handler_.reset();
#endif
    if (on_set_params_handler_ && node) {
      node->remove_on_set_parameters_callback(on_set_params_handler_.get());
    }
    on_set_params_handler_.reset();
  }

private:
#if RCLCPP_VERSION_GTE(17, 0, 0)
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
#endif
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__PARAMETER_CALLBACKS_HPP_