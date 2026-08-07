#ifndef NAV2_ROS_COMMON__PARAMETER_CALLBACKS_HPP_
#define NAV2_ROS_COMMON__PARAMETER_CALLBACKS_HPP_

#include <functional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

namespace nav2
{

class ParameterCallbacks
{
public:
  using ValidationCallback = std::function<rcl_interfaces::msg::SetParametersResult(
        const std::vector<rclcpp::Parameter> &)>;
  using UpdateCallback = std::function<void(const std::vector<rclcpp::Parameter> &)>;

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