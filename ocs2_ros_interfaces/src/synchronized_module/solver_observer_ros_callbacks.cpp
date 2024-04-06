/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/synchronized_module/solver_observer_ros_callbacks.hpp"

#include <memory>
#include <vector>

#include "ocs2_core/misc/LinearInterpolation.hpp"

namespace ocs2
{
namespace ros
{

template <class NodeT>
SolverObserver::constraint_callback_t create_constraint_cb(
  std::weak_ptr<NodeT> nodeptr, const scalar_array_t & observing_time_points,
  const std::vector<std::string> & topic_names,
  CallbackInterpolationStrategy interpolation_strategy)
{
  using vector_ref_array_t = std::vector<std::reference_wrapper<const vector_t>>;
  if (observing_time_points.size() != topic_names.size()) {
    throw std::runtime_error(
      "[create_constraint_cb] For each observing time points, you should provide a unique "
      "topic name!");
  }
  std::vector<rclcpp::Publisher<ConstraintMsg>::SharedPtr> constraint_pubs;
  if (auto _node = nodeptr.lock()) {
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params =
      _node->get_node_parameters_interface();
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
      _node->get_node_topics_interface();
    for (const auto & name : topic_names) {
      constraint_pubs.push_back(rclcpp::create_publisher<ConstraintMsg>(
        node_params, node_topics, name, rclcpp::ParametersQoS()));
    }
  } else {
    throw std::runtime_error("[create_lagrangian_cb] The node is not available!");
  }

  // note that we need to copy the publishers as the local ones will go out of scope.
  // Good news is that ROS publisher behaves like std::sharted_ptr
  // ("Once all copies of a specific Publisher go out of scope, any subscriber status callbacks
  // associated with that handle will stop being called.")
  return [=](
           const scalar_array_t & time_trajectory, const vector_ref_array_t & termConstraintArray) {
    if (!time_trajectory.empty()) {
      for (size_t i = 0; i < observing_time_points.size(); i++) {
        const auto t = time_trajectory.front() + observing_time_points[i];
        const auto index_alpha = LinearInterpolation::timeSegment(t, time_trajectory);
        const auto constraint = [&]() -> vector_t {
          switch (interpolation_strategy) {
            case CallbackInterpolationStrategy::nearest_time:
              return (index_alpha.second > 0.5) ? termConstraintArray[index_alpha.first].get()
                                                : termConstraintArray[index_alpha.first + 1].get();
            case CallbackInterpolationStrategy::linear_interpolation:
              return LinearInterpolation::interpolate(
                index_alpha, termConstraintArray,
                [](const vector_ref_array_t & array, size_t t) -> const vector_t & {
                  return array[t].get();
                });
            default:
              throw std::runtime_error(
                "[create_constraint_cb] This CallbackInterpolationStrategy is not "
                "implemented!");
          }
        }();
        constraint_pubs[i]->publish(ros_msg_conversions::create_constraint_msg(t, constraint));
      }
    }
  };
}

template <class NodeT>
SolverObserver::lagrangian_callback_t create_lagrangian_cb(
  std::weak_ptr<NodeT> nodeptr, const scalar_array_t & observing_time_points,
  const std::vector<std::string> & topic_names,
  CallbackInterpolationStrategy interpolation_strategy)
{
  if (observing_time_points.size() != topic_names.size()) {
    throw std::runtime_error(
      "[create_lagrangian_cb] For each observing time points, you should provide a unique "
      "topic name!");
  }
  std::vector<rclcpp::Publisher<LagrangianMetricsMsg>::SharedPtr> metrics_publishers;
  if (auto _node = nodeptr.lock()) {
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params =
      _node->get_node_parameters_interface();
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
      _node->get_node_topics_interface();
    for (const auto & name : topic_names) {
      metrics_publishers.push_back(rclcpp::create_publisher<LagrangianMetricsMsg>(
        node_params, node_topics, name, rclcpp::ParametersQoS()));
    }
  } else {
    throw std::runtime_error("[create_lagrangian_cb] The node is not available!");
  }

  // note that we need to copy the publishers as the local ones will go out of scope.
  // Good news is that ROS publisher behaves like std::sharted_ptr
  // ("Once all copies of a specific Publisher go out of scope, any subscriber status callbacks
  // associated with that handle will stop being called.")
  return [=](
           const scalar_array_t & time_trajectory,
           const std::vector<LagrangianMetricsConstRef> & termMetricsArray) {
    if (!time_trajectory.empty()) {
      for (size_t i = 0; i < observing_time_points.size(); i++) {
        const auto t = time_trajectory.front() + observing_time_points[i];
        const auto index_alpha = LinearInterpolation::timeSegment(t, time_trajectory);
        const auto metrics = [&]() -> LagrangianMetrics {
          switch (interpolation_strategy) {
            case CallbackInterpolationStrategy::nearest_time:
              return (index_alpha.second > 0.5)
                       ? static_cast<LagrangianMetrics>(termMetricsArray[index_alpha.first])
                       : static_cast<LagrangianMetrics>(termMetricsArray[index_alpha.first + 1]);
            case CallbackInterpolationStrategy::linear_interpolation:
              return LinearInterpolation::interpolate(index_alpha, termMetricsArray);
            default:
              throw std::runtime_error(
                "[create_lagrangian_cb] This CallbackInterpolationStrategy is not "
                "implemented!");
          }
        }();
        metrics_publishers[i]->publish(
          ros_msg_conversions::create_lagrangian_metrics_msg(t, metrics));
      }
    }
  };
}

template <class NodeT>
SolverObserver::multiplier_callback_t create_multiplier_cb(
  std::weak_ptr<NodeT> nodeptr, const scalar_array_t & observing_time_points,
  const std::vector<std::string> & topic_names,
  CallbackInterpolationStrategy interpolation_strategy)
{
  if (observing_time_points.size() != topic_names.size()) {
    throw std::runtime_error(
      "[create_multiplier_cb] For each observing time points, you should provide a unique "
      "topic name!");
  }
  std::vector<rclcpp::Publisher<MultiplierMsg>::SharedPtr> multiplier_publishers;
  if (auto _node = nodeptr.lock()) {
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params =
      _node->get_node_parameters_interface();
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
      _node->get_node_topics_interface();
    for (const auto & name : topic_names) {
      multiplier_publishers.push_back(rclcpp::create_publisher<MultiplierMsg>(
        node_params, node_topics, name, rclcpp::ParametersQoS()));
    }
  } else {
    throw std::runtime_error("[create_multiplier_cb] The node is not available!");
  }

  // note that we need to copy the publishers as the local ones will go out of scope. Good news is that ROS publisher
  // behaves like std::sharted_ptr ("Once all copies of a specific Publisher go out of scope, any subscriber status callbacks
  // associated with that handle will stop being called.")
  return [=](
           const scalar_array_t & time_trajectory,
           const std::vector<MultiplierConstRef> & term_multiplier_array) {
    if (!time_trajectory.empty()) {
      for (size_t i = 0; i < observing_time_points.size(); i++) {
        const auto t = time_trajectory.front() + observing_time_points[i];
        const auto index_alpha = LinearInterpolation::timeSegment(t, time_trajectory);
        const auto multiplier = [&]() -> Multiplier {
          switch (interpolation_strategy) {
            case CallbackInterpolationStrategy::nearest_time:
              return (index_alpha.second > 0.5)
                       ? static_cast<Multiplier>(term_multiplier_array[index_alpha.first])
                       : static_cast<Multiplier>(term_multiplier_array[index_alpha.first + 1]);
            case CallbackInterpolationStrategy::linear_interpolation:
              return LinearInterpolation::interpolate(index_alpha, term_multiplier_array);
            default:
              throw std::runtime_error(
                "[create_multiplier_cb] This CallbackInterpolationStrategy is not "
                "implemented!");
          }
        }();
        multiplier_publishers[i]->publish(
          ros_msg_conversions::create_multiplier_msg(t, multiplier));
      }
    }
  };
}

}  // namespace ros
}  // namespace ocs2
