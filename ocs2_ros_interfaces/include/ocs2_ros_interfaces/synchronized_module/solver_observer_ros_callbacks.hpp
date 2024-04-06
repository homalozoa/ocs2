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

#pragma once

#include <memory>

#include "ocs2_oc/synchronized_module/SolverObserver.hpp"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ocs2
{
namespace ros
{

enum class CallbackInterpolationStrategy {
  nearest_time,
  linear_interpolation,
};

template <class NodeT>
SolverObserver::constraint_callback_t create_constraint_cb(
  std::weak_ptr<NodeT> nodeptr, const scalar_array_t & observing_time_points,
  const std::vector<std::string> & topic_names,
  CallbackInterpolationStrategy interpolation_strategy =
    CallbackInterpolationStrategy::nearest_time);

template <class NodeT>
SolverObserver::lagrangian_callback_t create_lagrangian_cb(
  std::weak_ptr<NodeT> nodeptr, const scalar_array_t & observing_time_points,
  const std::vector<std::string> & topic_names,
  CallbackInterpolationStrategy interpolation_strategy =
    CallbackInterpolationStrategy::nearest_time);

template <class NodeT>
SolverObserver::multiplier_callback_t create_multiplier_cb(
  std::weak_ptr<NodeT> nodeptr, const scalar_array_t & observing_time_points,
  const std::vector<std::string> & topic_names,
  CallbackInterpolationStrategy interpolation_strategy =
    CallbackInterpolationStrategy::nearest_time);

}  // namespace ros
}  // namespace ocs2
