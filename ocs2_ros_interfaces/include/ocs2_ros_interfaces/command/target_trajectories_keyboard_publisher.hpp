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

#include <functional>
#include <memory>
#include <mutex>

#include "ocs2_mpc/SystemObservation.hpp"
#include "ocs2_ros_interfaces/command/target_trajectories_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ocs2
{

/**
 * This class lets the user to insert robot command form command line.
 */
class TargetTrajectoriesKeyboardPublisher final
{
public:
  using FuncCmdLineToTargetTrajs =
    std::function<TargetTrajectories(const vector_t &, const SystemObservation &)>;

  template <class NodePtrT>
  TargetTrajectoriesKeyboardPublisher(
    NodePtrT && nodeptr, const std::string & topic_prefix,
    const scalar_array_t & target_command_limits,
    FuncCmdLineToTargetTrajs func_cmdline_to_target_trajs,
    const rclcpp::QoS & qos = rclcpp::ParametersQoS())
  {
    target_command_limits_ =
      Eigen::Map<const vector_t>(target_command_limits.data(), target_command_limits.size());
    func_cmdline_to_target_trajs_ = std::move(func_cmdline_to_target_trajs);
    if (auto _node = nodeptr->lock()) {
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters =
        _node->get_node_parameters_interface();
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
        _node->get_node_topics_interface();
      auto observation_cb = [this](const MpcObservationMsg::ConstSharedPtr & msg) {
        std::lock_guard<std::mutex> lock(latest_observation_mutex_);
        latest_observation_ = ros_msg_conversions::read_observation_msg(*msg);
      };
      observation_subscriber_ = rclcpp::create_subscription<MpcObservationMsg>(
        node_parameters, node_topics, topic_prefix + "_mpc_observation", qos,
        std::move(observation_cb));
    } else {
      throw std::runtime_error("Parent node is not available");
    }
    target_trajs_publisher_ = std::make_unique<TargetTrajPublisher>(nodeptr, topic_prefix);
  }
  size_t target_command_size() const { return target_command_limits_.size(); }
  void publish_keyboard_command(const std::string & cmd_msg = "Enter command, separated by space");

private:
  vector_t get_command_line();
  const vector_t target_command_limits_;
  FuncCmdLineToTargetTrajs func_cmdline_to_target_trajs_;
  std::unique_ptr<TargetTrajPublisher> target_trajs_publisher_;
  rclcpp::Subscription<MpcObservationMsg> observation_subscriber_;
  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;
};

}  // namespace ocs2
