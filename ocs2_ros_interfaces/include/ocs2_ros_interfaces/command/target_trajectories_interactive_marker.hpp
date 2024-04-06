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

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "ocs2_mpc/SystemObservation.hpp"
#include "ocs2_ros_interfaces/command/target_trajectories_publisher.hpp"

namespace ocs2
{

using FuncGoalPoseToTargetTrajs = std::function<TargetTrajectories(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
  const SystemObservation & observation)>;

class TargetTrajInteractiveMarker final
{
public:
  template <class NodePtrT>
  explicit TargetTrajInteractiveMarker(
    NodePtrT && nodeptr, const std::string & topic_prefix,
    FuncGoalPoseToTargetTrajs func_goal_pose_to_traj,
    const rclcpp::QoS & qos = rclcpp::ParametersQoS())
  {
    func_goal_pose_to_traj_ = std::move(func_goal_pose_to_traj);
    if (auto _node = nodeptr.lock()) {
      clock_interface_ = _node->get_clock_interface();
      server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        _node->get_node_base_interface()->get_namespace());
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
    traj_publisher_ = std::make_unique<TargetTrajPublisher>(nodeptr, topic_prefix);
    menu_handler_.insert(
      "Send target pose",
      std::bind(&TargetTrajInteractiveMarker::process_feedback, this, std::placeholders::_1));
    auto marker = create_interactive_marker();
    server_->insert(marker);
    menu_handler_.apply(*server_, marker.name);
    server_->applyChanges();
  }
  ~TargetTrajInteractiveMarker() = default;

private:
  InteractiveMarkerMsg create_interactive_marker() const;
  void process_feedback(const InteractiveMarkerFeedbackMsg::ConstSharedPtr & feedback);

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  interactive_markers::MenuHandler menu_handler_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_ = nullptr;
  FuncGoalPoseToTargetTrajs func_goal_pose_to_traj_;
  std::unique_ptr<TargetTrajPublisher> traj_publisher_ = nullptr;
  rclcpp::Subscription<MpcObservationMsg>::SharedPtr observation_subscriber_;
  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;
};

}  // namespace ocs2
