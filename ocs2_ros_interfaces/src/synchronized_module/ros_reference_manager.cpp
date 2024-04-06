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

#include "ocs2_ros_interfaces/synchronized_module/ros_reference_manager.hpp"

#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"

namespace ocs2
{

template <class NodePtrT>
RosReferenceManager::RosReferenceManager(
  NodePtrT && nodeptr, std::string topic_prefix,
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr, const rclcpp::QoS qos)
: ReferenceManagerDecorator(std::move(referenceManagerPtr))
{
  auto mode_schedule_cb = [this](const ModeScheduleMsg::ConstSharedPtr & msg) {
    auto mode_schedule = ros_msg_conversions::read_mode_schedule_msg(*msg);
    referenceManagerPtr_->setModeSchedule(std::move(mode_schedule));
  };
  auto target_trajectories_cb = [this](const MpcTargetTrajectoriesMsg::ConstSharedPtr & msg) {
    auto target_trajectories = ros_msg_conversions::read_target_trajectories_msg(*msg);
    referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
  };

  if (auto _node = nodeptr.lock()) {
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params =
      _node->get_node_parameters_interface();
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
      _node->get_node_topics_interface();
    mode_schedule_sub_ = rclcpp::create_subscription<ModeScheduleMsg>(
      node_params, node_topics, topic_prefix + "_mode_schedule", qos,
      std::bind(mode_schedule_cb, std::placeholders::_1));
    target_trajs_sub_ = rclcpp::create_subscription<MpcTargetTrajectoriesMsg>(
      node_params, node_topics, topic_prefix + "_mpc_target", qos,
      std::bind(target_trajectories_cb, std::placeholders::_1));
  }
}

template <class ReferenceManagerType, class NodePtrT, class... Args>
std::unique_ptr<RosReferenceManager> RosReferenceManager::create(
  NodePtrT && nodeptr, const std::string & topicPrefix, Args &&... args)
{
  auto referenceManagerPtr = std::make_shared<ReferenceManagerType>(std::forward<Args>(args)...);
  return std::make_unique<RosReferenceManager>(
    nodeptr, topicPrefix, std::move(referenceManagerPtr));
}

}  // namespace ocs2
