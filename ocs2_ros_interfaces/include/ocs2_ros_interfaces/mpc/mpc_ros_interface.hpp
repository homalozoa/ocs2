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

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ocs2_core/control/FeedforwardController.hpp"
#include "ocs2_core/control/LinearController.hpp"
#include "ocs2_core/misc/Benchmark.hpp"
#include "ocs2_mpc/CommandData.hpp"
#include "ocs2_mpc/SystemObservation.hpp"
#include "ocs2_mpc/mpc_base.hpp"
#include "ocs2_msgs/msg/mode_schedule.hpp"
#include "ocs2_msgs/msg/mpc_flattened_controller.hpp"
#include "ocs2_msgs/msg/mpc_observation.hpp"
#include "ocs2_msgs/msg/mpc_target_trajectories.hpp"
#include "ocs2_msgs/srv/reset.hpp"
#include "ocs2_oc/oc_data/PrimalSolution.hpp"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ocs2
{
class MpcRosInterface
{
public:
  template <class NodePtrT>
  explicit MpcRosInterface(
    NodePtrT && nodeptr, MpcBase & mpc, const std::string & topic_prefix = "anonymousRobot",
    const std::string & logger_tag = "mpc_ros_interface",
    const rclcpp::QoS qos = rclcpp::ParametersQoS());
  ~MpcRosInterface() { shutdown_interface(); }
  void reset_mpc_interface(TargetTrajectories && init_trajs);
  void shutdown_interface();

private:
  static MpcFlattenedControllerMsg create_mpc_policy_msg(
    const PrimalSolution & primalSolution, const CommandData & commandData,
    const PerformanceIndex & performanceIndices);
  void publisher_worker();
  void copy_to_buffer(const SystemObservation & mpcInitObservation);
  void mpc_observation_cb(const MpcObservationMsg::ConstSharedPtr & msg);
  void reset_mpc_cb(
    const std::shared_ptr<rmw_request_id_t> request_header, ResetSrv::Request::ConstSharedPtr & req,
    ResetSrv::Response::SharedPtr res);

  std::unique_ptr<MpcBase> mpc_ptr_;
  rclcpp::Subscription<MpcObservationMsg>::SharedPtr mpc_observation_sub_;
  rclcpp::Publisher<MpcFlattenedControllerMsg>::SharedPtr mpc_policy_pub_;
  rclcpp::Service<ResetSrv>::SharedPtr mpc_reset_srv_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  std::string logger_tag_;

  std::unique_ptr<CommandData> buffer_command_ptr_;
  std::unique_ptr<CommandData> publisher_command_ptr_;
  std::unique_ptr<PrimalSolution> buffer_primal_solution_ptr_;
  std::unique_ptr<PrimalSolution> publisher_primal_solution_ptr_;
  std::unique_ptr<PerformanceIndex> buffer_performance_indices_ptr_;
  std::unique_ptr<PerformanceIndex> publisher_performance_indices_ptr_;
  mutable std::mutex buffer_mutex_;
  std::atomic_bool terminate_thread_{false};
  std::atomic_bool ready_to_publish_{false};
  std::thread publisher_worker_;
  std::mutex publisher_mutex_;
  std::condition_variable msg_ready_;
  benchmark::RepeatedTimer mpc_timer_;
  std::mutex reset_mutex_;
  std::atomic_bool reset_requested_ever_{false};
};

}  // namespace ocs2
