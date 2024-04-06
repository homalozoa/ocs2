/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include "ocs2_mpc/mrt_base.hpp"
#include "ocs2_msgs/srv/reset.hpp"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ocs2
{

class MrtRosInterface : public MrtBase
{
public:
  template <class NodePtrT>
  explicit MrtRosInterface(
    NodePtrT && nodeptr, const std::string & topic_prefix = "anonymousRobot",
    const rclcpp::QoS qos = rclcpp::ParametersQoS(),
    const std::string & logger_tag = "mrt_ros_interfaces");
  ~MrtRosInterface() noexcept { shutdown_interface(); }
  void reset_mpc_node(const TargetTrajectories & initTargetTrajectories) override;
  void shutdown_interface();
  void set_current_observation(const SystemObservation & currentObservation) override;

private:
  void mpc_policy_cb(const MpcFlattenedControllerMsg::ConstSharedPtr & msg);
  static void read_policy_msg(
    const MpcFlattenedControllerMsg & msg, CommandData & commandData,
    PrimalSolution & primalSolution, PerformanceIndex & performanceIndices);
  void publisher_worker_thread();

  std::string topic_prefix_;
  rclcpp::Publisher<MpcObservationMsg>::SharedPtr mpc_observation_pub_;
  rclcpp::Subscription<MpcFlattenedControllerMsg>::SharedPtr mpc_policy_sub_;
  rclcpp::Client<ResetSrv>::SharedPtr mpc_reset_client_;
  MpcObservationMsg mpc_observation_msg_;
  MpcObservationMsg mpc_observation_msg_buffer_;

  bool terminate_thread_;
  bool ready_to_publish_;
  std::thread publisher_worker_;
  std::mutex publisher_mutex_;
  std::condition_variable msg_ready_;
  std::string logger_tag_;
};

}  // namespace ocs2
