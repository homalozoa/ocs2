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

#include "ocs2_ros_interfaces/mrt/mrt_ros_interface.hpp"

#include "ocs2_core/control/FeedforwardController.hpp"
#include "ocs2_core/control/LinearController.hpp"
#include "ocs2_ros_interfaces/common/ros_logging.hpp"

namespace ocs2
{

template <class NodePtrT>
MrtRosInterface::MrtRosInterface(
  NodePtrT && nodeptr, const std::string & topic_prefix, const rclcpp::QoS qos,
  const std::string & logger_tag)
: logger_tag_(std::string("[") + logger_tag + std::string("]"))
{
  this->reset();
  terminate_thread_ = false;
  ready_to_publish_ = false;
  publisher_worker_ = std::thread(&MrtRosInterface::publisher_worker_thread, this);

  if (auto _node = nodeptr.lock()) {
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base =
      _node->get_node_base_interface();
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params =
      _node->get_node_parameters_interface();
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
      _node->get_node_topics_interface();
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph =
      _node->get_node_graph_interface();
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services =
      _node->get_node_services_interface();
    mpc_observation_msg_ = rclcpp::create_publisher<MpcObservationMsg>(
      node_params, node_topics, topic_prefix + "_mpc_observation", qos);
    mpc_policy_sub_ = rclcpp::create_subscription<MpcFlattenedControllerMsg>(
      node_params, node_topics, topic_prefix + "_mpc_policy", qos,
      std::bind(&MrtRosInterface::mpc_policy_cb, this, std::placeholders::_1));
    mpc_reset_client_ = rclcpp::create_client<ResetSrv>(
      node_base, node_graph, node_services, topic_prefix + "_reset",
      rmw_qos_profile_services_default, node_base->get_default_callback_group());
  }
}

void MrtRosInterface::reset_mpc_node(const TargetTrajectories & init_trajs)
{
  this->reset();
  ResetSrv::Request reset_req;
  reset_req.reset = static_cast<uint8_t>(true);
  reset_req.target_trajectories = ros_msg_conversions::create_target_trajectories_msg(init_trajs);

  while (!mpc_reset_client_->wait_for_service(std::chrono::seconds(5)) && rclcpp::ok()) {
    ros_logging::error(
      logger_tag_, std::string("Failed to call service to reset MPC, retrying..."));
  }

  mpc_reset_client_->async_send_request(
    std::make_shared<ResetSrv::Request>(reset_req),
    [this](rclcpp::Client<ResetSrv>::SharedFuture future) {
      auto response = future.get();
      if (response->done) {
        ros_logging::info(logger_tag_, std::string("MPC node has been reset."));
      } else {
        ros_logging::error(logger_tag_, std::string("Failed to reset MPC node."));
      }
    });
}

void MrtRosInterface::set_current_observation(const SystemObservation & observation_current)
{
  std::unique_lock<std::mutex> lk(publisher_mutex_);
  mpc_observation_msg_ = ros_msg_conversions::create_observation_msg(observation_current);
  ready_to_publish_ = true;
  lk.unlock();
  msg_ready_.notify_one();
}

void MrtRosInterface::publisher_worker_thread()
{
  while (!terminate_thread_) {
    std::unique_lock<std::mutex> lk(publisher_mutex_);
    msg_ready_.wait(lk, [&] { return (ready_to_publish_ || terminate_thread_); });
    if (terminate_thread_) {
      break;
    }
    mpc_observation_msg_buffer_ = std::move(mpc_observation_msg_);
    ready_to_publish_ = false;
    lk.unlock();
    msg_ready_.notify_one();
    mpc_observation_pub_->publish(mpc_observation_msg_buffer_);
  }
}

void MrtRosInterface::read_policy_msg(
  const MpcFlattenedControllerMsg & msg, CommandData & cmd_data, PrimalSolution & primal_solution,
  PerformanceIndex & performance_indices)
{
  cmd_data.mpcInitObservation_ = ros_msg_conversions::read_observation_msg(msg.init_observation);
  cmd_data.mpcTargetTrajectories_ =
    ros_msg_conversions::read_target_trajectories_msg(msg.plan_target_trajectories);
  performance_indices = ros_msg_conversions::read_performance_indices_msg(msg.performance_indices);

  const size_t N = msg.time_trajectory.size();
  if (N == 0) {
    throw std::runtime_error("[MrtRosInterface::read_policy_msg] controller message is empty!");
  }
  if (msg.state_trajectory.size() != N && msg.input_trajectory.size() != N) {
    throw std::runtime_error(
      "[MrtRosInterface::read_policy_msg] state and input trajectories must have same length!");
  }
  if (msg.data.size() != N) {
    throw std::runtime_error("[MrtRosInterface::read_policy_msg] Data has the wrong length!");
  }

  primal_solution.clear();

  primal_solution.mode_schedule_ = ros_msg_conversions::read_mode_schedule_msg(msg.mode_schedule);

  size_array_t stateDim(N);
  size_array_t inputDim(N);
  primal_solution.timeTrajectory_.reserve(N);
  primal_solution.stateTrajectory_.reserve(N);
  primal_solution.inputTrajectory_.reserve(N);
  for (size_t i = 0; i < N; i++) {
    stateDim[i] = msg.state_trajectory[i].value.size();
    inputDim[i] = msg.input_trajectory[i].value.size();
    primal_solution.timeTrajectory_.emplace_back(msg.time_trajectory[i]);
    primal_solution.stateTrajectory_.emplace_back(
      Eigen::Map<const Eigen::VectorXf>(msg.state_trajectory[i].value.data(), stateDim[i])
        .cast<scalar_t>());
    primal_solution.inputTrajectory_.emplace_back(
      Eigen::Map<const Eigen::VectorXf>(msg.input_trajectory[i].value.data(), inputDim[i])
        .cast<scalar_t>());
  }

  primal_solution.postEventIndices_.reserve(msg.post_event_indices.size());
  for (auto ind : msg.post_event_indices) {
    primal_solution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
  }

  std::vector<std::vector<float> const *> controllerDataPtrArray(N, nullptr);
  for (int i = 0; i < N; i++) {
    controllerDataPtrArray[i] = &(msg.data[i].data);
  }

  // instantiate the correct controller
  switch (msg.controller_type) {
    case MpcFlattenedControllerMsg::CONTROLLER_FEEDFORWARD: {
      auto controller =
        FeedforwardController::unFlatten(primal_solution.timeTrajectory_, controllerDataPtrArray);
      primal_solution.controllerPtr_.reset(new FeedforwardController(std::move(controller)));
      break;
    }
    case MpcFlattenedControllerMsg::CONTROLLER_LINEAR: {
      auto controller = LinearController::unFlatten(
        stateDim, inputDim, primal_solution.timeTrajectory_, controllerDataPtrArray);
      primal_solution.controllerPtr_.reset(new LinearController(std::move(controller)));
      break;
    }
    default:
      throw std::runtime_error("[MrtRosInterface::read_policy_msg] Unknown controllerType!");
  }
}

void MrtRosInterface::mpc_policy_cb(const MpcFlattenedControllerMsg::ConstSharedPtr & msg)
{
  auto cmd = std::make_unique<CommandData>();
  auto primal_solution = std::make_unique<PrimalSolution>();
  auto performance_indices = std::make_unique<PerformanceIndex>();
  read_policy_msg(*msg, *cmd, *primal_solution, *performance_indices);
  this->move_to_buffer(std::move(cmd), std::move(primal_solution), std::move(performance_indices));
}

void MrtRosInterface::shutdown_interface()
{
  std::unique_lock<std::mutex> lk(publisher_mutex_);
  terminate_thread_ = true;
  lk.unlock();
  msg_ready_.notify_all();

  if (publisher_worker_.joinable()) {
    publisher_worker_.join();
  }
}

}  // namespace ocs2
