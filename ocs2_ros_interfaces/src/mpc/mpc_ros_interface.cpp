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
#include "ocs2_ros_interfaces/mpc/mpc_ros_interface.hpp"

#include <string>

#include "ocs2_ros_interfaces/common/ros_logging.hpp"

namespace ocs2
{

template <class NodePtrT>
MpcRosInterface::MpcRosInterface(
  NodePtrT && nodeptr, MpcBase & mpc, const std::string & topic_prefix,
  const std::string & logger_tag, const rclcpp::QoS qos)
: logger_tag_(std::string("[") + logger_tag + std::string("] "))
{
  mpc_ptr_ = std::make_unique<MpcBase>(mpc);
  publisher_primal_solution_ptr_ = std::make_unique<PrimalSolution>();
  publisher_primal_solution_ptr_ = std::make_unique<PrimalSolution>();
  publisher_command_ptr_ = std::make_unique<CommandData>();
  buffer_performance_indices_ptr_ = std::make_unique<PerformanceIndex>();
  publisher_performance_indices_ptr_ = std::make_unique<PerformanceIndex>();
  if (auto _node = nodeptr.lock()) {
    logger_ = _node->get_node_logging_interface();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base =
      _node->get_node_base_interface();
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters =
      _node->get_node_parameters_interface();
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics =
      _node->get_node_topics_interface();
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services =
      _node->get_node_services_interface();
    mpc_observation_sub_ = rclcpp::create_subscription<MpcObservationMsg>(
      node_parameters, node_topics, topic_prefix + "_mpc_observation", qos,
      std::bind(&MpcRosInterface::mpc_observation_cb, this, std::placeholders::_1));
    mpc_policy_pub_ = rclcpp::create_publisher<MpcFlattenedControllerMsg>(
      node_parameters, node_topics, topic_prefix + "_mpc_policy", qos);
    mpc_reset_srv_ = rclcpp::create_service<ocs2_msgs::srv::Reset>(
      node_base, node_services, topic_prefix + "_reset",
      std::bind(
        &MpcRosInterface::reset_mpc_cb, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3),
      rmw_qos_profile_services_default, node_base->get_default_callback_group());
  } else {
    throw std::runtime_error("Parent node is not available");
  }
}

void MpcRosInterface::reset_mpc_interface(TargetTrajectories && init_trajs)
{
  std::lock_guard<std::mutex> resetLock(reset_mutex_);
  mpc_ptr_->reset();
  mpc_ptr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(init_trajs));
  mpc_timer_.reset();
  reset_requested_ever_ = true;
  terminate_thread_ = false;
  ready_to_publish_ = false;
}

void MpcRosInterface::shutdown_interface()
{
  ros_logging::info(logger_tag_, std::string("Shutting down workers ..."));
  std::unique_lock<std::mutex> lk(publisher_mutex_);
  terminate_thread_ = true;
  lk.unlock();
  msg_ready_.notify_all();
  if (publisher_worker_.joinable()) {
    publisher_worker_.join();
  }
  ros_logging::info(logger_tag_, std::string("All workers are shut down."));
}

void MpcRosInterface::reset_mpc_cb(
  const std::shared_ptr<rmw_request_id_t> request_header, ResetSrv::Request::ConstSharedPtr & req,
  ResetSrv::Response::SharedPtr res)
{
  if (static_cast<bool>(req->reset)) {
    auto targetTrajectories =
      ros_msg_conversions::read_target_trajectories_msg(req->target_trajectories);
    reset_mpc_interface(std::move(targetTrajectories));

    std::cerr << "\n#####################################################"
              << "\n#####################################################"
              << "\n#################  MPC is reset.  ###################"
              << "\n#####################################################"
              << "\n#####################################################\n";
    res->done = static_cast<bool>(true);

  } else {
    ros_logging::warn(logger_tag_, std::string("[MpcRosInterface] Reset request failed!"));
    res->done = static_cast<bool>(false);
  }
}

MpcFlattenedControllerMsg MpcRosInterface::create_mpc_policy_msg(
  const PrimalSolution & primalSolution, const CommandData & commandData,
  const PerformanceIndex & performanceIndices)
{
  MpcFlattenedControllerMsg mpc_policy_msg;

  mpc_policy_msg.init_observation =
    ros_msg_conversions::create_observation_msg(commandData.mpcInitObservation_);
  mpc_policy_msg.plan_target_trajectories =
    ros_msg_conversions::create_target_trajectories_msg(commandData.mpcTargetTrajectories_);
  mpc_policy_msg.mode_schedule =
    ros_msg_conversions::create_mode_schedule_msg(primalSolution.mode_schedule_);
  mpc_policy_msg.performance_indices = ros_msg_conversions::create_performance_indices_msg(
    commandData.mpcInitObservation_.time, performanceIndices);

  switch (primalSolution.controllerPtr_->getType()) {
    case ControllerType::FEEDFORWARD:
      mpc_policy_msg.controller_type = MpcFlattenedControllerMsg::CONTROLLER_FEEDFORWARD;
      break;
    case ControllerType::LINEAR:
      mpc_policy_msg.controller_type = MpcFlattenedControllerMsg::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error("MpcRosInterface::creatempc_policy_msg: Unknown ControllerType");
  }

  // maximum length of the message
  const size_t N = primalSolution.timeTrajectory_.size();

  mpc_policy_msg.time_trajectory.clear();
  mpc_policy_msg.time_trajectory.reserve(N);
  mpc_policy_msg.state_trajectory.clear();
  mpc_policy_msg.state_trajectory.reserve(N);
  mpc_policy_msg.data.clear();
  mpc_policy_msg.data.reserve(N);
  mpc_policy_msg.post_event_indices.clear();
  mpc_policy_msg.post_event_indices.reserve(primalSolution.postEventIndices_.size());

  // time
  for (auto t : primalSolution.timeTrajectory_) {
    mpc_policy_msg.time_trajectory.emplace_back(t);
  }

  // post-event indices
  for (auto ind : primalSolution.postEventIndices_) {
    mpc_policy_msg.post_event_indices.emplace_back(static_cast<uint16_t>(ind));
  }

  // state
  for (size_t k = 0; k < N; k++) {
    MpcStateMsg mpc_state;
    mpc_state.value.resize(primalSolution.stateTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++) {
      mpc_state.value[j] = primalSolution.stateTrajectory_[k](j);
    }
    mpc_policy_msg.state_trajectory.emplace_back(mpc_state);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++) {
    MpcInputMsg mpc_input;
    mpc_input.value.resize(primalSolution.inputTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) {
      mpc_input.value[j] = primalSolution.inputTrajectory_[k](j);
    }
    mpc_policy_msg.input_trajectory.emplace_back(mpc_input);
  }  // end of k loop

  // controller
  scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float> *> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);
  for (auto t : primalSolution.timeTrajectory_) {
    mpc_policy_msg.data.emplace_back(ControllerDataMsg());

    policyMsgDataPointers.push_back(&mpc_policy_msg.data.back().data);
    timeTrajectoryTruncated.push_back(t);
  }  // end of k loop

  // serialize controller into data buffer
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  return mpc_policy_msg;
}

void MpcRosInterface::publisher_worker()
{
  while (!terminate_thread_) {
    std::unique_lock<std::mutex> lk(publisher_mutex_);
    msg_ready_.wait(lk, [&] { return (ready_to_publish_ || terminate_thread_); });
    if (terminate_thread_) {
      break;
    }
    std::lock_guard<std::mutex> policy_buffer_lock(buffer_mutex_);
    publisher_command_ptr_.swap(buffer_command_ptr_);
    publisher_primal_solution_ptr_.swap(buffer_primal_solution_ptr_);
    publisher_performance_indices_ptr_.swap(buffer_performance_indices_ptr_);
    MpcFlattenedControllerMsg mpc_policy_msg = create_mpc_policy_msg(
      *publisher_primal_solution_ptr_, *publisher_command_ptr_,
      *publisher_performance_indices_ptr_);

    mpc_policy_pub_->publish(mpc_policy_msg);
    ready_to_publish_ = false;
    lk.unlock();
    msg_ready_.notify_one();
  }
}

void MpcRosInterface::copy_to_buffer(const SystemObservation & mpcInitObservation)
{
  // buffer policy mutex
  std::lock_guard<std::mutex> policyBufferLock(buffer_mutex_);

  // get solution
  scalar_t finalTime = mpcInitObservation.time + mpc_ptr_->settings().solutionTimeWindow_;
  if (mpc_ptr_->settings().solutionTimeWindow_ < 0) {
    finalTime = mpc_ptr_->getSolverPtr()->getFinalTime();
  }
  mpc_ptr_->getSolverPtr()->getPrimalSolution(finalTime, buffer_primal_solution_ptr_.get());

  // command
  buffer_command_ptr_->mpcInitObservation_ = mpcInitObservation;
  buffer_command_ptr_->mpcTargetTrajectories_ =
    mpc_ptr_->getSolverPtr()->getReferenceManager().getTargetTrajectories();

  // performance indices
  *buffer_performance_indices_ptr_ = mpc_ptr_->getSolverPtr()->getPerformanceIndeces();
}

void MpcRosInterface::mpc_observation_cb(const MpcObservationMsg::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> resetLock(reset_mutex_);

  if (!reset_requested_ever_.load()) {
    ros_logging::warn(
      logger_tag_, std::string("MPC should be reset first. Either call MpcRosInterface::reset() or "
                               "use the reset service."));
    return;
  }

  // current time, state, input, and subsystem
  const auto current_observation = ros_msg_conversions::read_observation_msg(*msg);

  // measure the delay in running MPC
  mpc_timer_.startTimer();

  // run MPC
  bool controllerIsUpdated = mpc_ptr_->run(current_observation.time, current_observation.state);
  if (!controllerIsUpdated) {
    return;
  }
  copy_to_buffer(current_observation);

  // measure the delay for sending ROS messages
  mpc_timer_.endTimer();

  // check MPC delay and solution window compatibility
  scalar_t timeWindow = mpc_ptr_->settings().solutionTimeWindow_;
  if (mpc_ptr_->settings().solutionTimeWindow_ < 0) {
    timeWindow = mpc_ptr_->getSolverPtr()->getFinalTime() - current_observation.time;
  }
  if (timeWindow < 2.0 * mpc_timer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!\n";
  }

  // display
  if (mpc_ptr_->settings().debugPrint_) {
    std::cerr << '\n';
    std::cerr << "\n### MPC_ROS Benchmarking";
    std::cerr << "\n###   Maximum : " << mpc_timer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpc_timer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpc_timer_.getLastIntervalInMilliseconds() << "[ms]."
              << std::endl;
  }

  std::unique_lock<std::mutex> lk(publisher_mutex_);
  ready_to_publish_ = true;
  lk.unlock();
  msg_ready_.notify_one();
}
}  // namespace ocs2
