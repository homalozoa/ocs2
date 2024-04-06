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
#include "ocs2_ros_interfaces/mrt/mrt_ros_dummy_loop.hpp"

#include <chrono>

namespace ocs2
{

MrtRosDummyLoop::MrtRosDummyLoop(
  MrtRosInterface & mrt, scalar_t mrt_desired_frequency, scalar_t mpc_desired_frequency)
: mrt_(mrt),
  mrt_desired_frequency_(mrt_desired_frequency),
  mpc_desired_frequency_(mpc_desired_frequency)
{
  if (mrt_desired_frequency_ < 0) {
    throw std::runtime_error("MRT loop frequency should be a positive number.");
  }

  if (mrt_desired_frequency_ > 0) {
    ros_logging::warn(
      logging_tag_, std::string("MPC loop is not realtime! For realtime setting, set "
                                "mpcDesiredFrequency to any negative number."));
  }
}

void MrtRosDummyLoop::run(
  const SystemObservation & initObservation, const TargetTrajectories & initTargetTrajectories)
{
  ros_logging::info(logging_tag_, std::string("Waiting for the initial policy ..."));

  // Reset MPC node
  mrt_.reset_mpc_node(initTargetTrajectories);

  // Wait for the initial policy
  rclcpp::Rate wait_rate(mrt_desired_frequency_);
  while (!mrt_.initial_policy_received() && rclcpp::ok()) {
    mrt_.set_current_observation(initObservation);
    wait_rate.sleep();
  }
  ros_logging::info(logging_tag_, std::string("Initial policy has been received."));

  // Pick simulation loop mode
  if (mpc_desired_frequency_ > 0.0) {
    synchronized_dummy_loop(initObservation, initTargetTrajectories);
  } else {
    realtime_dummy_loop(initObservation, initTargetTrajectories);
  }
}

void MrtRosDummyLoop::synchronized_dummy_loop(
  const SystemObservation & initObservation, const TargetTrajectories & initTargetTrajectories)
{
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio =
    std::max(static_cast<size_t>(mrt_desired_frequency_ / mpc_desired_frequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) {
    constexpr scalar_t tol = 0.1;  // policy must start within this fraction of dt
    return mrt_.update_policy() && std::abs(mrt_.get_policy().timeTrajectory_.front() - time) <
                                     (tol / mpc_desired_frequency_);
  };

  rclcpp::Rate wait_rate(mrt_desired_frequency_);
  while (rclcpp::ok()) {
    std::cout << "### Current time " << currentObservation.time << "\n";
    currentObservation = forward_simulation(currentObservation);
    modify_observation(currentObservation);
    if ((loopCounter + 1) % mpcUpdateRatio == 0) {
      mrt_.set_current_observation(currentObservation);
      std::cout << ">>> Observation is published at " << currentObservation.time << "\n";
    }
    for (auto & observer : observers_) {
      observer->update(currentObservation, mrt_.get_policy(), mrt_.get_command());
    }
    ++loopCounter;
    wait_rate.sleep();
  }
}

void MrtRosDummyLoop::realtime_dummy_loop(
  const SystemObservation & initObservation, const TargetTrajectories & initTargetTrajectories)
{
  // Loop variables
  SystemObservation currentObservation = initObservation;

  rclcpp::Rate wait_rate(mrt_desired_frequency_);
  while (rclcpp::ok()) {
    std::cout << "### Current time " << currentObservation.time << "\n";
    // Update the policy if a new on was received
    if (mrt_.update_policy()) {
      std::cout << "<<< New MPC policy starting at " << mrt_.get_policy().timeTrajectory_.front()
                << "\n";
    }
    currentObservation = forward_simulation(currentObservation);
    modify_observation(currentObservation);
    mrt_.set_current_observation(currentObservation);
    for (auto & observer : observers_) {
      observer->update(currentObservation, mrt_.get_policy(), mrt_.get_command());
    }
    wait_rate.sleep();
  }
}

SystemObservation MrtRosDummyLoop::forward_simulation(const SystemObservation & currentObservation)
{
  const scalar_t dt = 1.0 / mrt_desired_frequency_;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt;
  // If available, use the provided rollout as to integrate the dynamics.
  if (mrt_.is_rollout_set()) {
    mrt_.rollout_policy(
      currentObservation.time, currentObservation.state, dt, nextObservation.state,
      nextObservation.input, nextObservation.mode);
  } else {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
    mrt_.evaluate_policy(
      currentObservation.time + dt, currentObservation.state, nextObservation.state,
      nextObservation.input, nextObservation.mode);
  }

  return nextObservation;
}

}  // namespace ocs2
