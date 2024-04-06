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

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include "eigen3/Eigen/Dense"
#include "ocs2_core/Types.hpp"
#include "ocs2_core/control/ControllerBase.hpp"
#include "ocs2_core/misc/LinearInterpolation.hpp"
#include "ocs2_core/reference/ModeSchedule.hpp"
#include "ocs2_core/reference/TargetTrajectories.hpp"
#include "ocs2_mpc/CommandData.hpp"
#include "ocs2_mpc/MrtObserver.hpp"
#include "ocs2_mpc/SystemObservation.hpp"
#include "ocs2_oc/oc_data/PerformanceIndex.hpp"
#include "ocs2_oc/oc_data/PrimalSolution.hpp"
#include "ocs2_oc/rollout/RolloutBase.hpp"

namespace ocs2
{

/**
 * This class implements core MRT (Model Reference Tracking) functionality.
 * The responsibility of filling the buffer variables is left to the deriving classes.
 */
class MrtBase
{
public:
  MrtBase();

  virtual ~MrtBase() = default;

  /**
   * Resets the class to its instantiated state.
   */
  void reset();

  /**
   * Request the MPC node to reset. This method is a blocking method.
   *
   * @param [in] initTargetTrajectories: The initial desired cost trajectories.
   */
  virtual void reset_mpc_node(const TargetTrajectories & initTargetTrajectories) = 0;

  /**
   * Whether the initial MPC policy has been already received.
   */
  bool initial_policy_received() const { return policyReceivedEver_; }

  /**
   * @brief set_current_observation notifies MPC of a new state
   * @param observation: the current measurement to send to the MPC
   */
  virtual void set_current_observation(const SystemObservation & observation) = 0;

  /**
   * Gets a reference to the command data corresponding to the current policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to update_policy() must be synced by the user.
   *
   * @return a constant reference to command data.
   */
  const CommandData & get_command() const;

  /**
   * Gets a reference to the performance indices data corresponding to the current policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to update_policy() must be synced by the user.
   *
   * @return a constant reference to performance indices data.
   */
  const PerformanceIndex & get_performance_indices() const;

  /**
   * Gets a reference to current optimized policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to update_policy() must be synced by the user.
   *
   * @return constant reference to the policy data.
   */
  const PrimalSolution & get_policy() const;

  /**
   * @brief Initializes rollout class to roll out a feedback policy
   * @param rolloutPtr: The rollout object to be used
   */
  void init_rollout(const RolloutBase * rolloutPtr);

  /**
   * @brief Evaluates the controller
   *
   * @param [in] currentTime: the query time.
   * @param [in] currentState: the query state.
   * @param [out] mpcState: the current nominal state of MPC.
   * @param [out] mpcInput: the optimized control input.
   * @param [out] mode: the active mode.
   */
  void evaluate_policy(
    scalar_t currentTime, const vector_t & currentState, vector_t & mpcState, vector_t & mpcInput,
    size_t & mode);

  /**
   * @brief Rolls out the control policy from the current time and state to get the next state and input using the MPC policy.
   *
   * @param [in] currentTime: start time of the rollout.
   * @param [in] currentState: state to start rollout from.
   * @param [in] timeStep: duration of the forward rollout.
   * @param [out] mpcState: the new forwarded state of MPC.
   * @param [out] mpcInput: the new control input of MPC.
   * @param [out] mode: the active mode.
   */
  void rollout_policy(
    scalar_t currentTime, const vector_t & currentState, const scalar_t & timeStep,
    vector_t & mpcState, vector_t & mpcInput, size_t & mode);

  /**
   * Checks the data buffer for an update of the MPC policy. If a new policy
   * is available on the buffer this method will load it to the in-use policy.
   * This method also calls the modify_active_solution() method.
   *
   * @return True if the policy is updated.
   */
  bool update_policy();

  /**
   * @brief rolloutSet: Whether or not the internal rollout object has been set
   * @return True if a rollout object is available.
   */
  bool is_rollout_set() const { return rolloutPtr_ != nullptr; }

  /**
   * Adds an MRT observer to the policy update process
   */
  void add_mrt_observer(std::shared_ptr<MrtObserver> mrtObserver)
  {
    observerPtrArray_.push_back(std::move(mrtObserver));
  };

protected:
  void move_to_buffer(
    std::unique_ptr<CommandData> commandDataPtr, std::unique_ptr<PrimalSolution> primalSolutionPtr,
    std::unique_ptr<PerformanceIndex> performanceIndicesPtr);

private:
  /** Calls modify_active_solution on all mrt observers. This function is called while holding a policyBufferMutex lock */
  void modify_active_solution(const CommandData & command, PrimalSolution & primalSolution);

  /** Calls modify_buffered_solution on all mrt observers. This function is called while holding a policyBufferMutex lock */
  void modify_buffered_solution(
    const CommandData & commandBuffer, PrimalSolution & primalSolutionBuffer);

  // flags on state of the class
  std::atomic_bool policyReceivedEver_;
  bool newPolicyInBuffer_;  // whether a new policy is waiting to be swapped in

  // variables related to the MPC output
  std::unique_ptr<CommandData> activeCommandPtr_;
  std::unique_ptr<CommandData> bufferCommandPtr_;
  std::unique_ptr<PrimalSolution> activePrimalSolutionPtr_;
  std::unique_ptr<PrimalSolution> bufferPrimalSolutionPtr_;
  std::unique_ptr<PerformanceIndex> activePerformanceIndicesPtr_;
  std::unique_ptr<PerformanceIndex> bufferPerformanceIndicesPtr_;

  // thread safety
  mutable std::mutex bufferMutex_;  // for policy variables with the prefix (buffer*)
  const size_t mrtTrylockWarningThreshold_ = 5;
  size_t mrtTrylockWarningCount_;

  // variables needed for policy evaluation
  std::unique_ptr<RolloutBase> rolloutPtr_;

  std::vector<std::shared_ptr<MrtObserver>> observerPtrArray_;
};

}  // namespace ocs2
