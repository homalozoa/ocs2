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

#include "ocs2_mpc/mrt_base.hpp"

#include "ocs2_oc/rollout/TimeTriggeredRollout.hpp"

namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MrtBase::MrtBase() { reset(); }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::reset()
{
  std::lock_guard<std::mutex> lock(bufferMutex_);

  policyReceivedEver_ = false;
  newPolicyInBuffer_ = false;
  mrtTrylockWarningCount_ = 0;

  activeCommandPtr_.reset();
  bufferCommandPtr_.reset();
  activePrimalSolutionPtr_.reset();
  bufferPrimalSolutionPtr_.reset();
  activePerformanceIndicesPtr_.reset();
  bufferPerformanceIndicesPtr_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const CommandData & MrtBase::get_command() const
{
  if (activeCommandPtr_ != nullptr) {
    return *activeCommandPtr_;
  } else {
    throw std::runtime_error("[MrtBase::get_command] update_policy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PrimalSolution & MrtBase::get_policy() const
{
  if (activePrimalSolutionPtr_ != nullptr) {
    return *activePrimalSolutionPtr_;
  } else {
    throw std::runtime_error("[MrtBase::get_policy] update_policy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PerformanceIndex & MrtBase::get_performance_indices() const
{
  if (activePerformanceIndicesPtr_ != nullptr) {
    return *activePerformanceIndicesPtr_;
  } else {
    throw std::runtime_error(
      "[MrtBase::get_performance_indices] update_policy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::init_rollout(const RolloutBase * rolloutPtr)
{
  rolloutPtr_.reset(rolloutPtr->clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::evaluate_policy(
  scalar_t currentTime, const vector_t & currentState, vector_t & mpcState, vector_t & mpcInput,
  size_t & mode)
{
  if (activePrimalSolutionPtr_ == nullptr) {
    throw std::runtime_error("[MrtBase::evaluate_policy] update_policy() should be called first!");
  }

  if (currentTime > activePrimalSolutionPtr_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: "
              << std::to_string(currentTime) << ">"
              << std::to_string(activePrimalSolutionPtr_->timeTrajectory_.back()) << "\n";
  }

  mpcInput = activePrimalSolutionPtr_->controllerPtr_->computeInput(currentTime, currentState);
  mpcState = LinearInterpolation::interpolate(
    currentTime, activePrimalSolutionPtr_->timeTrajectory_,
    activePrimalSolutionPtr_->stateTrajectory_);

  mode = activePrimalSolutionPtr_->mode_schedule_.modeAtTime(currentTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::rollout_policy(
  scalar_t currentTime, const vector_t & currentState, const scalar_t & timeStep,
  vector_t & mpcState, vector_t & mpcInput, size_t & mode)
{
  if (rolloutPtr_ == nullptr) {
    throw std::runtime_error(
      "[MrtBase::rollout_policy] rollout class is not set! Use init_rollout() to initialize it!");
  }

  if (activePrimalSolutionPtr_ == nullptr) {
    throw std::runtime_error("[MrtBase::rollout_policy] update_policy() should be called first!");
  }

  if (currentTime > activePrimalSolutionPtr_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: "
              << std::to_string(currentTime) << ">"
              << std::to_string(activePrimalSolutionPtr_->timeTrajectory_.back()) << "\n";
  }

  // perform a rollout
  scalar_array_t timeTrajectory;
  size_array_t postEventIndicesStock;
  vector_array_t stateTrajectory, inputTrajectory;
  const scalar_t finalTime = currentTime + timeStep;
  rolloutPtr_->run(
    currentTime, currentState, finalTime, activePrimalSolutionPtr_->controllerPtr_.get(),
    activePrimalSolutionPtr_->mode_schedule_, timeTrajectory, postEventIndicesStock, stateTrajectory,
    inputTrajectory);

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  mode = activePrimalSolutionPtr_->mode_schedule_.modeAtTime(finalTime);
}

bool MrtBase::update_policy()
{
  std::unique_lock<std::mutex> lock(bufferMutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    mrtTrylockWarningCount_ = 0;
    if (newPolicyInBuffer_) {
      // update the active solution from buffer
      activeCommandPtr_.swap(bufferCommandPtr_);
      activePrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
      activePerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
      newPolicyInBuffer_ = false;  // make sure we don't swap in the old policy again

      modify_active_solution(*activeCommandPtr_, *activePrimalSolutionPtr_);
      return true;
    } else {
      return false;  // No policy update: the buffer contains nothing new.
    }
  } else {
    ++mrtTrylockWarningCount_;
    if (mrtTrylockWarningCount_ > mrtTrylockWarningThreshold_) {
      std::cerr << "[MrtBase::update_policy] failed to lock the policyBufferMutex for "
                << mrtTrylockWarningCount_ << " consecutive times.\n";
    }
    return false;  // No policy update: the lock could not be acquired.
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::move_to_buffer(
  std::unique_ptr<CommandData> commandDataPtr, std::unique_ptr<PrimalSolution> primalSolutionPtr,
  std::unique_ptr<PerformanceIndex> performanceIndicesPtr)
{
  if (commandDataPtr == nullptr) {
    throw std::runtime_error("[MrtBase::move_to_buffer] commandDataPtr cannot be a null pointer!");
  }

  if (primalSolutionPtr == nullptr) {
    throw std::runtime_error(
      "[MrtBase::move_to_buffer] primalSolutionPtr cannot be a null pointer!");
  }

  if (performanceIndicesPtr == nullptr) {
    throw std::runtime_error(
      "[MrtBase::move_to_buffer] performanceIndicesPtr cannot be a null pointer!");
  }

  std::lock_guard<std::mutex> lk(bufferMutex_);
  // use swap such that the old objects are destroyed after releasing the lock.
  bufferCommandPtr_.swap(commandDataPtr);
  bufferPrimalSolutionPtr_.swap(primalSolutionPtr);
  bufferPerformanceIndicesPtr_.swap(performanceIndicesPtr);

  // allow user to modify the buffer
  modify_buffered_solution(*bufferCommandPtr_, *bufferPrimalSolutionPtr_);

  newPolicyInBuffer_ = true;
  policyReceivedEver_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::modify_active_solution(const CommandData & command, PrimalSolution & primalSolution)
{
  for (auto & mrtObserver : observerPtrArray_) {
    if (mrtObserver != nullptr) {
      mrtObserver->modify_active_solution(command, primalSolution);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MrtBase::modify_buffered_solution(
  const CommandData & commandBuffer, PrimalSolution & primalSolutionBuffer)
{
  for (auto & mrtObserver : observerPtrArray_) {
    if (mrtObserver != nullptr) {
      mrtObserver->modify_buffered_solution(commandBuffer, primalSolutionBuffer);
    }
  }
}

}  // namespace ocs2
