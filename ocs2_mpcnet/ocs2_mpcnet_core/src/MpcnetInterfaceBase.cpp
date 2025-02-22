/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include "ocs2_mpcnet_core/MpcnetInterfaceBase.h"

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetInterfaceBase::startDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                              size_t nSamples, const matrix_t& samplingCovariance,
                                              const std::vector<SystemObservation>& initialObservations,
                                              const std::vector<ModeSchedule>& mode_schedules,
                                              const std::vector<TargetTrajectories>& targetTrajectories) {
  mpcnetRolloutManagerPtr_->startDataGeneration(alpha, policyFilePath, timeStep, dataDecimation, nSamples, samplingCovariance,
                                                initialObservations, mode_schedules, targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetInterfaceBase::isDataGenerationDone() {
  return mpcnetRolloutManagerPtr_->isDataGenerationDone();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
data_array_t MpcnetInterfaceBase::getGeneratedData() {
  return mpcnetRolloutManagerPtr_->getGeneratedData();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetInterfaceBase::startPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                                const std::vector<SystemObservation>& initialObservations,
                                                const std::vector<ModeSchedule>& mode_schedules,
                                                const std::vector<TargetTrajectories>& targetTrajectories) {
  mpcnetRolloutManagerPtr_->startPolicyEvaluation(alpha, policyFilePath, timeStep, initialObservations, mode_schedules, targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetInterfaceBase::isPolicyEvaluationDone() {
  return mpcnetRolloutManagerPtr_->isPolicyEvaluationDone();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_array_t MpcnetInterfaceBase::getComputedMetrics() {
  return mpcnetRolloutManagerPtr_->getComputedMetrics();
}

}  // namespace mpcnet
}  // namespace ocs2
