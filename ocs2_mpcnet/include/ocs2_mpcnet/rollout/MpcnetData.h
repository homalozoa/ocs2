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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_mpcnet/MpcnetDefinitionBase.h"

namespace ocs2 {
namespace mpcnet {

struct DataPoint {
  size_t mode;
  scalar_t t;
  vector_t x;
  vector_t u;
  vector_t generalizedTime;
  vector_t relativeState;
  matrix_t inputTransformation;
  ScalarFunctionQuadraticApproximation hamiltonian;
};
using data_point_t = DataPoint;
using data_array_t = std::vector<data_point_t>;
using data_ptr_t = std::unique_ptr<data_array_t>;

/**
 * Get a data point.
 * @param [in] mpc : The MPC with a pointer to the underlying solver.
 * @param [in] mpcnetDefinition : The MPC-Net definitions.
 * @param [in] deviation : The state deviation from the nominal state where to get the data point from.
 * @return A data point.
 */
inline data_point_t getDataPoint(MPC_BASE& mpc, MpcnetDefinitionBase& mpcnetDefinition, const vector_t& deviation) {
  data_point_t dataPoint;
  const auto& referenceManager = mpc.getSolverPtr()->getReferenceManager();
  const auto primalSolution = mpc.getSolverPtr()->primalSolution(mpc.getSolverPtr()->getFinalTime());
  dataPoint.t = primalSolution.timeTrajectory_.front();
  dataPoint.x = primalSolution.stateTrajectory_.front() + deviation;
  dataPoint.u = primalSolution.controllerPtr_->computeInput(dataPoint.t, dataPoint.x);
  dataPoint.mode = primalSolution.modeSchedule_.modeAtTime(dataPoint.t);
  dataPoint.generalizedTime = mpcnetDefinition.getGeneralizedTime(dataPoint.t, referenceManager.getModeSchedule());
  dataPoint.relativeState = mpcnetDefinition.getRelativeState(dataPoint.t, dataPoint.x, referenceManager.getTargetTrajectories());
  dataPoint.inputTransformation = mpcnetDefinition.getInputTransformation(dataPoint.t, dataPoint.x);
  dataPoint.hamiltonian = mpc.getSolverPtr()->getHamiltonian(dataPoint.t, dataPoint.x, dataPoint.u);
  return dataPoint;
}

}  // namespace mpcnet
}  // namespace ocs2
