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

#include <ocs2_core/Types.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>

namespace ocs2 {
namespace multiple_shooting {

/** Struct to contain the result and logging data of the stepsize computation */
struct StepInfo {
  enum class StepType { UNKNOWN, CONSTRAINT, DUAL, COST, ZERO };

  // Step size and type
  scalar_t stepSize = 0.0;
  StepType stepType = StepType::UNKNOWN;

  // Step in primal variables
  scalar_t dx_norm = 0.0;  // norm of the state trajectory update
  scalar_t du_norm = 0.0;  // norm of the input trajectory update

  // Performance result after the step
  PerformanceIndex performanceAfterStep;
  scalar_t totalConstraintViolationAfterStep;  // constraint metric used in the line search
};

std::string toString(const StepInfo::StepType& stepType);

/** Different types of convergence */
enum class Convergence { FALSE, ITERATIONS, STEPSIZE, METRICS, PRIMAL };

std::string toString(const Convergence& convergence);

}  // namespace multiple_shooting
}  // namespace ocs2