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

#include "ocs2_mpc/mpc_base.hpp"
#include "ocs2_sqp/SqpSolver.hpp"

namespace ocs2
{

class SqpMpc final : public MpcBase
{
public:
  /**
   * Constructor
   *
   * @param mpcSettings : settings for the mpc wrapping of the solver. Do not use this for maxIterations and stepsize, use
   * multiple shooting SQP settings directly.
   * @param settings : settings for the multiple shooting SQP solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  SqpMpc(
    mpc::Settings mpcSettings, sqp::Settings settings,
    const OptimalControlProblem & optimalControlProblem, const Initializer & initializer)
  : MpcBase(std::move(mpcSettings))
  {
    solverPtr_.reset(new SqpSolver(std::move(settings), optimalControlProblem, initializer));
  };

  ~SqpMpc() override = default;

  SqpSolver * getSolverPtr() override { return solverPtr_.get(); }
  const SqpSolver * getSolverPtr() const override { return solverPtr_.get(); }

protected:
  void calculateController(
    scalar_t initTime, const vector_t & initState, scalar_t finalTime) override
  {
    if (settings().coldStart_) {
      solverPtr_->reset();
    }
    solverPtr_->run(initTime, initState, finalTime);
  }

private:
  std::unique_ptr<SqpSolver> solverPtr_;
};

}  // namespace ocs2
