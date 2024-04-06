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

#include "ocs2_core/thread_support/BufferedValue.hpp"
#include "ocs2_oc/synchronized_module/ReferenceManagerInterface.hpp"

namespace ocs2
{

/**
 * Implements the reference manager with a thread-safe buffer for setting and getting the references.
 * A protected virtual interface is provided to modify the references before each solver run.
 */
class ReferenceManager : public ReferenceManagerInterface
{
public:
  explicit ReferenceManager(
    TargetTrajectories initialTargetTrajectories = TargetTrajectories(),
    ModeSchedule initialModeSchedule = ModeSchedule());

  ~ReferenceManager() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t & initState) override;

  const ModeSchedule & getModeSchedule() const override { return mode_schedule_.get(); }
  void setModeSchedule(const ModeSchedule & mode_schedule) override
  {
    mode_schedule_.setBuffer(mode_schedule);
  }
  void setModeSchedule(ModeSchedule && mode_schedule) override
  {
    mode_schedule_.setBuffer(std::move(mode_schedule));
  }

  const TargetTrajectories & getTargetTrajectories() const override
  {
    return targetTrajectories_.get();
  }
  void setTargetTrajectories(const TargetTrajectories & targetTrajectories) override
  {
    return targetTrajectories_.setBuffer(targetTrajectories);
  }
  void setTargetTrajectories(TargetTrajectories && targetTrajectories) override
  {
    return targetTrajectories_.setBuffer(std::move(targetTrajectories));
  }

protected:
  /**
   * Modifies the active ModeSchedule and TargetTrajectories.
   *
   * @param [in] initTime : Start time of the optimization horizon.
   * @param [in] finalTime : Final time of the optimization horizon.
   * @param [in] initState : State at the start of the optimization horizon.
   * @param [in, out] targetTrajectories : The updated TargetTrajectories. If setTargetTrajectories() has been called before,
   * TargetTrajectories is already updated by the set value.
   * @param [in, out] mode_schedule : The updated ModeSchedule. If setModeSchedule() has been called before, mode_schedule is
   * already updated by the set value.
   */
  virtual void modifyReferences(
    scalar_t initTime, scalar_t finalTime, const vector_t & initState,
    TargetTrajectories & targetTrajectories, ModeSchedule & mode_schedule)
  {
  }

private:
  BufferedValue<ModeSchedule> mode_schedule_;
  BufferedValue<TargetTrajectories> targetTrajectories_;
};

}  // namespace ocs2
