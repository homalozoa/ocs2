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

#include "ocs2_mpc/CommandData.hpp"
#include "ocs2_oc/oc_data/PrimalSolution.hpp"

namespace ocs2
{

/**
 * This class allows modification of the MPC solution at two separate points of the MRT process.
 * Moreover, it can be used to add custom operations in sync with the MRT update strategy.
 *
 * The MRT uses a buffer structure to allow access to a in-use policy while a new policy is being prepared in a separate thread.
 * After a new policy is available in the separate thread, it is loaded into a policy buffer.
 *      - At this point the "modify_buffered_solution" of this class is called.
 *
 * When a user requests an update, the in-use policy is swapped for the buffered policy.
 *      - At this point the "modify_active_solution" of this class is called.
 *
 * Both filling of the buffer and the update swapping are protected by the same mutex.
 */
class MrtObserver
{
public:
  MrtObserver() = default;
  virtual ~MrtObserver() = default;

  // Delete copy and move operations
  MrtObserver(const MrtObserver &) = delete;
  MrtObserver & operator=(const MrtObserver &) = delete;
  MrtObserver(MrtObserver &&) = delete;
  MrtObserver & operator=(MrtObserver &&) = delete;

  /**
   * This method is called as part of MrtBase::update_policy().
   * It allows the user to modify the policy that will become in-use after the update_policy function returns.
   *
   * This function is executed sequentially with update_policy and thus blocks the main thread. Computationally expensive modifications
   * should therefore rather be done in "modify_buffered_solution".
   *
   * A call to this function is protected by the same mutex as modify_buffered_solution.
   */
  virtual void modify_active_solution(const CommandData & command, PrimalSolution & primalSolution) {}

  /**
   * This method is called by the MRT when a new policy is loaded into the buffer.
   * It allows the user to modify the buffered solution before it can be swapped during the update_policy call.
   *
   * When using a multi-threaded MRT, this function does not block the main thread.
   *
   * A call to this function is protected by the same mutex as modify_active_solution.
   */
  virtual void modify_buffered_solution(
    const CommandData & commandBuffer, PrimalSolution & primalSolutionBuffer)
  {
  }
};

}  // namespace ocs2