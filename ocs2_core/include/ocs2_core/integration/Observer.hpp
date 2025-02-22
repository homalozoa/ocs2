// Copyright 2024 Homalozoa. All rights reserved.
// Copyright 2020 Farbod Farshidian. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Farbod nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "ocs2_core/Types.hpp"

namespace ocs2
{

/**
 * The Observer class stores data in given containers.
 */
class Observer
{
public:
  /**
   * Constructor.
   *
   * @param stateTrajectoryPtr: A pinter to an state trajectory container to store resulting state trajectory.
   * @param timeTrajectoryPtr: A pinter to an time trajectory container to store resulting time trajectory.
   */
  explicit Observer(
    vector_array_t * stateTrajectoryPtr = nullptr, scalar_array_t * timeTrajectoryPtr = nullptr);

  /**
   * Default destructor.
   */
  ~Observer() = default;

  /**
   * Observe function to retrieve the variable of interest.
   * @param [in] state: Current state.
   * @param [in] time: Current time.
   */
  void observe(const vector_t & state, scalar_t time);

private:
  scalar_array_t * timeTrajectoryPtr_;
  vector_array_t * stateTrajectoryPtr_;
};

}  // namespace ocs2
