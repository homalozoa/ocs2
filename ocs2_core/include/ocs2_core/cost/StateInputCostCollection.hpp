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

#include "StateInputCost.hpp"
#include "ocs2_core/Types.hpp"
#include "ocs2_core/misc/Collection.hpp"
#include "ocs2_core/reference/TargetTrajectories.hpp"

namespace ocs2
{

/**
 * State Input Cost function combining a collection of cost terms.
 *
 * This class collects a variable number of cost terms and provides methods to get the
 * summed cost values and quadratic approximations. Each cost term can be accessed through its
 * string name and can be activated or deactivated.
 */
class StateInputCostCollection : public Collection<StateInputCost>
{
public:
  StateInputCostCollection() = default;
  ~StateInputCostCollection() override = default;
  StateInputCostCollection * clone() const override;

  /** Get state-input cost value */
  virtual scalar_t getValue(
    scalar_t time, const vector_t & state, const vector_t & input,
    const TargetTrajectories & targetTrajectories, const PreComputation & preComp) const;

  /** Get state-input cost quadratic approximation */
  virtual ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    scalar_t time, const vector_t & state, const vector_t & input,
    const TargetTrajectories & targetTrajectories, const PreComputation & preComp) const;

protected:
  /** Copy constructor */
  StateInputCostCollection(const StateInputCostCollection & other);
};

}  // namespace ocs2
