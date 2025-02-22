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

#include <memory>
#include <utility>
#include <vector>

#include "ocs2_core/augmented_lagrangian/StateAugmentedLagrangianCollection.hpp"
#include "ocs2_core/loopshaping/LoopshapingDefinition.hpp"

namespace ocs2
{

/**
 * Loopshaping state-only augmented Lagrangian collection class
 */
class LoopshapingStateAugmentedLagrangian final : public StateAugmentedLagrangianCollection
{
public:
  LoopshapingStateAugmentedLagrangian(
    const StateAugmentedLagrangianCollection & lagrangianCollection,
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
  : StateAugmentedLagrangianCollection(lagrangianCollection),
    loopshapingDefinition_(std::move(loopshapingDefinition))
  {
  }

  ~LoopshapingStateAugmentedLagrangian() override = default;
  LoopshapingStateAugmentedLagrangian * clone() const override
  {
    return new LoopshapingStateAugmentedLagrangian(*this);
  }

  std::vector<LagrangianMetrics> getValue(
    scalar_t t, const vector_t & x, const std::vector<Multiplier> & termsMultiplier,
    const PreComputation & preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    scalar_t t, const vector_t & x, const std::vector<Multiplier> & termsMultiplier,
    const PreComputation & preComp) const override;

  void updateLagrangian(
    scalar_t t, const vector_t & x, std::vector<LagrangianMetrics> & termsMetrics,
    std::vector<Multiplier> & termsMultiplier) const override;

private:
  LoopshapingStateAugmentedLagrangian(const LoopshapingStateAugmentedLagrangian & other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
