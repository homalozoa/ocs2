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

#include "ocs2_core/dynamics/SystemDynamicsBase.hpp"

namespace ocs2
{

/**
 *
 * A linear time invariant system with the following flow and jump maps:
 *
 * - \f$ \dot{x} = A * x + B * u   \quad \text{for intermediate times}, \f$
 * - \f$ x^{+} = G * x^{-}         \quad \text{for switching times}. \f$
 *
 * where \f$ g(x) \f$ is the guard surface defined by OdeBase::computeGuardSurfaces(t, x).
 */
class LinearSystemDynamics : public SystemDynamicsBase
{
public:
  LinearSystemDynamics(matrix_t A, matrix_t B, matrix_t G = matrix_t());

  ~LinearSystemDynamics() override = default;

  LinearSystemDynamics * clone() const override;

  vector_t computeFlowMap(
    scalar_t t, const vector_t & x, const vector_t & u, const PreComputation &) override;

  vector_t computeJumpMap(scalar_t t, const vector_t & x, const PreComputation &) override;

  VectorFunctionLinearApproximation linearApproximation(
    scalar_t t, const vector_t & x, const vector_t & u, const PreComputation &) override;

  VectorFunctionLinearApproximation jumpMapLinearApproximation(
    scalar_t t, const vector_t & x, const PreComputation &) override;

protected:
  LinearSystemDynamics(const LinearSystemDynamics & other) = default;

  matrix_t A_;
  matrix_t B_;
  matrix_t G_;
};

}  // namespace ocs2
