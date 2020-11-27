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

#include <ocs2_core/soft_constraint/penalties/PenaltyFunctionBase.h>

namespace ocs2 {

/**
 * Configuration object for the relaxed barrier penalty.
 * mu : scaling factor
 * delta: relaxation parameter, see class description
 */
struct RelaxedBarrierPenaltyConfig {
  scalar_t mu = 1.0;
  scalar_t delta = 1e-3;
};

/**
 * Implements the relaxed barrier function for a single inequality constraint \f$ h \geq 0 \f$
 *
 * \f[
 *   p(h)=\left\lbrace
 *               \begin{array}{ll}
 *                 -\mu \ln(h) & if \quad  h > \delta, \\
 *                 -\mu \ln(\delta) + \mu \frac{1}{2} \left( \left( \frac{h-2\delta}{\delta} \right)^2 - 1 \right) & otherwise,
 *               \end{array}
 *             \right.
 * \f]
 *
 * where \f$ \mu \geq 0 \f$, and \f$ \delta \geq 0 \f$ are user defined parameters.
 */
class RelaxedBarrierPenaltyFunction final : public PenaltyFunctionBase {
 public:
  /**
   * Constructior
   * @param [in] config: Configuration object containing mu and delta.
   */
  explicit RelaxedBarrierPenaltyFunction(RelaxedBarrierPenaltyConfig config) : config_(std::move(config)) {}

  /** Default destructor */
  ~RelaxedBarrierPenaltyFunction() override = default;

  RelaxedBarrierPenaltyFunction* clone() const override { return new RelaxedBarrierPenaltyFunction(*this); }

  scalar_t getValue(scalar_t h) const override;
  scalar_t getDerivative(scalar_t h) const override;
  scalar_t getSecondDerivative(scalar_t h) const override;

 private:
  RelaxedBarrierPenaltyFunction(const RelaxedBarrierPenaltyFunction& other) = default;

  RelaxedBarrierPenaltyConfig config_;
};

}  // namespace ocs2
