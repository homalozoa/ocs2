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

#include "ocs2_core/automatic_differentiation/CppAdSparsity.hpp"

namespace ocs2
{

namespace cppad_sparsity
{

SparsityPattern getIntersection(const SparsityPattern & p0, const SparsityPattern & p1)
{
  assert(p0.size() == p1.size());
  const auto numRows = p0.size();

  SparsityPattern result(p0.size());
  for (int row = 0; row < numRows; row++) {
    std::set_intersection(
      p0[row].begin(), p0[row].end(), p1[row].begin(), p1[row].end(),
      std::inserter(result[row], result[row].begin()));
  }
  return result;
}

SparsityPattern getJacobianVariableSparsity(int rangeDim, int variableDim)
{
  // Jacobian : all variables are declared non-zero
  SparsityPattern jacobianSparsity(rangeDim);
  for (auto & sparsityRow : jacobianSparsity) {
    for (size_t i = 0; i < variableDim; i++) {
      sparsityRow.insert(i);
    }
  }
  return jacobianSparsity;
}

SparsityPattern getHessianVariableSparsity(int variableDim, int parameterDim)
{
  // Hessian : all upper triangular variable entries are declared non-zero
  SparsityPattern hessianSparsity(variableDim + parameterDim);
  for (size_t i = 0; i < variableDim; i++) {
    for (size_t j = i; j < variableDim; j++) {
      hessianSparsity[i].insert(j);
    }
  }
  return hessianSparsity;
}

size_t getNumberOfNonZeros(const SparsityPattern & sparsityPattern)
{
  size_t nnz = 0;
  for (const auto & row : sparsityPattern) {
    nnz += row.size();
  }
  return nnz;
}

}  // namespace cppad_sparsity
}  // namespace ocs2
