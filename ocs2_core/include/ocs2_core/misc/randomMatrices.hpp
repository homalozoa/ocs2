// Copyright 2020 Ruben Grandia. All rights reserved.
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
namespace LinearAlgebra
{
/**
 * Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix.
 * @return random SPD matrix.
 */
template <typename MatrixType>
MatrixType generateSPDmatrix()
{
  MatrixType A;
  A.setRandom();
  A = 0.5 * (A + A.transpose()).eval();    // avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix of dimension size.
 * @param [in] size: Matrix dimension.
 * @return random SPD matrix.
 */
template <typename MatrixType>
MatrixType generateSPDmatrix(int size)
{
  MatrixType A(size, size);
  A.setRandom();
  A = 0.5 * (A + A.transpose()).eval();    // Avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * Compute random full row rank matrix
 * @return random SPD matrix.
 */
matrix_t generateFullRowRankmatrix(size_t m, size_t n)
{
  if (m > n) {
    throw std::runtime_error(
      "[generateFullRowRankmatrix] Can't generate matrix with more rows than columns");
  }

  // Some random constraint matrix
  matrix_t A = matrix_t::Random(m, n);
  A.block(0, 0, m, m).setIdentity();  // Makes sure rows are independent
  return A;
}
}  // namespace LinearAlgebra
}  // namespace ocs2
