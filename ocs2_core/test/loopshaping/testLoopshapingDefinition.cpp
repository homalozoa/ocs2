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
#include "gtest/gtest.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.hpp"
#include "testLoopshapingConfigurations.hpp"

TEST(testLoopshapingDefinition, readingAllDefinitions)
{
  for (const auto config : ocs2::configNames) {
    const auto configPath = ocs2::getAbsolutePathToConfigurationFile(config);
    auto loopshapingDefinition = ocs2::loopshaping_property_tree::load(configPath);
    loopshapingDefinition->print();
  }

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, stateInputAccessFunctions)
{
  for (const auto config : ocs2::configNames) {
    const auto configPath = ocs2::getAbsolutePathToConfigurationFile(config);
    auto loopshapingDefinition = ocs2::loopshaping_property_tree::load(configPath);

    const size_t stateDim = loopshapingDefinition->getInputFilter().getNumStates();
    const size_t systemStateDim = 1;
    const size_t augmentedstateDim = stateDim + systemStateDim;
    const size_t inputDim = loopshapingDefinition->getInputFilter().getNumInputs();

    const ocs2::vector_t augmentedState = ocs2::vector_t::Random(augmentedstateDim);
    const ocs2::vector_t augmentedInput = ocs2::vector_t::Random(inputDim);

    const ocs2::vector_t systemState = loopshapingDefinition->getSystemState(augmentedState);
    ASSERT_TRUE(augmentedState.head(systemStateDim).isApprox(systemState));

    const ocs2::vector_t filterState = loopshapingDefinition->getFilterState(augmentedState);
    ASSERT_TRUE(augmentedState.tail(stateDim).isApprox(filterState));

    const ocs2::vector_t augmentedStateConstructed =
      loopshapingDefinition->concatenateSystemAndFilterState(systemState, filterState);
    ASSERT_TRUE(augmentedState.isApprox(augmentedStateConstructed));

    const ocs2::vector_t systemInput =
      loopshapingDefinition->getSystemInput(augmentedState, augmentedInput);
    const ocs2::vector_t filteredInput =
      loopshapingDefinition->getFilteredInput(augmentedState, augmentedInput);
    const ocs2::vector_t augmentedInputConstructed =
      loopshapingDefinition->augmentedSystemInput(systemInput, filteredInput);
    ASSERT_TRUE(augmentedInput.isApprox(augmentedInputConstructed));

    ocs2::vector_t equilibriumState;
    ocs2::vector_t equilibriumInput;
    loopshapingDefinition->getFilterEquilibrium(systemInput, equilibriumState, equilibriumInput);
  }
}
