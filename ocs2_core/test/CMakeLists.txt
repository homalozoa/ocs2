ament_auto_add_gtest(
  test_control
  control/testLinearController.cpp
  control/testFeedforwardController.cpp
)

ament_auto_add_gtest(
  initialization_unittest
  initialization/InitializationTest.cpp
)

ament_auto_add_gtest(
  test_integration
  integration/testSensitivityIntegrator.cpp
  integration/IntegrationTest.cpp
  integration/testRungeKuttaDormandPrince5.cpp
  integration/TrapezoidalIntegrationTest.cpp
)

ament_auto_add_gtest(
  interpolation_unittest
  misc/testInterpolation.cpp
)

ament_auto_add_gtest(
  ${PROJECT_NAME}_cppadcg
  cppad_cg/testCppADCG_dynamics.cpp
  cppad_cg/testSparsityHelpers.cpp
  cppad_cg/testCppAdInterface.cpp
)

ament_auto_add_gtest(
  test_transferfunctionbase
  dynamics/testTransferfunctionBase.cpp
)

ament_auto_add_gtest(
  ${PROJECT_NAME}_loopshaping
  loopshaping/testLoopshapingConfiguration.cpp
  loopshaping/testLoopshapingAugmentedLagrangian.cpp
  loopshaping/testLoopshapingConstraint.cpp
  loopshaping/testLoopshapingCost.cpp
  loopshaping/testLoopshapingSoftConstraint.cpp
  loopshaping/testLoopshapingDefinition.cpp
  loopshaping/testLoopshapingDynamics.cpp
  loopshaping/testLoopshapingFilterDynamics.cpp
  loopshaping/testLoopshapingPreComputation.cpp
)

ament_auto_add_gtest(
  ${PROJECT_NAME}_test_misc
  misc/testInterpolation.cpp
  misc/testLinearAlgebra.cpp
  misc/testLogging.cpp
  misc/testLoadData.cpp
  misc/testLookup.cpp
)

ament_auto_add_gtest(
  test_dynamics
  dynamics/testSystemDynamicsLinearizer.cpp
  dynamics/testSystemDynamicsPreComputation.cpp
)

ament_auto_add_gtest(
  test_cost
  cost/testCostCollection.cpp
  cost/testCostCppAd.cpp
  cost/testQuadraticCostFunction.cpp
)

ament_auto_add_gtest(
  test_constraint
  constraint/testConstraintCollection.cpp
  constraint/testConstraintCppAd.cpp
  constraint/testLinearConstraint.cpp
)

ament_auto_add_gtest(
  test_metrics
  model_data/testMetrics.cpp
)

ament_auto_add_gtest(
  test_multiplier
  model_data/testMultiplier.cpp
)

ament_auto_add_gtest(
  test_ModelData
  model_data/testModelData.cpp
)

ament_auto_add_gtest(
  test_ModeSchedule
  reference/testModeSchedule.cpp
)

ament_auto_add_gtest(
  test_softConstraint
  soft_constraint/testSoftConstraint.cpp
  soft_constraint/testDoubleSidedPenalty.cpp
)

ament_auto_add_gtest(
  ${PROJECT_NAME}_test_thread_support
  thread_support/testBufferedValue.cpp
  thread_support/testSynchronized.cpp
  thread_support/testThreadPool.cpp
)

ament_auto_add_gtest(
  ${PROJECT_NAME}_test_core
  testPrecomputation.cpp
  testTypes.cpp
)
