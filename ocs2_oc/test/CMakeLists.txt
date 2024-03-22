include_directories(include)

ament_auto_add_gtest(test_${PROJECT_NAME}_multiple_shooting
  multiple_shooting/testProjectionMultiplierCoefficients.cpp
  multiple_shooting/testTranscriptionMetrics.cpp
  multiple_shooting/testTranscriptionPerformanceIndex.cpp
)

ament_auto_add_gtest(test_${PROJECT_NAME}_data
  oc_data/testTimeDiscretization.cpp
)

ament_auto_add_gtest(test_${PROJECT_NAME}_rollout
  rollout/testTimeTriggeredRollout.cpp
  rollout/testStateTriggeredRollout.cpp
)

ament_auto_add_gtest(test_change_of_variables
  testChangeOfInputVariables.cpp
)

ament_auto_add_gtest(test_trajectory_spreading
  trajectory_adjustment/TrajectorySpreadingTest.cpp
)

ament_auto_add_gtest(test_ocp_to_kkt
  oc_problem/testOcpToKkt.cpp
)

ament_auto_add_gtest(test_precondition
  precondition/testPrecondition.cpp
)
