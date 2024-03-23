include_directories(include)
ament_auto_add_gtest(test_${PROJECT_NAME}
  testDiscreteTranscription.cpp
  testQpSolver.cpp
  testOcs2QpSolver.cpp
)
