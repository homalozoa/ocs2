ament_auto_add_gtest(exp0_ddp_test
  Exp0Test.cpp
)

ament_auto_add_gtest(exp1_ddp_test
  Exp1Test.cpp
)

ament_auto_add_gtest(correctness_test
  CorrectnessTest.cpp
)

ament_auto_add_gtest(riccati_ode_test
  RiccatiTest.cpp
)

ament_auto_add_gtest(circular_kinematics_ddp_test
  CircularKinematicsTest.cpp
)

ament_auto_add_gtest(hybrid_slq_test
  HybridSlqTest.cpp
)

ament_auto_add_gtest(bouncing_mass_test
  bouncingmass/BouncingMassTest.cpp
  bouncingmass/OverallReference.cpp
  bouncingmass/Reference.cpp
)

ament_auto_add_gtest(testContinuousTimeLqr
  testContinuousTimeLqr.cpp
)

ament_auto_add_gtest(testDdpHelperFunction
  testDdpHelperFunction.cpp
)

ament_auto_add_gtest(testReachingTask
  testReachingTask.cpp
)
