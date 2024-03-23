ament_auto_add_gtest(quadratic_test
  QuadraticTest.cpp
)

ament_auto_add_gtest(matyas_test
  MatyasTest.cpp
)

ament_auto_add_gtest(glpk_test
  testGLPK.cpp
)
target_link_libraries(glpk_test
  GLPK::GLPK
)
