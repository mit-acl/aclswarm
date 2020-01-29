#include <gtest/gtest.h>

#include "aclswarm/admm.h"
#include "aclswarm/utils.h"

using namespace acl::aclswarm;

TEST(ADMMTest, fourAgentSquareFullyConnected)
{
  static constexpr size_t n = 4;
  ADMM admm(n);

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  PtsMat p = PtsMat::Zero(n, 3);

  p(0,0) = 0.0; p(0,1) = 0.0; p(0,2) = 2.5;
  p(1,0) = 2.0; p(1,1) = 0.0; p(1,2) = 2.5;
  p(2,0) = 2.0; p(2,1) = 2.0; p(2,2) = 2.5;
  p(3,0) = 0.0; p(3,1) = 2.0; p(3,2) = 2.5;

  std::cout << p << std::endl;
  std::cout << std::endl;

  GainMat A = admm.calculateFormationGains(p, adj);

  std::cout << A << std::endl;

  std::cout << A.trace() << std::endl;

  EXPECT_DOUBLE_EQ(0, 0);
}

// ----------------------------------------------------------------------------

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
