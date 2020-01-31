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
  p(1,0) = 2.0; p(1,1) = 0.0; p(1,2) = 3.5;
  p(2,0) = 2.0; p(2,1) = 2.0; p(2,2) = 4.5;
  p(3,0) = 0.0; p(3,1) = 2.0; p(3,2) = 1.5;

  GainMat A = admm.calculateFormationGains(p, adj);

  Eigen::MatrixXd Amatlab(A.rows(), A.cols());
  Amatlab << -0.50, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.00, 0.00, 0.25,-0.25, 0.00,
             -0.00,-0.50, 0.00,-0.25, 0.25, 0.00, 0.00, 0.00, 0.00, 0.25, 0.25, 0.00,
              0.00, 0.00,-0.70, 0.00, 0.00, 0.20, 0.00, 0.00, 0.10, 0.00, 0.00, 0.40,
              0.25,-0.25, 0.00,-0.50, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.00, 0.00,
              0.25, 0.25, 0.00, 0.00,-0.50, 0.00,-0.25, 0.25, 0.00,-0.00,-0.00, 0.00,
              0.00, 0.00, 0.20, 0.00, 0.00,-0.70, 0.00, 0.00, 0.40, 0.00, 0.00, 0.10,
             -0.00, 0.00, 0.00, 0.25,-0.25, 0.00,-0.50,-0.00, 0.00, 0.25, 0.25, 0.00,
             -0.00, 0.00, 0.00, 0.25, 0.25, 0.00,-0.00,-0.50, 0.00,-0.25, 0.25, 0.00,
              0.00, 0.00, 0.10, 0.00, 0.00, 0.40, 0.00, 0.00,-0.30, 0.00, 0.00,-0.20,
              0.25, 0.25, 0.00, 0.00,-0.00, 0.00, 0.25,-0.25, 0.00,-0.50,-0.00, 0.00,
             -0.25, 0.25, 0.00, 0.00,-0.00, 0.00, 0.25, 0.25, 0.00,-0.00,-0.50, 0.00,
              0.00, 0.00, 0.40, 0.00, 0.00, 0.10, 0.00, 0.00,-0.20, 0.00, 0.00,-0.30;

  double err = (A - Amatlab).norm();

  EXPECT_NEAR(err, 0, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, fixedTraceFullyConnected)
{
  static constexpr size_t n = 20;
  ADMM admm(n);

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  PtsMat p = PtsMat::Random(n, 3) * 5;

  GainMat A = admm.calculateFormationGains(p, adj);

  static constexpr double d = 3;
  static constexpr double m = n - 2; // reduced dimension of problem
  static constexpr double expectedTrace = -d * m;

  EXPECT_NEAR(A.trace(), expectedTrace, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, fixedTraceSparse)
{
  static constexpr size_t n = 20;
  ADMM admm(n);

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  adj(0,5) = adj(5,0) = 0;
  adj(3,15) = adj(15,3) = 0;
  PtsMat p = PtsMat::Random(n, 3) * 5;

  GainMat A = admm.calculateFormationGains(p, adj);

  static constexpr double d = 3;
  static constexpr double m = n - 2; // reduced dimension of problem
  static constexpr double expectedTrace = -d * m;

  EXPECT_NEAR(A.trace(), expectedTrace, 1e-8);
}

// ----------------------------------------------------------------------------

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
