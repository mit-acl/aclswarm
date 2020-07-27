#include <gtest/gtest.h>

#include <eigen3/unsupported/Eigen/KroneckerProduct>

#include <admm/solver.h>
#include <aclswarm/utils.h>

using namespace acl::aclswarm;

TEST(ADMMTest, fourAgentSquareFullyConnected)
{
  static constexpr size_t n = 4;
  admm::Solver admm;

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  PtsMat p = PtsMat::Zero(n, 3);

  p(0,0) = 0.0; p(0,1) = 0.0; p(0,2) = 2.5;
  p(1,0) = 2.0; p(1,1) = 0.0; p(1,2) = 3.5;
  p(2,0) = 2.0; p(2,1) = 2.0; p(2,2) = 4.5;
  p(3,0) = 0.0; p(3,1) = 2.0; p(3,2) = 1.5;

  GainMat A = admm.solve(p.transpose(), adj.cast<double>());

  Eigen::MatrixXd Amatlab(A.rows(), A.cols());
  Amatlab << -0.50,    0,    0, 0.25, 0.25,    0,    0,    0,    0, 0.25,-0.25,    0,
                 0,-0.50,    0,-0.25, 0.25,    0,    0,    0,    0, 0.25, 0.25,    0,
                 0,    0,-0.70,    0,    0, 0.20,    0,    0, 0.10,    0,    0, 0.40,
              0.25,-0.25,    0,-0.50,    0,    0, 0.25, 0.25,    0,    0,    0,    0,
              0.25, 0.25,    0,    0,-0.50,    0,-0.25, 0.25,    0,    0,    0,    0,
                 0,    0, 0.20,    0,    0,-0.70,    0,    0, 0.40,    0,    0, 0.10,
                 0,    0,    0, 0.25,-0.25,    0,-0.50,    0,    0, 0.25, 0.25,    0,
                 0,    0,    0, 0.25, 0.25,    0,    0,-0.50,    0,-0.25, 0.25,    0,
                 0,    0, 0.10,    0,    0, 0.40,    0,    0,-0.30,    0,    0,-0.20,
              0.25, 0.25,    0,    0,    0,    0, 0.25,-0.25,    0,-0.50,    0,    0,
             -0.25, 0.25,    0,    0,-   0,    0, 0.25, 0.25,    0,-   0,-0.50,    0,
                 0,    0, 0.40,    0,    0, 0.10,    0,    0,-0.20,    0,    0,-0.30;

  double err = (A - Amatlab).norm();

  EXPECT_NEAR(err, 0, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, fourAgentSquareNonComplete)
{
  static constexpr size_t n = 4;
  admm::Solver admm;

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  adj(0,2) = 0; adj(2,0) = 0;
  adj(1,3) = 0; adj(3,1) = 0;
  PtsMat p = PtsMat::Zero(n, 3);

  p(0,0) = 0.0; p(0,1) = 0.0; p(0,2) = 2.5;
  p(1,0) = 2.0; p(1,1) = 0.0; p(1,2) = 3.5;
  p(2,0) = 2.0; p(2,1) = 2.0; p(2,2) = 4.5;
  p(3,0) = 0.0; p(3,1) = 2.0; p(3,2) = 1.5;

  GainMat A = admm.solve(p.transpose(), adj.cast<double>());

  Eigen::MatrixXd Amatlab(A.rows(), A.cols());
  Amatlab << -0.500,     0,     0, 0.250, 0.250,     0,     0,     0,     0, 0.250,-0.250,     0,
                  0,-0.500,     0,-0.250, 0.250,     0,     0,     0,     0, 0.250, 0.250,     0,
                  0,     0,-0.750,     0,     0, 0.375,     0,     0,     0,     0,     0, 0.375,
              0.250,-0.250,     0,-0.500,     0,     0, 0.250, 0.250,     0,     0,     0,     0,
              0.250, 0.250,     0,     0,-0.500,     0,-0.250, 0.250,     0,     0,     0,     0,
                  0,     0, 0.375,     0,     0,-0.750,     0,     0, 0.375,     0,     0,     0,
                  0,     0,     0, 0.250,-0.250,     0,-0.500,     0,     0, 0.250, 0.250,     0,
                  0,     0,     0, 0.250, 0.250,     0,     0,-0.500,     0,-0.250, 0.250,     0,
                  0,     0,     0,     0,     0, 0.375,     0,     0,-0.250,     0,     0,-0.125,
              0.250, 0.250,     0,     0,     0,     0, 0.250,-0.250,     0,-0.500,     0,     0,
             -0.250, 0.250,     0,     0,     0,     0, 0.250, 0.250,     0,     0,-0.500,     0,
                  0,     0, 0.375,     0,     0,     0,     0,     0,-0.125,     0,     0,-0.250;

  double err = (A - Amatlab).norm();

  EXPECT_NEAR(err, 0, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, nineAgentSquareNonCompleteZeroBlocks)
{
  static constexpr size_t n = 9;
  admm::Solver admm;

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  adj(0,6) = 0;
  adj(2,4) = 0;
  adj(5,7) = 0; adj(5,8) = 0;
  adj(6,7) = 0;

  adj(4,2) = 0;
  adj(6,0) = 0;
  adj(7,5) = 0; adj(7,6) = 0;
  adj(8,5) = 0;

  PtsMat p = PtsMat::Zero(n, 3);
  p(0,0) = -1.7484733199059646; p(0,1) =  1.7306756147165174; p(0,2) = 0.2977622220453062;
  p(1,0) =  6.8174866001631180; p(1,1) = -6.2778267151168700; p(1,2) = 1.7416024649609380;
  p(2,0) = -3.8137004331127518; p(2,1) = -2.3232057308608365; p(2,2) = 0.4655014204423282;
  p(3,0) =  2.7536551200474015; p(3,1) = -5.5700708736518450; p(3,2) = 1.7252000594155040;
  p(4,0) = -3.5935365621834463; p(4,1) =  4.8028457222331170; p(4,2) = 1.2981050175550286;
  p(5,0) = -2.5820075847777666; p(5,1) =  7.4136205487374910; p(5,2) = 1.5131454738258028;
  p(6,0) =  0.8900655441583734; p(6,1) =  3.2902893860285527; p(6,2) = 1.5581930129432586;
  p(7,0) =  0.4370445360276376; p(7,1) = -5.7714142992744755; p(7,2) = 0.2531727259898202;
  p(8,0) = -6.1065377928157310; p(8,1) = -5.7852241311701940; p(8,2) = 1.7663507973073431;

  GainMat A = admm.solve(p.transpose(), adj.cast<double>());

  // boolean not of adjmat
  Eigen::Matrix<double, n, n> adjbar = (adj.cast<double>().array() - 1.0).cwiseAbs();
  adjbar += -Eigen::Matrix<double, n, n>::Identity();

  // select the 3x3 blocks of the gain matrix that should be zero
  GainMat Asel = Eigen::kroneckerProduct(adjbar, Eigen::Matrix3d::Ones());
  GainMat Azero = Asel.cwiseProduct(A);

  // check if all elements of 3x3 supposed zero blocks are actually zero
  EXPECT_NEAR(Azero.sum(), 0, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, nineAgentSquareNonCompleteBlockStructure)
{
  static constexpr size_t n = 9;
  admm::Solver admm;

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  adj(0,6) = 0;
  adj(2,4) = 0;
  adj(5,7) = 0; adj(5,8) = 0;
  adj(6,7) = 0;

  adj(4,2) = 0;
  adj(6,0) = 0;
  adj(7,5) = 0; adj(7,6) = 0;
  adj(8,5) = 0;

  PtsMat p = PtsMat::Zero(n, 3);
  p(0,0) = -1.7484733199059646; p(0,1) =  1.7306756147165174; p(0,2) = 0.2977622220453062;
  p(1,0) =  6.8174866001631180; p(1,1) = -6.2778267151168700; p(1,2) = 1.7416024649609380;
  p(2,0) = -3.8137004331127518; p(2,1) = -2.3232057308608365; p(2,2) = 0.4655014204423282;
  p(3,0) =  2.7536551200474015; p(3,1) = -5.5700708736518450; p(3,2) = 1.7252000594155040;
  p(4,0) = -3.5935365621834463; p(4,1) =  4.8028457222331170; p(4,2) = 1.2981050175550286;
  p(5,0) = -2.5820075847777666; p(5,1) =  7.4136205487374910; p(5,2) = 1.5131454738258028;
  p(6,0) =  0.8900655441583734; p(6,1) =  3.2902893860285527; p(6,2) = 1.5581930129432586;
  p(7,0) =  0.4370445360276376; p(7,1) = -5.7714142992744755; p(7,2) = 0.2531727259898202;
  p(8,0) = -6.1065377928157310; p(8,1) = -5.7852241311701940; p(8,2) = 1.7663507973073431;

  GainMat A = admm.solve(p.transpose(), adj.cast<double>());

  for (size_t i=0; i<n; ++i) {
    for (size_t j=0; j<n; ++j) {
      GainMat block = A.block<3,3>(3*i, 3*j);

      // all 3x3 blocks should have a specific "complex-like, symmetric" structure:
      // [ a  b  0]
      // [-b  a  0]
      // [ 0  0  c]

      bool sym = true;
      sym = (block(0,0) + -block(1,1))<1e-8 &&
            (block(1,0) +  block(0,1))<1e-8 &&
            block(0,2)<1e-8 &&
            block(2,0)<1e-8 &&
            block(1,2)<1e-8 &&
            block(2,1)<1e-8;

      EXPECT_TRUE(sym);
    }
  }

  // boolean not of adjmat
  Eigen::Matrix<double, n, n> adjbar = (adj.cast<double>().array() - 1.0).cwiseAbs();
  adjbar += -Eigen::Matrix<double, n, n>::Identity();

  // select the 3x3 blocks of the gain matrix that should be zero
  GainMat Asel = Eigen::kroneckerProduct(adjbar, Eigen::Matrix3d::Ones());
  GainMat Azero = Asel.cwiseProduct(A);

  // check if all elements of 3x3 supposed zero blocks are actually zero
  EXPECT_NEAR(Azero.sum(), 0, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, fixedTraceFullyConnected)
{
  static constexpr size_t n = 20;
  admm::Solver admm;

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  PtsMat p = PtsMat::Random(n, 3) * 5;

  GainMat A = admm.solve(p.transpose(), adj.cast<double>());

  static constexpr double d = 3;
  static constexpr double m = n - 2; // reduced dimension of problem
  static constexpr double expectedTrace = -d * m;

  EXPECT_NEAR(A.trace(), expectedTrace, 1e-8);
}

// ----------------------------------------------------------------------------

TEST(ADMMTest, fixedTraceSparse)
{
  static constexpr size_t n = 20;
  admm::Solver admm;

  AdjMat adj = AdjMat::Ones(n, n) - AdjMat::Identity(n, n);
  adj(0,5) = adj(5,0) = 0;
  adj(3,15) = adj(15,3) = 0;
  PtsMat p = PtsMat::Random(n, 3) * 5;

  GainMat A = admm.solve(p.transpose(), adj.cast<double>());

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
