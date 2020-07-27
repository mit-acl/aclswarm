/**
 * @file solver.h
 * @brief API for ADMM-based formation gain solver
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 25 July 2020
 */

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace acl {
namespace aclswarm {
namespace admm {

  /**
   * @brief      User parameters for ADMM solver
   */
  struct Params {
    bool verbose = false;
    double thrSparseZero = 1e-8; ///< sparse manip, smaller values become 0
    double thrPlanar = 1e-2; ///< if std(qz) less, formation is flat planar

    // \brief ADMM solver parameters
    double epsEig = 1e-5; ///< precision for positive eig vals
    double mu = 1; ///< ADMM penalty

    // \brief Stopping criteria for ADMM iterations
    double thresh = 1e-4; ///< threshold for change in decision variable, X
    double threshTr = 0.10; ///< if Tr[\bar{A}] within this percent of desired, stop.
    size_t maxItr = 10; ///< maximum number of ADMM iterations
  };

  class Solver
  {
  public:

  public:
    Solver(const Params& params = {});
    ~Solver() = default;

    Eigen::MatrixXd solve(
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& pts,
                const Eigen::MatrixXd& adj);

  private:
    Params params_;

    using SpMat = Eigen::SparseMatrix<double>;

    Eigen::MatrixXd solve1d(
                    const Eigen::Matrix<double, 1, Eigen::Dynamic>& pts,
                    const Eigen::MatrixXd& adj);

    Eigen::MatrixXd solve2d(
                    const Eigen::Matrix<double, 2, Eigen::Dynamic>& pts,
                    const Eigen::MatrixXd& adj);

    void parse(size_t d, size_t m, size_t n,
                const Eigen::MatrixXd& adj, const Eigen::MatrixXd& Q,
                SpMat& C, SpMat& A, SpMat& b, SpMat& X);

    void admm(const SpMat& C, const SpMat& A, const SpMat& b, SpMat& X);

    inline void vectorize(const SpMat& X, SpMat& x);
    inline void unvectorize(const SpMat& X, SpMat& x);
    inline size_t blksel(size_t dim, size_t blkidx, size_t subidx);
    inline size_t vecsel(size_t rows, size_t cols, size_t i, size_t j);
  };


} // ns admm
} // ns aclswarm
} // ns acl