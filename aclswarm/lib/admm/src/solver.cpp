/**
 * @file solver.cpp
 * @brief API for ADMM-based formation gain solver
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 25 July 2020
 */

#include <iostream>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/SparseCholesky>

#include "admm/solver.h"

namespace acl {
namespace aclswarm {
namespace admm {

Solver::Solver(const Params& params)
: params_(params)
{

}

// ----------------------------------------------------------------------------

Eigen::MatrixXd Solver::solve(
                        const Eigen::Matrix<double, 3, Eigen::Dynamic>& pts,
                        const Eigen::MatrixXd& adj)
{

  //
  // Solve 2D gain design subproblem
  //

  const auto A2d = solve2d(pts.topRows(2), adj);

  //
  // Solve 1D gain design subproblem
  //

  const auto A1d = solve1d(pts.bottomRows(1), adj);

  //
  // Combine for 3D gain design problem
  //

  const size_t n = pts.cols();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*n,3*n);
  for (size_t i=0; i<A.rows(); ++i) {
    for (size_t j=0; j<A.cols(); ++j) {

      // which 3x3 A_ij sub-block are we in?
      const size_t blki = i / 3;
      const size_t blkj = j / 3;

      // map index into 2d sub-block
      const size_t i2d = i - blki;
      const size_t j2d = j - blkj;

      // map index into 1d sub-block
      const size_t i1d = blki;
      const size_t j1d = blkj;

      // determine if we are indexing the 3rd row/col in A_ij
      bool row3 = ((i+1) % 3) == 0;
      bool col3 = ((j+1) % 3) == 0;

      if (!row3 && !col3) {
        A(i,j) = A2d(i2d,j2d);
      } else if (row3 && col3) {
        A(i,j) = A1d(i1d,j1d);
      }
    }
  }

  return A;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

Eigen::MatrixXd Solver::solve1d(
                        const Eigen::Matrix<double, 1, Eigen::Dynamic>& pts,
                        const Eigen::MatrixXd& adj)
{

  //
  // Build orthogonal complement of gain matrix kernel
  //

  const size_t n = adj.rows();
  const size_t d = 1; // ambient dimension of the problem

  // xy stacked
  Eigen::Map<const Eigen::VectorXd> qz(pts.data(), pts.size());

  // one vector
  Eigen::VectorXd ez = Eigen::VectorXd::Ones(n);

  // determine if desired formation is actually 2D (flat planar)
  const double stdev = std::sqrt((qz.array() - qz.mean()).array().square().sum()/(n-1));
  bool xyflat = (stdev < params_.thrPlanar);

  // kernel of gain matrix
  size_t dimKer;
  Eigen::MatrixXd N;
  if (xyflat) {
    dimKer = 1;
    N = Eigen::MatrixXd(pts.size(), dimKer);
    N << qz;
  } else {
    dimKer = 2;
    N = Eigen::MatrixXd(pts.size(), dimKer);
    N << qz, ez;
  }
  const size_t m = n - dimKer; // reduced number due to orth. compl. restriction

  // find the orthogonal complement of the kernel
  // recall: N = [U1 U2][S 0; 0 0][V1h; V2h]. We want U2.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(N, Eigen::ComputeFullU);
  Eigen::MatrixXd Q = svd.matrixU().rightCols(svd.matrixU().cols() - dimKer);

  //
  // Build the gain design optimization problem
  //

  SpMat C, A, b, X;
  parse(d, m, n, adj, Q, C, A, b, X);

  //
  // Solve SDP using ADMM on sparse matrices
  //

  admm(C, A, b, X);

  //
  // Recover gain matrix
  //

  Eigen::MatrixXd Aopt = - Q * X.bottomRightCorner(d*m, d*m) * Q.transpose();
  Aopt = (params_.thrSparseZero < Aopt.array().abs()).select(Aopt, 0.0);

  return Aopt;
}

// ----------------------------------------------------------------------------

Eigen::MatrixXd Solver::solve2d(
                        const Eigen::Matrix<double, 2, Eigen::Dynamic>& pts,
                        const Eigen::MatrixXd& adj)
{

  //
  // Build orthogonal complement of gain matrix kernel
  //

  const size_t n = adj.rows();
  const size_t m = n - 2; // reduced number due to orth. compl. restriction
  const size_t d = 2; // ambient dimension of the problem

  // xy stacked
  Eigen::Map<const Eigen::VectorXd> q(pts.data(), pts.size());

  // 90-degree rotated (-yx stacked)
  Eigen::VectorXd qbar = Eigen::VectorXd::Zero(pts.size());
  Eigen::Map<const Eigen::VectorXd, 0, Eigen::InnerStride<2>> qx(q.data(), q.size()/2);
  Eigen::Map<const Eigen::VectorXd, 0, Eigen::InnerStride<2>> qy(q.data()+1, q.size()/2);
  Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<2>> qbarx(qbar.data(), qbar.size()/2);
  Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<2>> qbary(qbar.data()+1, qbar.size()/2);
  qbarx = -qy;
  qbary =  qx;

  // one vectors
  Eigen::VectorXd ex = Eigen::Vector2d::UnitX().replicate(n, 1);
  Eigen::VectorXd ey = Eigen::Vector2d::UnitY().replicate(n, 1);

  // kernel of gain matrix
  static constexpr size_t dimKer = 4;
  Eigen::Matrix<double, Eigen::Dynamic, dimKer> N = Eigen::MatrixXd(pts.size(), dimKer);
  N << q, qbar, ex, ey;

  // find the orthogonal complement of the kernel
  // recall: N = [U1 U2][S 0; 0 0][V1h; V2h]. We want U2.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(N, Eigen::ComputeFullU);
  Eigen::MatrixXd Q = svd.matrixU().rightCols(svd.matrixU().cols() - dimKer);

  //
  // Build the gain design optimization problem
  //

  SpMat C, A, b, X;
  parse(d, m, n, adj, Q, C, A, b, X);

  //
  // Solve SDP using ADMM on sparse matrices
  //

  admm(C, A, b, X);

  //
  // Recover gain matrix
  //

  Eigen::MatrixXd Aopt = - Q * X.bottomRightCorner(d*m, d*m) * Q.transpose();
  Aopt = (params_.thrSparseZero < Aopt.array().abs()).select(Aopt, 0.0);

  return Aopt;
}

// ----------------------------------------------------------------------------

inline size_t Solver::vecsel(size_t rows, size_t cols, size_t i, size_t j)
{
  return j*rows + i;
}

// ----------------------------------------------------------------------------

inline size_t Solver::blksel(size_t dim, size_t blkidx, size_t subidx)
{
  return dim*blkidx + subidx;
}

// ----------------------------------------------------------------------------

inline void Solver::vectorize(const SpMat& X, SpMat& x)
{
  x.resize(X.size(), 1);
  x.reserve(X.nonZeros());
  x.startVec(0);
  for (size_t j=0; j<X.cols(); ++j) {
    for (SpMat::InnerIterator it(X, j); it; ++it) {
      x.insertBack(j*X.rows() + it.row(), 0) = it.value();
    }
  }
}

// ----------------------------------------------------------------------------

inline void Solver::unvectorize(const SpMat& x, SpMat& X)
{
  X.reserve(x.nonZeros());
  int curj = -1;

  for (SpMat::InnerIterator it(x, 0); it; ++it) {

    // select the correct destination row/col.
    const size_t i = it.row() % X.rows();
    const size_t j = it.row() / X.cols();
    if (j != curj) {
      X.startVec(j);
      curj = j;
    }

    X.insertBack(i, j) = it.value();
  }
}

// ----------------------------------------------------------------------------

void Solver::admm(const SpMat& C, const SpMat& A, const SpMat& b, SpMat& X)
{

  // cached operations
  const SpMat As = A.adjoint(); // dual operator
  Eigen::SimplicialCholesky<SpMat> AAs((A * As).pruned());

  // initialize intermediate variables
  SpMat Xold;
  SpMat S(X.rows(), X.cols());
  SpMat y(b.rows(), 1);

  //
  // ADMM Iterations
  //

  for (size_t i=0; i<params_.maxItr; ++i) {

    // update y
    {
      const SpMat D = C - S - params_.mu * X;
      SpMat Dvec; vectorize(D, Dvec);
      const SpMat e = A * Dvec + params_.mu * b;
      y = AAs.solve(e); // AAs \ e
    }

    // update S
    SpMat W;
    {
      const SpMat d = (As * y).pruned(1, params_.thrSparseZero);
      SpMat dmat(X.rows(), X.cols()); unvectorize(d, dmat);
      const SpMat WW = C - dmat - params_.mu * X;
      W = (WW + SpMat(WW.transpose())) / 2.0;
    }

    // determine index where positive evals start
    Eigen::SelfAdjointEigenSolver<SpMat> es(W);
    size_t k = 0;
    for (size_t i=0; i<W.rows(); ++i) {
      if (es.eigenvalues()(i) > params_.epsEig) {
        k = i;
        break;
      }
    }
    const size_t idxPosStart = W.rows() - k;

    // remove non-positive modes
    const Eigen::MatrixXd V = es.eigenvectors().rightCols(idxPosStart);
    const Eigen::MatrixXd D = es.eigenvalues().tail(idxPosStart).asDiagonal();
    S = (V * D * V.transpose()).sparseView(1, params_.thrSparseZero);

    // update X
    Xold = X;
    X = (S - W) / params_.mu;

    // check stop criteria --- difference in X
    const double diffX = (X - Xold).cwiseAbs().sum();
    if (diffX < params_.thresh) break;

    // check problem specific stop criteria --- trace value of \bar{A}
    const auto Abar = X.bottomRightCorner(X.rows()/2, X.cols()/2);
    const double Etr = Abar.rows(); // expected trace value (d*m)
    double tr = 0;
    for (size_t k=0; k<Abar.rows(); ++k) tr += Abar.coeff(k,k);
    double trPercentErr = (tr - Etr) / Etr;
    if (trPercentErr < params_.threshTr) break;
  }

  //
  // Project soln to ensure graph constraints are satisfied (set S=0)
  //

  const SpMat D = C - params_.mu * X;
  SpMat Dvec; vectorize(D, Dvec);
  const SpMat e = A * Dvec + params_.mu * b;
  y = AAs.solve(e); // AAs \ e

  const SpMat d = (As * y).pruned(1, params_.thrSparseZero);
  SpMat dmat(X.rows(), X.cols()); unvectorize(d, dmat);
  const SpMat WW = C - dmat - params_.mu * X;
  const SpMat W = (WW + SpMat(WW.transpose())) / 2.0;

  X = (- W) / params_.mu;
}

// ----------------------------------------------------------------------------

void Solver::parse(size_t d, size_t m, size_t n,
                      const Eigen::MatrixXd& adj, const Eigen::MatrixXd& Q,
                      SpMat& C, SpMat& A, SpMat& b, SpMat& X)
{
  //
  // Preallocate number of non-zeros
  //

  // block X_11
  const size_t nrA_X11 =
      (d*m-1)*2               // [X_11]_11 can be whatever it wants (t)
                              // but the other diag elements must be == [X_11]_11
    + (d*m)*(d*m-1)/2;        // set upper-triangular elements to zero
  const size_t nrb_X11 = 0;

  // block X_12
  const size_t nrA_X12 =
      d*m                     // each diagonal elem must be 1
    + (d*m)*(d*m-1);          // each off-diagonal elem must be 0
  const size_t nrb_X12 = d*m; // each diagonal elem must be 1

  // structure constraints for each gain matrix block
  const size_t nrA_X22_struct =
  (d == 2) ?
      0.5*m*(m+1)*(2+2)       // structure constraints: A_ij = [a b; -b a]
                              // 0.5*m*(m+1): each blk, including A_ii blks
    - m                       // don't count -b elem on A_ii (below diag)
                              // (2+2) because a-a=0 is 2 and b-b=0 is 2
  : 0; // no structure requirement for 1D subproblem
  const size_t nrb_X22_struct = 0;

  // zero-gain constraints based on given adj mat
  const size_t nr0 = ((adj.array()==0).count() - n)/2; // number of zeros in adj
  const size_t nrA_X22_adjmat =
      nr0*(d*(d*m)*(d*m));    // each 0 in adj creates d constraints on \bar{A}
  const size_t nrb_X22_adjmat = 0;

  // trace constraint on \bar{A}
  const size_t nrA_X22_trace =
      d*m;                    // the sum of each [X_22]_ii == d*m*destrace
  const size_t nrb_X22_trace = 1;

  // X must be symmetric: [X]_ij == [X]_ji
  const size_t nrA_X_sym =
      2 * d*m * (2*d*m-1);    //
  const size_t nrb_X_sym = 0;

  // total number of elements from constraints
  const size_t nrA = nrA_X11 + nrA_X12 + nrA_X22_struct + nrA_X22_adjmat + nrA_X22_trace + nrA_X_sym;
  const size_t nrb = nrb_X11 + nrb_X12 + nrb_X22_struct + nrb_X22_adjmat + nrb_X22_trace + nrb_X_sym;

  if (params_.verbose) {
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << "nrA_X11: " << nrA_X11 << std::endl;
    std::cout << "nrA_X12: " << nrA_X12 << std::endl;
    std::cout << "nrA_X22_struct: " << nrA_X22_struct << std::endl;
    std::cout << "nr0: " << nr0 << std::endl;
    std::cout << "nrA_X22_adjmat: " << nrA_X22_adjmat << std::endl;
    std::cout << "nrA_X22_trace: " << nrA_X22_trace << std::endl;
    std::cout << "nrA_X_sym: " << nrA_X_sym << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << "nrA: " << nrA << std::endl;
    std::cout << "nrb: " << nrb << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
  }

  std::vector<Eigen::Triplet<double>> Acoeffs, bcoeffs;
  Acoeffs.reserve(nrA);
  bcoeffs.reserve(nrb);

  size_t itrr = 0; // which row of \mathbf{A} should nz val be in?

  //
  // Build constraints for block X_11
  //

  // diagonal entries of X_11 should be equal to the first diagonal entry
  for (size_t i=1; i<d*m; ++i) {

    // always select first diagonal entry
    Acoeffs.emplace_back(itrr, 0, 1);

    // [1 0 ... -1 ... 0] vec(X) = 0
    //           ^
    //           selects elem corresponding to diagonal, [X_11]_ii
    const size_t itrc = vecsel(2*d*m, 2*d*m, i, i);
    Acoeffs.emplace_back(itrr, itrc, -1);

    // create new row in \mathbf{A} linear constraint matrix
    itrr++;
  }

  // off-diagonal entries should be zero
  for (size_t i=0; i<d*m; ++i) {
    for (size_t j=i+1; j<d*m; ++j) {

      const size_t itrc = vecsel(2*d*m, 2*d*m, i, j);
      Acoeffs.emplace_back(itrr, itrc, 1);

      // create new row in \mathbf{A} linear constraint matrix
      itrr++;
    }
  }

  size_t tmpA = 0;
  size_t tmpb = 0;

  if (params_.verbose) {
    std::cout << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << "Built rows in \\mathbf{A} for X_11 constraints" << std::endl;
    std::cout << "nnzA: " << Acoeffs.size()-tmpA << std::endl;
    std::cout << "nnzb: " << bcoeffs.size()-tmpb << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl;

    tmpA = Acoeffs.size();
    tmpb = bcoeffs.size();
  }

  //
  // Build constraints for block X_12
  //

  // off-diagonal entries should be zero
  for (size_t i=0; i<d*m; ++i) {
    for (size_t j=0; j<d*m; ++j) {

      const size_t jj = d*m + j; // skip first dm cols to get into X_12
      const size_t itrc = vecsel(2*d*m, 2*d*m, i, jj);

      // diagonal entries should be one
      if (i == j) {
        Acoeffs.emplace_back(itrr, itrc, 1);
        bcoeffs.emplace_back(itrr,    0, 1);
      } else { // all other entries should be zero
        Acoeffs.emplace_back(itrr, itrc, 1);
      }

      // create new row in \mathbf{A} linear constraint matrix
      itrr++;
    }
  }

  if (params_.verbose) {
    std::cout << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << "Built rows in \\mathbf{A} for X_12 constraints" << std::endl;
    std::cout << "nnzA: " << Acoeffs.size()-tmpA << std::endl;
    std::cout << "nnzb: " << bcoeffs.size()-tmpb << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl;

    tmpA = Acoeffs.size();
    tmpb = bcoeffs.size();
  }

  //
  // Build constraints for block X_22 = \bar{A}
  //

  if (d == 2) {
    // structure constraints A_ij = [a b; -b a]
    for (size_t i=0; i<m; ++i) {
      for (size_t j=i; j<m; ++j) {

        // diagonal entries should be equal
        const size_t ii1 = d*m + blksel(d, i, 0); // skip first dm rows
        const size_t jj1 = d*m + blksel(d, j, 0); // and cols for X_22
        const size_t ii2 = d*m + blksel(d, i, 1);
        const size_t jj2 = d*m + blksel(d, j, 1);
        const size_t itrc1 = vecsel(2*d*m, 2*d*m, ii1, jj1);
        const size_t itrc2 = vecsel(2*d*m, 2*d*m, ii2, jj2);
        Acoeffs.emplace_back(itrr, itrc1,  1);
        Acoeffs.emplace_back(itrr, itrc2, -1);

        // create new row in \mathbf{A} linear constraint matrix
        itrr++;

        // off-diagonal entries should have same value with opposite sign.
        if (i == j) {
          // If operating on a blk on the diag (A_ii), only enfore constraints
          // for the upper triangular portion---sym const enforced later.
          // note that these constraints enforce b = 0.
          const size_t ii = d*m + blksel(d, i, 0); // skip first dm rows
          const size_t jj = d*m + blksel(d, j, 1); // and cols for X_22
          const size_t itrc = vecsel(2*d*m, 2*d*m, ii, jj);
          Acoeffs.emplace_back(itrr, itrc, 1);
        } else {
          const size_t ii1 = d*m + blksel(d, i, 0); // skip first dm rows
          const size_t jj1 = d*m + blksel(d, j, 1); // and cols for X_22
          const size_t ii2 = d*m + blksel(d, i, 1);
          const size_t jj2 = d*m + blksel(d, j, 0);
          const size_t itrc1 = vecsel(2*d*m, 2*d*m, ii1, jj1);
          const size_t itrc2 = vecsel(2*d*m, 2*d*m, ii2, jj2);
          Acoeffs.emplace_back(itrr, itrc1, 1);
          Acoeffs.emplace_back(itrr, itrc2, 1);
        }

        // create new row in \mathbf{A} linear constraint matrix
        itrr++;
      }
    }
  }

  // graph constraints (zero blocks for non-neighbors)
  if (nr0 > 0) {
    for (size_t i=0; i<n; ++i) {
      for (size_t j=i+1; j<n; ++j) {
        if (adj(i,j) == 1) continue;

        // we leverage the structure constraint [a b; -b a] and only
        // create explicit constraints for [A_ij]_11 and [A_ij]_12.

        // two constraint rows are created in \mathbf{A}
        const size_t itrr1 = itrr;
        const size_t itrr2 = itrr + 1;

        // Constraint on [A_ij]_11
        const size_t ii1 = blksel(d, i, 0); // skip first dm rows
        const size_t jj1 = blksel(d, j, 0); // and cols for X_22
        const Eigen::MatrixXd QQ1 = Q.transpose().col(jj1) * Q.row(ii1);

        Eigen::MatrixXd QQ2;
        if (d == 2) {
          // Constraint on [A_ij]_12
          const size_t ii2 = blksel(d, i, 1); // skip first dm rows
          const size_t jj2 = blksel(d, j, 0); // and cols for X_22
          QQ2 = Q.transpose().col(jj2) * Q.row(ii2);
        }

        // Note how the linear transformation using the orthogonal complement Q
        // leaks signal into each element of the gain matrix \bar{A}.
        for (size_t ki=0; ki<d*m; ++ki) {
          for (size_t kj=0; kj<d*m; ++kj) {

            const size_t ii = d*m + ki; // skip first dm rows
            const size_t jj = d*m + kj; // and cols for X_22
            const size_t itrc = vecsel(2*d*m, 2*d*m, ii, jj);

            Acoeffs.emplace_back(itrr1, itrc, QQ1(ki,kj));
            if (d == 2) Acoeffs.emplace_back(itrr2, itrc, QQ2(ki,kj));
          }
        }

        // advance by d rows in \mathbf{A} linear constraint matrix
        itrr += d;
      }
    }
  }

  // trace of \bar{A} matrix must be the specified value
  {
    for (size_t i=0; i<d*m; ++i) {

      const size_t ii = d*m + i; // skip first dm rows/cols for X_22
      const size_t itrc = vecsel(2*d*m, 2*d*m, ii, ii);
      Acoeffs.emplace_back(itrr, itrc, 1);
    }

    // expected trace value
    bcoeffs.emplace_back(itrr, 0, d*m);

    // create new row in \mathbf{A} linear constraint matrix
    itrr++;
  }

  if (params_.verbose) {
    std::cout << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << "Built rows in \\mathbf{A} for X_22 constraints" << std::endl;
    std::cout << "nnzA: " << Acoeffs.size()-tmpA << std::endl;
    std::cout << "nnzb: " << bcoeffs.size()-tmpb << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl;

    tmpA = Acoeffs.size();
    tmpb = bcoeffs.size();
  }

  //
  // Symmetry constraints for entire X matrix
  //

  // symmetric entries should be equal
  for (size_t i=0; i<2*d*m; ++i) {
    for (size_t j=i+1; j<2*d*m; ++j) {

      const size_t itrc1 = vecsel(2*d*m, 2*d*m, i, j);
      const size_t itrc2 = vecsel(2*d*m, 2*d*m, j, i);
      Acoeffs.emplace_back(itrr, itrc1,  1);
      Acoeffs.emplace_back(itrr, itrc2, -1);

      // create new row in \mathbf{A} linear constraint matrix
      itrr++;
    }
  }

  if (params_.verbose) {
    std::cout << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << "Built rows in \\mathbf{A} for X symmetry constraints" << std::endl;
    std::cout << "nnzA: " << Acoeffs.size()-tmpA << std::endl;
    std::cout << "nnzb: " << bcoeffs.size()-tmpb << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl;

    tmpA = Acoeffs.size();
    tmpb = bcoeffs.size();
  }

  //
  // Prepare sparse matrices for ADMM
  //

  C.resize(2*d*m, 2*d*m);
  C.reserve(Eigen::VectorXi::Constant(d*m,1)); // reserve 1 nz per column of X_11
  for (size_t i=0; i<d*m; ++i) C.insert(i,i) = 1; // make [I 0; 0 0]

  // initialize decision variable to something fairly close
  X.resize(2*d*m, 2*d*m); // [I I; I I]
  X.reserve(Eigen::VectorXi::Constant(2*d*m,2)); // reserve 2 nz per column
  for (size_t i=0; i<d*m; ++i) {
    X.insert(i,i) = 1;
    X.insert(d*m+i,i) = 1;
  }
  for (size_t i=d*m; i<2*d*m; ++i) {
    X.insert(i,i) = 1;
    X.insert(i-d*m,i) = 1;
  }

  A.resize(itrr, X.size());
  A.setFromTriplets(Acoeffs.begin(), Acoeffs.end());

  b.resize(itrr, 1);
  b.setFromTriplets(bcoeffs.begin(), bcoeffs.end());
}



} // ns admm
} // ns aclswarm
} // ns acl