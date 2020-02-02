//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign3D.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign2D.h"
#include "ADMMGainDesign3D_data.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "ADMMGainDesign3D_initialize.h"
#include "diag.h"
#include "diag1.h"
#include "eig.h"
#include "find.h"
#include "horzcat.h"
#include "mtimes.h"
#include "mtimes1.h"
#include "reshape.h"
#include "rt_nonfinite.h"
#include "sparse.h"
#include "sparse1.h"
#include "speye.h"
#include "std.h"
#include "sum.h"
#include "svd.h"
#include "triu.h"
#include "vec.h"
#include "vertcat.h"
#include <cmath>

// Function Definitions

//
// print out size information
// Arguments    : const emxArray_real_T *Qs
//                const emxArray_real_T *adj
//                emxArray_real_T *Aopt
// Return Type  : void
//
void ADMMGainDesign3D(const emxArray_real_T *Qs, const emxArray_real_T *adj,
                      emxArray_real_T *Aopt)
{
  emxArray_real_T *b_Qs;
  int loop_ub;
  int i;
  emxArray_real_T *Axy;
  emxArray_real_T *qz;
  emxArray_real_T *b_qz;
  int dimkerAz;
  int m;
  emxArray_real_T *U;
  emxArray_real_T *unusedU0;
  double unusedU1[4];
  emxArray_real_T *Qt;
  int b_loop_ub;
  int i1;
  emxArray_real_T *I0_d;
  int t7_m;
  coder_internal_sparse expl_temp;
  emxArray_int32_T *I0_colidx;
  emxArray_int32_T *I0_rowidx;
  emxArray_int32_T *Z0_colidx;
  emxArray_int32_T *Z0_rowidx;
  emxArray_real_T *Z_d;
  emxArray_int32_T *Z_colidx;
  emxArray_int32_T *Z_rowidx;
  emxArray_real_T *Ai;
  emxArray_real_T *bi;
  int I0_m;
  int I0_n;
  int Z0_m;
  int Z0_n;
  int Z_m;
  int Z_n;
  emxArray_int32_T *t7_colidx;
  emxArray_int32_T *t7_rowidx;
  emxArray_real_T *Aj;
  int t7_n;
  emxArray_int32_T *t8_colidx;
  emxArray_int32_T *t8_rowidx;
  emxArray_real_T *C_d;
  emxArray_int32_T *C_colidx;
  emxArray_int32_T *C_rowidx;
  emxArray_boolean_T *S;
  emxArray_real_T *QQ;
  emxArray_boolean_T *r;
  emxArray_boolean_T *t11_d;
  emxArray_int32_T *ii;
  emxArray_int32_T *jj;
  double idxR;
  double numElmAtot_tmp;
  emxArray_real_T *Av;
  emxArray_real_T *bv;
  double itrr;
  unsigned int itra;
  unsigned int itrb;
  int b_i;
  int kj;
  emxArray_real_T *A_d;
  emxArray_int32_T *A_colidx;
  emxArray_int32_T *A_rowidx;
  int A_m;
  emxArray_real_T *b_d;
  emxArray_int32_T *b_colidx;
  emxArray_int32_T *b_rowidx;
  emxArray_real_T *As_d;
  int b_m;
  int b_n;
  emxArray_int32_T *As_colidx;
  emxArray_int32_T *As_rowidx;
  emxArray_real_T *AAs_d;
  int As_m;
  emxArray_int32_T *AAs_colidx;
  emxArray_int32_T *AAs_rowidx;
  int AAs_m;
  int AAs_n;
  emxArray_real_T *X_d;
  emxArray_int32_T *X_colidx;
  emxArray_int32_T *X_rowidx;
  int X_m;
  int X_n;
  emxArray_creal_T *dd;
  emxArray_creal_T *V;
  emxArray_creal_T *D;
  emxArray_int32_T *r1;
  emxArray_int32_T *r2;
  emxArray_creal_T *y;
  emxArray_creal_T *b;
  emxArray_real_T *b_y;
  emxArray_real_T *c_y;
  emxArray_int32_T *t9_colidx;
  emxArray_int32_T *t9_rowidx;
  coder_internal_sparse_1 b_expl_temp;
  emxArray_creal_T *b_dd;
  emxArray_real_T *b_D;
  bool exitg1;
  double sizX[2];
  double d;
  if (isInitialized_ADMMGainDesign3D == false) {
    ADMMGainDesign3D_initialize();
  }

  emxInit_real_T(&b_Qs, 2);

  //  ADMM for computing gain matrix.
  //  --> Speeded up by using sparse matrix representation
  //  --> Set trace to a fixed value
  //
  //
  //  Inputs:
  //
  //        - Qs :  Desired formation coordinates (2*n matrix, each column representing coordinate of formation point) 
  //        - adj:  Graph adjacency matrix (n*n logical matrix)
  //
  //  Outputs:
  //
  //        - Aopt : Gain matrix
  //
  //
  //  Gain design for 2D component of the formation
  //  (2D component is defined as the projection of the 3D formation on the x-y plane) 
  loop_ub = Qs->size[1];
  i = b_Qs->size[0] * b_Qs->size[1];
  b_Qs->size[0] = 2;
  b_Qs->size[1] = Qs->size[1];
  emxEnsureCapacity_real_T(b_Qs, i);
  for (i = 0; i < loop_ub; i++) {
    b_Qs->data[2 * i] = Qs->data[3 * i];
    b_Qs->data[2 * i + 1] = Qs->data[3 * i + 1];
  }

  emxInit_real_T(&Axy, 2);
  emxInit_real_T(&qz, 1);
  ADMMGainDesign2D(b_Qs, adj, Axy);

  //  Gain design for the altitude
  //  Number of agents
  //  Vector of ones
  loop_ub = Qs->size[1];
  i = qz->size[0];
  qz->size[0] = Qs->size[1];
  emxEnsureCapacity_real_T(qz, i);
  emxFree_real_T(&b_Qs);
  for (i = 0; i < loop_ub; i++) {
    qz->data[i] = Qs->data[3 * i + 2];
  }

  emxInit_real_T(&b_qz, 2);

  //  Vector of z-coordinates
  //  ambient dimension of problem
  //  Kernel of gain matrix
  //  Determine if desired formation is actually 2D
  //  Reduced problem dimension
  if (b_std(qz) < 0.01) {
    dimkerAz = 1;
  } else {
    dimkerAz = 2;
  }

  m = adj->size[0] - dimkerAz;

  //  Get orthogonal complement of N
  i = b_qz->size[0] * b_qz->size[1];
  b_qz->size[0] = qz->size[0];
  b_qz->size[1] = 2;
  emxEnsureCapacity_real_T(b_qz, i);
  loop_ub = qz->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_qz->data[i] = qz->data[i];
  }

  loop_ub = adj->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_qz->data[i + b_qz->size[0]] = 1.0;
  }

  emxInit_real_T(&U, 2);
  emxInit_real_T(&unusedU0, 2);
  c_svd(b_qz, U, unusedU0, unusedU1);

  //  Qbar = U(:,1:dimkerAz);
  emxFree_real_T(&b_qz);
  emxFree_real_T(&unusedU0);
  if (dimkerAz + 1 > adj->size[0]) {
    dimkerAz = 0;
    i = -1;
  } else {
    i = adj->size[0] - 1;
  }

  emxInit_real_T(&Qt, 2);
  loop_ub = U->size[0];
  b_loop_ub = i - dimkerAz;
  i = b_loop_ub + 1;
  i1 = Qt->size[0] * Qt->size[1];
  Qt->size[0] = i;
  Qt->size[1] = U->size[0];
  emxEnsureCapacity_real_T(Qt, i1);
  for (i1 = 0; i1 < loop_ub; i1++) {
    for (t7_m = 0; t7_m <= b_loop_ub; t7_m++) {
      Qt->data[t7_m + Qt->size[0] * i1] = U->data[i1 + U->size[0] * (dimkerAz +
        t7_m)];
    }
  }

  emxInit_real_T(&I0_d, 1);
  c_emxInitStruct_coder_internal_(&expl_temp);

  //  Preallocate variables
  speye(static_cast<double>(m), &expl_temp);
  i1 = I0_d->size[0];
  I0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(I0_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&I0_colidx, 1);
  i1 = I0_colidx->size[0];
  I0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(I0_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&I0_rowidx, 1);
  i1 = I0_rowidx->size[0];
  I0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(I0_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_int32_T(&Z0_colidx, 1);
  emxInit_int32_T(&Z0_rowidx, 1);
  emxInit_real_T(&Z_d, 1);
  emxInit_int32_T(&Z_colidx, 1);
  emxInit_int32_T(&Z_rowidx, 1);
  emxInit_real_T(&Ai, 1);
  emxInit_real_T(&bi, 1);
  I0_m = expl_temp.m;
  I0_n = expl_temp.n;
  sparse(static_cast<double>(m), static_cast<double>(m), bi, Z0_colidx,
         Z0_rowidx, &Z0_m, &Z0_n);
  sparse(2.0 * static_cast<double>(m), 2.0 * static_cast<double>(m), Z_d,
         Z_colidx, Z_rowidx, &Z_m, &Z_n);

  //  Cost function's coefficient matrix: f = <C,X>
  sparse_horzcat(I0_d, I0_colidx, I0_rowidx, expl_temp.m, expl_temp.n, bi,
                 Z0_colidx, Z0_rowidx, Z0_m, Z0_n, &expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&t7_colidx, 1);
  i1 = t7_colidx->size[0];
  t7_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t7_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&t7_rowidx, 1);
  i1 = t7_rowidx->size[0];
  t7_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t7_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_real_T(&Aj, 1);
  t7_m = expl_temp.m;
  t7_n = expl_temp.n;
  sparse_horzcat(bi, Z0_colidx, Z0_rowidx, Z0_m, Z0_n, bi, Z0_colidx, Z0_rowidx,
                 Z0_m, Z0_n, &expl_temp);
  i1 = Aj->size[0];
  Aj->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Aj, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Aj->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&t8_colidx, 1);
  i1 = t8_colidx->size[0];
  t8_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t8_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t8_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&t8_rowidx, 1);
  i1 = t8_rowidx->size[0];
  t8_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t8_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t8_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_real_T(&C_d, 1);
  sparse_vertcat(Ai, t7_colidx, t7_rowidx, t7_m, t7_n, Aj, t8_colidx, t8_rowidx,
                 expl_temp.m, expl_temp.n, &expl_temp);
  i1 = C_d->size[0];
  C_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(C_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    C_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&C_colidx, 1);
  i1 = C_colidx->size[0];
  C_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(C_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    C_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&C_rowidx, 1);
  i1 = C_rowidx->size[0];
  C_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(C_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    C_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_boolean_T(&S, 2);

  //  Trace of gain matrix must be the specified value in 'trVal'
  //  fixed value for trace
  // %%%%%%%%%%%%%%%%%%%%% Find total number of nonzero elements in A and b matrices 
  //  Number of elements in block [X]_11
  //  Number of elements in block [X]_12
  //  Zero-gain constraints for the given adjacency graph
  i1 = S->size[0] * S->size[1];
  S->size[0] = adj->size[0];
  S->size[1] = adj->size[1];
  emxEnsureCapacity_boolean_T(S, i1);
  loop_ub = adj->size[0] * adj->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    S->data[i1] = !(adj->data[i1] != 0.0);
  }

  emxInit_real_T(&QQ, 2);
  emxInit_boolean_T(&r, 2);
  emxInit_boolean_T(&t11_d, 1);

  //  Upper triangular part
  diag(S, t11_d);
  b_diag(t11_d, r);
  i1 = QQ->size[0] * QQ->size[1];
  QQ->size[0] = S->size[0];
  QQ->size[1] = S->size[1];
  emxEnsureCapacity_real_T(QQ, i1);
  loop_ub = S->size[0] * S->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    QQ->data[i1] = static_cast<double>(S->data[i1]) - static_cast<double>
      (r->data[i1]);
  }

  emxFree_boolean_T(&r);
  emxFree_boolean_T(&S);
  emxInit_int32_T(&ii, 1);
  emxInit_int32_T(&jj, 1);
  triu(QQ);
  eml_find(QQ, ii, jj);
  i1 = qz->size[0];
  qz->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(qz, i1);
  loop_ub = ii->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    qz->data[i1] = ii->data[i1];
  }

  //  Find location of nonzero entries
  //  Number of constraints
  //  Number of elements in block [X]_22
  //  Number of elements for trace of [X]_22
  //  Number of elements for symmetry
  //  Number of elements for pinning down the b-vector
  //  Total number of elements
  idxR = static_cast<double>(m) * (static_cast<double>(m) - 1.0);
  numElmAtot_tmp = 2.0 * static_cast<double>(m);

  // %%%%%%%%%%%%%%%%%%%%% Preallocate sparse matrices A & b
  //
  //  Constraint: A * vec(X) = b
  //
  //  Indices of A: entry [Ai(k), Aj(k)] takes value of Av(k)
  //  Each row of matrix A will represent a constraint
  loop_ub = static_cast<int>(((((((static_cast<double>(m) - 1.0) * 2.0 + idxR /
    2.0) + (static_cast<double>(m) + idxR)) + static_cast<double>(qz->size[0]) *
    (static_cast<double>(m) * static_cast<double>(m))) + static_cast<double>(m))
    + numElmAtot_tmp * (numElmAtot_tmp - 1.0)));
  i1 = Ai->size[0];
  Ai->size[0] = loop_ub;
  emxEnsureCapacity_real_T(Ai, i1);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = 0.0;
  }

  i1 = Aj->size[0];
  Aj->size[0] = loop_ub;
  emxEnsureCapacity_real_T(Aj, i1);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Aj->data[i1] = 0.0;
  }

  emxInit_real_T(&Av, 1);
  i1 = Av->size[0];
  Av->size[0] = loop_ub;
  emxEnsureCapacity_real_T(Av, i1);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Av->data[i1] = 0.0;
  }

  loop_ub = static_cast<int>(((static_cast<double>(m) + 1.0) + 1.0));
  i1 = bi->size[0];
  bi->size[0] = loop_ub;
  emxEnsureCapacity_real_T(bi, i1);
  for (i1 = 0; i1 < loop_ub; i1++) {
    bi->data[i1] = 0.0;
  }

  emxInit_real_T(&bv, 1);
  i1 = bv->size[0];
  bv->size[0] = loop_ub;
  emxEnsureCapacity_real_T(bv, i1);
  for (i1 = 0; i1 < loop_ub; i1++) {
    bv->data[i1] = 0.0;
  }

  //  Generate the constraint matrix
  //
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // %%%%%%%%%%%%%%%%%   Constraints: A . vec(X) = b  %%%%%%%%%%%%%%%%%%%%%%%%%
  itrr = 0.0;

  //  Counter for rows in A constraint
  itra = 0U;

  //  Counter for entries in A constraint
  itrb = 0U;

  //  Counter for entries in b constraint
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_11
  //  The diagonal entries of X should be equal to the first diagonal entry
  for (b_i = 0; b_i <= m - 2; b_i++) {
    itrr++;
    itra++;
    Z0_n = static_cast<int>(itra) - 1;
    Ai->data[Z0_n] = itrr;
    Aj->data[Z0_n] = 1.0;
    Av->data[Z0_n] = 1.0;
    itra++;
    Ai->data[static_cast<int>(itra) - 1] = itrr;
    i1 = static_cast<int>(itra) - 1;
    Aj->data[i1] = ((static_cast<double>(b_i) + 2.0) - 1.0) * numElmAtot_tmp + (
      static_cast<double>(b_i) + 2.0);
    Av->data[i1] = -1.0;
  }

  //  Off-diagonal entries should be zero
  for (b_i = 0; b_i <= m - 2; b_i++) {
    i1 = m - b_i;
    for (loop_ub = 0; loop_ub <= i1 - 2; loop_ub++) {
      itrr++;
      itra++;
      Z0_n = static_cast<int>(itra) - 1;
      Ai->data[Z0_n] = itrr;
      Aj->data[Z0_n] = ((static_cast<double>(b_i) + 1.0) - 1.0) * numElmAtot_tmp
        + static_cast<double>(((static_cast<unsigned int>(b_i) + loop_ub) + 2U));
      Av->data[Z0_n] = 1.0;
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_12
  //  Diagonal entries should be 1
  for (b_i = 0; b_i < m; b_i++) {
    itrr++;
    itra++;
    Z0_n = static_cast<int>(itra) - 1;
    Ai->data[Z0_n] = itrr;
    Aj->data[Z0_n] = ((static_cast<double>(b_i) + 1.0) - 1.0) * numElmAtot_tmp +
      ((static_cast<double>(b_i) + 1.0) + static_cast<double>(m));
    Av->data[Z0_n] = 1.0;
    itrb++;
    Z0_m = static_cast<int>(itrb) - 1;
    bi->data[Z0_m] = itrr;
    bv->data[Z0_m] = 1.0;
  }

  //  Other entries should be 0
  for (b_i = 0; b_i < m; b_i++) {
    for (loop_ub = 0; loop_ub < m; loop_ub++) {
      if (b_i + 1 != loop_ub + 1) {
        itrr++;
        itra++;
        Z0_n = static_cast<int>(itra) - 1;
        Ai->data[Z0_n] = itrr;
        Aj->data[Z0_n] = ((static_cast<double>(b_i) + 1.0) - 1.0) *
          numElmAtot_tmp + ((static_cast<double>(loop_ub) + 1.0) + static_cast<
                            double>(m));
        Av->data[Z0_n] = 1.0;
      }
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_22
  //  Zero constraints due to the adjacency matrix
  i1 = qz->size[0];
  for (b_i = 0; b_i < i1; b_i++) {
    //  Term corresponding to row ii and column jj
    loop_ub = Qt->size[0];
    t7_m = QQ->size[0] * QQ->size[1];
    QQ->size[0] = Qt->size[0];
    QQ->size[1] = b_loop_ub + 1;
    emxEnsureCapacity_real_T(QQ, t7_m);
    for (t7_m = 0; t7_m <= b_loop_ub; t7_m++) {
      for (Z0_n = 0; Z0_n < loop_ub; Z0_n++) {
        QQ->data[Z0_n + QQ->size[0] * t7_m] = Qt->data[Z0_n + Qt->size[0] * (
          static_cast<int>(((static_cast<double>(jj->data[b_i]) - 1.0) + 1.0)) -
          1)] * U->data[(static_cast<int>(((qz->data[b_i] - 1.0) + 1.0)) +
                         U->size[0] * (dimkerAz + t7_m)) - 1];
      }
    }

    itrr++;
    for (Z0_m = 0; Z0_m < m; Z0_m++) {
      for (kj = 0; kj < m; kj++) {
        itra++;
        Z0_n = static_cast<int>(itra) - 1;
        Ai->data[Z0_n] = itrr;
        Aj->data[Z0_n] = ((static_cast<double>(m) + (static_cast<double>(Z0_m) +
          1.0)) - 1.0) * numElmAtot_tmp + (static_cast<double>(m) + (
          static_cast<double>(kj) + 1.0));
        Av->data[Z0_n] = QQ->data[Z0_m + QQ->size[0] * kj];
      }
    }
  }

  //  Trace of gain matrix must be the specified value in 'trVal'
  itrr++;
  for (b_i = 0; b_i < m; b_i++) {
    itra++;
    idxR = static_cast<double>(m) + (static_cast<double>(b_i) + 1.0);
    Z0_n = static_cast<int>(itra) - 1;
    Ai->data[Z0_n] = itrr;
    Aj->data[Z0_n] = (idxR - 1.0) * numElmAtot_tmp + idxR;
    Av->data[Z0_n] = 1.0;
  }

  itrb++;
  Z0_m = static_cast<int>(itrb) - 1;
  bi->data[Z0_m] = itrr;
  bv->data[Z0_m] = m;

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%% Symmetry
  i1 = m << 1;
  for (b_i = 0; b_i <= i1 - 2; b_i++) {
    t7_m = i1 - b_i;
    for (loop_ub = 0; loop_ub <= t7_m - 2; loop_ub++) {
      idxR = ((static_cast<double>(b_i) + 1.0) + 1.0) + static_cast<double>
        (loop_ub);

      //  Symmetric entries should be equal
      itrr++;
      itra++;
      Z0_n = static_cast<int>(itra) - 1;
      Ai->data[Z0_n] = itrr;
      Aj->data[Z0_n] = ((static_cast<double>(b_i) + 1.0) - 1.0) * numElmAtot_tmp
        + idxR;
      Av->data[Z0_n] = 1.0;
      itra++;
      Z0_n = static_cast<int>(itra) - 1;
      Ai->data[Z0_n] = itrr;
      Aj->data[Z0_n] = (idxR - 1.0) * numElmAtot_tmp + (static_cast<double>(b_i)
        + 1.0);
      Av->data[Z0_n] = -1.0;
    }
  }

  emxInit_real_T(&A_d, 1);

  //  Last element set to fix the size of b
  itrb++;
  Z0_m = static_cast<int>(itrb) - 1;
  bi->data[Z0_m] = itrr;
  bv->data[Z0_m] = 0.0;

  //  % Remove any additional entries
  //  Ai(itra+1:end) = [];
  //  Aj(itra+1:end) = [];
  //  Av(itra+1:end) = [];
  //
  //  bi(itrb+1:end) = [];
  //  bv(itrb+1:end) = [];
  //  Make sparse matrices
  b_sparse(Ai, Aj, Av, &expl_temp);
  i1 = A_d->size[0];
  A_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(A_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    A_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&A_colidx, 1);
  i1 = A_colidx->size[0];
  A_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(A_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    A_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&A_rowidx, 1);
  i1 = A_rowidx->size[0];
  A_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(A_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    A_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  A_m = expl_temp.m;
  Z0_m = expl_temp.n;
  i1 = qz->size[0];
  qz->size[0] = bi->size[0];
  emxEnsureCapacity_real_T(qz, i1);
  loop_ub = bi->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    qz->data[i1] = 1.0;
  }

  emxInit_real_T(&b_d, 1);
  b_sparse(bi, qz, bv, &expl_temp);
  i1 = b_d->size[0];
  b_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(b_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&b_colidx, 1);
  i1 = b_colidx->size[0];
  b_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(b_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&b_rowidx, 1);
  i1 = b_rowidx->size[0];
  b_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(b_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_real_T(&As_d, 1);
  b_m = expl_temp.m;
  b_n = expl_temp.n;

  //  Size of optimization variable
  //  ADMM algorithm--full eigendecomposition
  sparse_transpose(A_d, A_colidx, A_rowidx, A_m, Z0_m, &expl_temp);
  i1 = As_d->size[0];
  As_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(As_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    As_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&As_colidx, 1);
  i1 = As_colidx->size[0];
  As_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(As_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    As_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&As_rowidx, 1);
  i1 = As_rowidx->size[0];
  As_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(As_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    As_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_real_T(&AAs_d, 1);
  As_m = expl_temp.m;

  //  Dual operator
  sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, As_d, As_colidx, As_rowidx,
                expl_temp.n, &expl_temp);
  i1 = AAs_d->size[0];
  AAs_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(AAs_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    AAs_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&AAs_colidx, 1);
  i1 = AAs_colidx->size[0];
  AAs_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(AAs_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    AAs_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&AAs_rowidx, 1);
  i1 = AAs_rowidx->size[0];
  AAs_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(AAs_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    AAs_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  AAs_m = expl_temp.m;
  AAs_n = expl_temp.n;

  //  Penalty
  //  Precision for positive eig vals
  //  Stop criteria
  //  Threshold based on change in X updates
  //  Percentage threshold based on the trace value. If trace of X2 reaches within the specified percentage, the algorithm stops. 
  //  Maximum # of iterations
  //  Initialize:
  sparse_horzcat(I0_d, I0_colidx, I0_rowidx, I0_m, I0_n, I0_d, I0_colidx,
                 I0_rowidx, I0_m, I0_n, &expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = expl_temp.d->data[i1];
  }

  i1 = t7_colidx->size[0];
  t7_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t7_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = t7_rowidx->size[0];
  t7_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t7_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  t7_m = expl_temp.m;
  t7_n = expl_temp.n;
  sparse_horzcat(I0_d, I0_colidx, I0_rowidx, I0_m, I0_n, I0_d, I0_colidx,
                 I0_rowidx, I0_m, I0_n, &expl_temp);
  i1 = Aj->size[0];
  Aj->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Aj, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Aj->data[i1] = expl_temp.d->data[i1];
  }

  i1 = t8_colidx->size[0];
  t8_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t8_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t8_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = t8_rowidx->size[0];
  t8_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t8_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t8_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  emxInit_real_T(&X_d, 1);
  sparse_vertcat(Ai, t7_colidx, t7_rowidx, t7_m, t7_n, Aj, t8_colidx, t8_rowidx,
                 expl_temp.m, expl_temp.n, &expl_temp);
  i1 = X_d->size[0];
  X_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(X_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    X_d->data[i1] = expl_temp.d->data[i1];
  }

  emxInit_int32_T(&X_colidx, 1);
  i1 = X_colidx->size[0];
  X_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(X_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    X_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  emxInit_int32_T(&X_rowidx, 1);
  i1 = X_rowidx->size[0];
  X_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(X_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    X_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  X_m = expl_temp.m;
  X_n = expl_temp.n;
  b_i = 0;
  emxInit_creal_T(&dd, 1);
  emxInit_creal_T(&V, 2);
  emxInit_creal_T(&D, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_int32_T(&r2, 1);
  emxInit_creal_T(&y, 2);
  emxInit_creal_T(&b, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_real_T(&c_y, 2);
  emxInit_int32_T(&t9_colidx, 1);
  emxInit_int32_T(&t9_rowidx, 1);
  d_emxInitStruct_coder_internal_(&b_expl_temp);
  emxInit_creal_T(&b_dd, 1);
  emxInit_real_T(&b_D, 2);
  exitg1 = false;
  while ((!exitg1) && (b_i < 10)) {
    // %%%%%% Update for y
    sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
    i1 = I0_d->size[0];
    I0_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(I0_d, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      I0_d->data[i1] = expl_temp.d->data[i1];
    }

    i1 = I0_colidx->size[0];
    I0_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(I0_colidx, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      I0_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = I0_rowidx->size[0];
    I0_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(I0_rowidx, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      I0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    I0_m = expl_temp.m;
    I0_n = expl_temp.n;
    sparse_times(b_d, b_colidx, b_rowidx, b_m, b_n, &expl_temp);
    i1 = bi->size[0];
    bi->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(bi, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      bi->data[i1] = expl_temp.d->data[i1];
    }

    i1 = Z0_colidx->size[0];
    Z0_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(Z0_colidx, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Z0_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = Z0_rowidx->size[0];
    Z0_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(Z0_rowidx, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Z0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    sparse_minus(C_d, C_colidx, C_rowidx, Z_d, Z_colidx, Z_rowidx, Z_m, Z_n, Ai,
                 t7_colidx, t7_rowidx, &t7_m, &t7_n);
    sparse_minus(Ai, t7_colidx, t7_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m,
                 I0_n, Aj, t8_colidx, t8_rowidx, &Z0_m, &t7_n);
    vec(Aj, t8_colidx, t8_rowidx, Z0_m, t7_n, bv, t9_colidx, t9_rowidx, &kj);
    b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, bv, t9_colidx, t9_rowidx,
                    &b_expl_temp);
    i1 = Ai->size[0];
    Ai->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    loop_ub = b_expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Ai->data[i1] = b_expl_temp.d->data[i1];
    }

    i1 = t7_colidx->size[0];
    t7_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t7_colidx, i1);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_colidx->data[i1] = b_expl_temp.colidx->data[i1];
    }

    i1 = t7_rowidx->size[0];
    t7_rowidx->size[0] = b_expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t7_rowidx, i1);
    loop_ub = b_expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_rowidx->data[i1] = b_expl_temp.rowidx->data[i1];
    }

    sparse_plus(Ai, t7_colidx, t7_rowidx, bi, Z0_colidx, Z0_rowidx, expl_temp.m,
                bv, t9_colidx, t9_rowidx, &kj);
    sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, bv, t9_colidx,
                    t9_rowidx, kj, &b_expl_temp);
    i1 = Ai->size[0];
    Ai->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    loop_ub = b_expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Ai->data[i1] = b_expl_temp.d->data[i1];
    }

    i1 = t7_colidx->size[0];
    t7_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t7_colidx, i1);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_colidx->data[i1] = b_expl_temp.colidx->data[i1];
    }

    i1 = t7_rowidx->size[0];
    t7_rowidx->size[0] = b_expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t7_rowidx, i1);
    loop_ub = b_expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_rowidx->data[i1] = b_expl_temp.rowidx->data[i1];
    }

    // %%%%%% Update for S
    sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
    i1 = I0_d->size[0];
    I0_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(I0_d, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      I0_d->data[i1] = expl_temp.d->data[i1];
    }

    i1 = I0_colidx->size[0];
    I0_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(I0_colidx, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      I0_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = I0_rowidx->size[0];
    I0_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(I0_rowidx, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      I0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    I0_m = expl_temp.m;
    I0_n = expl_temp.n;
    b_sparse_mtimes(As_d, As_colidx, As_rowidx, As_m, Ai, t7_colidx, t7_rowidx,
                    &b_expl_temp);
    i1 = bv->size[0];
    bv->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(bv, i1);
    loop_ub = b_expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      bv->data[i1] = b_expl_temp.d->data[i1];
    }

    i1 = t9_colidx->size[0];
    t9_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t9_colidx, i1);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t9_colidx->data[i1] = b_expl_temp.colidx->data[i1];
    }

    i1 = t9_rowidx->size[0];
    t9_rowidx->size[0] = b_expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t9_rowidx, i1);
    loop_ub = b_expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t9_rowidx->data[i1] = b_expl_temp.rowidx->data[i1];
    }

    sizX[0] = numElmAtot_tmp;
    sizX[1] = numElmAtot_tmp;
    sparse_reshape(bv, t9_colidx, t9_rowidx, sizX, &expl_temp);
    i1 = Ai->size[0];
    Ai->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Ai->data[i1] = expl_temp.d->data[i1];
    }

    i1 = t7_colidx->size[0];
    t7_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t7_colidx, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = t7_rowidx->size[0];
    t7_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t7_rowidx, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    sparse_minus(C_d, C_colidx, C_rowidx, Ai, t7_colidx, t7_rowidx, expl_temp.m,
                 expl_temp.n, Aj, t8_colidx, t8_rowidx, &Z0_m, &t7_n);
    sparse_minus(Aj, t8_colidx, t8_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m,
                 I0_n, qz, ii, jj, &Z0_m, &t7_m);
    sparse_transpose(qz, ii, jj, Z0_m, t7_m, &expl_temp);
    i1 = Ai->size[0];
    Ai->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Ai->data[i1] = expl_temp.d->data[i1];
    }

    i1 = t7_colidx->size[0];
    t7_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t7_colidx, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = t7_rowidx->size[0];
    t7_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t7_rowidx, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t7_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    b_sparse_plus(qz, ii, jj, Ai, t7_colidx, t7_rowidx, expl_temp.m, expl_temp.n,
                  Aj, t8_colidx, t8_rowidx, &Z0_m, &t7_n);
    sparse_rdivide(Aj, t8_colidx, t8_rowidx, Z0_m, t7_n, &expl_temp);
    i1 = qz->size[0];
    qz->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(qz, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      qz->data[i1] = expl_temp.d->data[i1];
    }

    i1 = ii->size[0];
    ii->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(ii, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      ii->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = jj->size[0];
    jj->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(jj, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      jj->data[i1] = expl_temp.rowidx->data[i1];
    }

    sparse_full(qz, ii, jj, expl_temp.m, expl_temp.n, b_D);
    eig(b_D, V, D);
    c_diag(D, dd);
    Z0_n = dd->size[0] - 1;
    t7_m = 0;
    for (Z0_m = 0; Z0_m <= Z0_n; Z0_m++) {
      if (dd->data[Z0_m].re > 1.0E-5) {
        t7_m++;
      }
    }

    i1 = r1->size[0];
    r1->size[0] = t7_m;
    emxEnsureCapacity_int32_T(r1, i1);
    kj = 0;
    I0_m = dd->size[0] - 1;
    t7_m = 0;
    for (Z0_m = 0; Z0_m <= Z0_n; Z0_m++) {
      if (dd->data[Z0_m].re > 1.0E-5) {
        r1->data[kj] = Z0_m + 1;
        kj++;
        t7_m++;
      }
    }

    i1 = r2->size[0];
    r2->size[0] = t7_m;
    emxEnsureCapacity_int32_T(r2, i1);
    kj = 0;
    for (Z0_m = 0; Z0_m <= I0_m; Z0_m++) {
      if (dd->data[Z0_m].re > 1.0E-5) {
        r2->data[kj] = Z0_m + 1;
        kj++;
      }
    }

    loop_ub = V->size[0];
    i1 = D->size[0] * D->size[1];
    D->size[0] = V->size[0];
    D->size[1] = r2->size[0];
    emxEnsureCapacity_creal_T(D, i1);
    Z0_m = r2->size[0];
    for (i1 = 0; i1 < Z0_m; i1++) {
      for (t7_m = 0; t7_m < loop_ub; t7_m++) {
        D->data[t7_m + D->size[0] * i1] = V->data[t7_m + V->size[0] * (r2->
          data[i1] - 1)];
      }
    }

    i1 = b_dd->size[0];
    b_dd->size[0] = r2->size[0];
    emxEnsureCapacity_creal_T(b_dd, i1);
    loop_ub = r2->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_dd->data[i1] = dd->data[r2->data[i1] - 1];
    }

    d_diag(b_dd, b);
    if ((r2->size[0] == 1) || (b->size[0] == 1)) {
      i1 = y->size[0] * y->size[1];
      y->size[0] = D->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity_creal_T(y, i1);
      loop_ub = D->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        Z0_m = b->size[1];
        for (t7_m = 0; t7_m < Z0_m; t7_m++) {
          y->data[i1 + y->size[0] * t7_m].re = 0.0;
          y->data[i1 + y->size[0] * t7_m].im = 0.0;
          kj = D->size[1];
          for (Z0_n = 0; Z0_n < kj; Z0_n++) {
            y->data[i1 + y->size[0] * t7_m].re += D->data[i1 + D->size[0] * Z0_n]
              .re * b->data[Z0_n + b->size[0] * t7_m].re - D->data[i1 + D->size
              [0] * Z0_n].im * b->data[Z0_n + b->size[0] * t7_m].im;
            y->data[i1 + y->size[0] * t7_m].im += D->data[i1 + D->size[0] * Z0_n]
              .re * b->data[Z0_n + b->size[0] * t7_m].im + D->data[i1 + D->size
              [0] * Z0_n].im * b->data[Z0_n + b->size[0] * t7_m].re;
          }
        }
      }
    } else {
      mtimes(D, b, y);
    }

    loop_ub = V->size[0];
    i1 = b->size[0] * b->size[1];
    b->size[0] = r1->size[0];
    b->size[1] = V->size[0];
    emxEnsureCapacity_creal_T(b, i1);
    for (i1 = 0; i1 < loop_ub; i1++) {
      Z0_m = r1->size[0];
      for (t7_m = 0; t7_m < Z0_m; t7_m++) {
        b->data[t7_m + b->size[0] * i1] = V->data[i1 + V->size[0] * (r1->
          data[t7_m] - 1)];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i1 = D->size[0] * D->size[1];
      D->size[0] = y->size[0];
      D->size[1] = b->size[1];
      emxEnsureCapacity_creal_T(D, i1);
      loop_ub = y->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        Z0_m = b->size[1];
        for (t7_m = 0; t7_m < Z0_m; t7_m++) {
          D->data[i1 + D->size[0] * t7_m].re = 0.0;
          D->data[i1 + D->size[0] * t7_m].im = 0.0;
          kj = y->size[1];
          for (Z0_n = 0; Z0_n < kj; Z0_n++) {
            D->data[i1 + D->size[0] * t7_m].re += y->data[i1 + y->size[0] * Z0_n]
              .re * b->data[Z0_n + b->size[0] * t7_m].re - y->data[i1 + y->size
              [0] * Z0_n].im * b->data[Z0_n + b->size[0] * t7_m].im;
            D->data[i1 + D->size[0] * t7_m].im += y->data[i1 + y->size[0] * Z0_n]
              .re * b->data[Z0_n + b->size[0] * t7_m].im + y->data[i1 + y->size
              [0] * Z0_n].im * b->data[Z0_n + b->size[0] * t7_m].re;
          }
        }
      }
    } else {
      mtimes(y, b, D);
    }

    i1 = b_D->size[0] * b_D->size[1];
    b_D->size[0] = D->size[0];
    b_D->size[1] = D->size[1];
    emxEnsureCapacity_real_T(b_D, i1);
    loop_ub = D->size[0] * D->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_D->data[i1] = D->data[i1].re;
    }

    c_sparse(b_D, Z_d, Z_colidx, Z_rowidx, &Z_m, &Z_n);

    // %%%%%% Update for X
    i1 = bi->size[0];
    bi->size[0] = X_d->size[0];
    emxEnsureCapacity_real_T(bi, i1);
    loop_ub = X_d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      bi->data[i1] = X_d->data[i1];
    }

    i1 = Z0_colidx->size[0];
    Z0_colidx->size[0] = X_colidx->size[0];
    emxEnsureCapacity_int32_T(Z0_colidx, i1);
    loop_ub = X_colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Z0_colidx->data[i1] = X_colidx->data[i1];
    }

    i1 = Z0_rowidx->size[0];
    Z0_rowidx->size[0] = X_rowidx->size[0];
    emxEnsureCapacity_int32_T(Z0_rowidx, i1);
    loop_ub = X_rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      Z0_rowidx->data[i1] = X_rowidx->data[i1];
    }

    Z0_m = X_m;
    Z0_n = X_n;
    sparse_minus(Z_d, Z_colidx, Z_rowidx, qz, ii, jj, expl_temp.m, expl_temp.n,
                 Ai, t7_colidx, t7_rowidx, &t7_m, &t7_n);
    sparse_times(Ai, t7_colidx, t7_rowidx, t7_m, t7_n, &expl_temp);
    i1 = X_d->size[0];
    X_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(X_d, i1);
    loop_ub = expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      X_d->data[i1] = expl_temp.d->data[i1];
    }

    i1 = X_colidx->size[0];
    X_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(X_colidx, i1);
    loop_ub = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      X_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = X_rowidx->size[0];
    X_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(X_rowidx, i1);
    loop_ub = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      X_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    X_m = expl_temp.m;
    X_n = expl_temp.n;

    // %%%%%% Stop criteria
    b_sparse_parenReference(bi, Z0_colidx, Z0_rowidx, Z0_m, Z0_n, bv, t9_colidx,
      t9_rowidx, &kj);
    b_sparse_parenReference(X_d, X_colidx, X_rowidx, expl_temp.m, expl_temp.n,
      Ai, t7_colidx, t7_rowidx, &Z0_m);
    b_sparse_minus(bv, t9_colidx, t9_rowidx, Ai, t7_colidx, t7_rowidx, Z0_m, qz,
                   ii, jj, &I0_m);
    sparse_abs(qz, ii, jj, I0_m, &b_expl_temp);
    i1 = bv->size[0];
    bv->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(bv, i1);
    loop_ub = b_expl_temp.d->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      bv->data[i1] = b_expl_temp.d->data[i1];
    }

    i1 = t9_colidx->size[0];
    t9_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t9_colidx, i1);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      t9_colidx->data[i1] = b_expl_temp.colidx->data[i1];
    }

    sum(bv, t9_colidx, b_expl_temp.m, qz, jj, t7_colidx);
    sparse_lt(qz, jj, t11_d, ii, t7_colidx);
    if (b_sparse_full(t11_d, ii)) {
      //  change in X is small
      exitg1 = true;
    } else {
      //  trace of sparse matrix for MATLAB Coder
      if (expl_temp.m < m + 1) {
        b_y->size[0] = 1;
        b_y->size[1] = 0;
      } else {
        i1 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        loop_ub = expl_temp.m - m;
        b_y->size[1] = loop_ub;
        emxEnsureCapacity_real_T(b_y, i1);
        for (i1 = 0; i1 < loop_ub; i1++) {
          b_y->data[i1] = (m + i1) + 1;
        }
      }

      if (expl_temp.n < m + 1) {
        c_y->size[0] = 1;
        c_y->size[1] = 0;
      } else {
        i1 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = 1;
        loop_ub = expl_temp.n - m;
        c_y->size[1] = loop_ub;
        emxEnsureCapacity_real_T(c_y, i1);
        for (i1 = 0; i1 < loop_ub; i1++) {
          c_y->data[i1] = (m + i1) + 1;
        }
      }

      c_sparse_parenReference(X_d, X_colidx, X_rowidx, b_y, c_y, Ai, t7_colidx,
        t7_rowidx, &t7_m, &t7_n);
      sparse_diag(Ai, t7_colidx, t7_rowidx, t7_m, t7_n, bv, t9_colidx, t9_rowidx,
                  &kj);
      sum(bv, t9_colidx, kj, qz, jj, t7_colidx);
      if (std::abs(c_sparse_full(qz, jj) - static_cast<double>(m)) /
          static_cast<double>(m) * 100.0 < 10.0) {
        //  trace of X is close to the desired value
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }

  emxFree_creal_T(&b_dd);
  emxFree_boolean_T(&t11_d);
  emxFree_real_T(&c_y);
  emxFree_real_T(&b_y);
  emxFree_creal_T(&b);
  emxFree_creal_T(&y);
  emxFree_int32_T(&r2);
  emxFree_int32_T(&r1);
  emxFree_creal_T(&D);
  emxFree_creal_T(&V);
  emxFree_creal_T(&dd);

  //  Set S=0 to project the final solution and ensure that it satisfies the linear constraints given by the adjacency matrix 
  b_sparse_times(Z_d, Z_colidx, Z_rowidx, Z_m, Z_n, &expl_temp);
  i1 = Av->size[0];
  Av->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Av, i1);
  loop_ub = expl_temp.d->size[0];
  emxFree_real_T(&Z_d);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Av->data[i1] = expl_temp.d->data[i1];
  }

  i1 = Z_colidx->size[0];
  Z_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(Z_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Z_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = Z_rowidx->size[0];
  Z_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(Z_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Z_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  Z0_m = expl_temp.m;
  kj = expl_temp.n;
  sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
  i1 = I0_d->size[0];
  I0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(I0_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_d->data[i1] = expl_temp.d->data[i1];
  }

  i1 = I0_colidx->size[0];
  I0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(I0_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = I0_rowidx->size[0];
  I0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(I0_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  I0_m = expl_temp.m;
  I0_n = expl_temp.n;
  sparse_times(b_d, b_colidx, b_rowidx, b_m, b_n, &expl_temp);
  i1 = bi->size[0];
  bi->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(bi, i1);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&b_rowidx);
  emxFree_int32_T(&b_colidx);
  emxFree_real_T(&b_d);
  for (i1 = 0; i1 < loop_ub; i1++) {
    bi->data[i1] = expl_temp.d->data[i1];
  }

  i1 = Z0_colidx->size[0];
  Z0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(Z0_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Z0_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = Z0_rowidx->size[0];
  Z0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(Z0_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    Z0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  sparse_minus(C_d, C_colidx, C_rowidx, Av, Z_colidx, Z_rowidx, Z0_m, kj, Ai,
               t7_colidx, t7_rowidx, &t7_m, &t7_n);
  sparse_minus(Ai, t7_colidx, t7_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m, I0_n,
               Aj, t8_colidx, t8_rowidx, &Z0_m, &t7_n);
  vec(Aj, t8_colidx, t8_rowidx, Z0_m, t7_n, bv, t9_colidx, t9_rowidx, &kj);
  b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, bv, t9_colidx, t9_rowidx,
                  &b_expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = b_expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  loop_ub = b_expl_temp.d->size[0];
  emxFree_int32_T(&A_rowidx);
  emxFree_int32_T(&A_colidx);
  emxFree_real_T(&A_d);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = b_expl_temp.d->data[i1];
  }

  i1 = t7_colidx->size[0];
  t7_colidx->size[0] = b_expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t7_colidx, i1);
  loop_ub = b_expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_colidx->data[i1] = b_expl_temp.colidx->data[i1];
  }

  i1 = t7_rowidx->size[0];
  t7_rowidx->size[0] = b_expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t7_rowidx, i1);
  loop_ub = b_expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_rowidx->data[i1] = b_expl_temp.rowidx->data[i1];
  }

  sparse_plus(Ai, t7_colidx, t7_rowidx, bi, Z0_colidx, Z0_rowidx, expl_temp.m,
              bv, t9_colidx, t9_rowidx, &kj);
  sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, bv, t9_colidx,
                  t9_rowidx, kj, &b_expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = b_expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  loop_ub = b_expl_temp.d->size[0];
  emxFree_int32_T(&AAs_rowidx);
  emxFree_int32_T(&AAs_colidx);
  emxFree_real_T(&AAs_d);
  emxFree_real_T(&bi);
  emxFree_int32_T(&Z0_rowidx);
  emxFree_int32_T(&Z0_colidx);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = b_expl_temp.d->data[i1];
  }

  i1 = t7_colidx->size[0];
  t7_colidx->size[0] = b_expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t7_colidx, i1);
  loop_ub = b_expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_colidx->data[i1] = b_expl_temp.colidx->data[i1];
  }

  i1 = t7_rowidx->size[0];
  t7_rowidx->size[0] = b_expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t7_rowidx, i1);
  loop_ub = b_expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_rowidx->data[i1] = b_expl_temp.rowidx->data[i1];
  }

  sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
  i1 = I0_d->size[0];
  I0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(I0_d, i1);
  loop_ub = expl_temp.d->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_d->data[i1] = expl_temp.d->data[i1];
  }

  i1 = I0_colidx->size[0];
  I0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(I0_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = I0_rowidx->size[0];
  I0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(I0_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    I0_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  I0_m = expl_temp.m;
  I0_n = expl_temp.n;
  b_sparse_mtimes(As_d, As_colidx, As_rowidx, As_m, Ai, t7_colidx, t7_rowidx,
                  &b_expl_temp);
  i1 = bv->size[0];
  bv->size[0] = b_expl_temp.d->size[0];
  emxEnsureCapacity_real_T(bv, i1);
  loop_ub = b_expl_temp.d->size[0];
  emxFree_int32_T(&As_rowidx);
  emxFree_int32_T(&As_colidx);
  emxFree_real_T(&As_d);
  for (i1 = 0; i1 < loop_ub; i1++) {
    bv->data[i1] = b_expl_temp.d->data[i1];
  }

  i1 = t9_colidx->size[0];
  t9_colidx->size[0] = b_expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t9_colidx, i1);
  loop_ub = b_expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t9_colidx->data[i1] = b_expl_temp.colidx->data[i1];
  }

  i1 = t9_rowidx->size[0];
  t9_rowidx->size[0] = b_expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t9_rowidx, i1);
  loop_ub = b_expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t9_rowidx->data[i1] = b_expl_temp.rowidx->data[i1];
  }

  d_emxFreeStruct_coder_internal_(&b_expl_temp);
  sizX[0] = numElmAtot_tmp;
  sizX[1] = numElmAtot_tmp;
  sparse_reshape(bv, t9_colidx, t9_rowidx, sizX, &expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&t9_rowidx);
  emxFree_int32_T(&t9_colidx);
  emxFree_real_T(&bv);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = expl_temp.d->data[i1];
  }

  i1 = t7_colidx->size[0];
  t7_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t7_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = t7_rowidx->size[0];
  t7_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t7_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  sparse_minus(C_d, C_colidx, C_rowidx, Ai, t7_colidx, t7_rowidx, expl_temp.m,
               expl_temp.n, Aj, t8_colidx, t8_rowidx, &Z0_m, &t7_n);
  sparse_minus(Aj, t8_colidx, t8_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m, I0_n,
               qz, ii, jj, &Z0_m, &t7_m);
  sparse_transpose(qz, ii, jj, Z0_m, t7_m, &expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&C_rowidx);
  emxFree_int32_T(&C_colidx);
  emxFree_real_T(&C_d);
  emxFree_int32_T(&I0_rowidx);
  emxFree_int32_T(&I0_colidx);
  emxFree_real_T(&I0_d);
  for (i1 = 0; i1 < loop_ub; i1++) {
    Ai->data[i1] = expl_temp.d->data[i1];
  }

  i1 = t7_colidx->size[0];
  t7_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t7_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = t7_rowidx->size[0];
  t7_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t7_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t7_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  b_sparse_plus(qz, ii, jj, Ai, t7_colidx, t7_rowidx, expl_temp.m, expl_temp.n,
                Aj, t8_colidx, t8_rowidx, &Z0_m, &t7_n);
  sparse_rdivide(Aj, t8_colidx, t8_rowidx, Z0_m, t7_n, &expl_temp);
  i1 = qz->size[0];
  qz->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(qz, i1);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&t8_rowidx);
  emxFree_int32_T(&t8_colidx);
  emxFree_real_T(&Aj);
  for (i1 = 0; i1 < loop_ub; i1++) {
    qz->data[i1] = expl_temp.d->data[i1];
  }

  i1 = ii->size[0];
  ii->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(ii, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    ii->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = jj->size[0];
  jj->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(jj, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    jj->data[i1] = expl_temp.rowidx->data[i1];
  }

  sparse_minus(Av, Z_colidx, Z_rowidx, qz, ii, jj, expl_temp.m, expl_temp.n, Ai,
               t7_colidx, t7_rowidx, &t7_m, &t7_n);
  sparse_times(Ai, t7_colidx, t7_rowidx, t7_m, t7_n, &expl_temp);
  i1 = X_d->size[0];
  X_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(X_d, i1);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&t7_rowidx);
  emxFree_int32_T(&t7_colidx);
  emxFree_int32_T(&jj);
  emxFree_int32_T(&ii);
  emxFree_real_T(&Av);
  emxFree_real_T(&Ai);
  emxFree_int32_T(&Z_rowidx);
  emxFree_int32_T(&Z_colidx);
  emxFree_real_T(&qz);
  for (i1 = 0; i1 < loop_ub; i1++) {
    X_d->data[i1] = expl_temp.d->data[i1];
  }

  i1 = X_colidx->size[0];
  X_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(X_colidx, i1);
  loop_ub = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    X_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = X_rowidx->size[0];
  X_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(X_rowidx, i1);
  loop_ub = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    X_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  sparse_full(X_d, X_colidx, X_rowidx, expl_temp.m, expl_temp.n, QQ);

  //  Transform X from sparse to full representation
  c_emxFreeStruct_coder_internal_(&expl_temp);
  emxFree_int32_T(&X_rowidx);
  emxFree_int32_T(&X_colidx);
  emxFree_real_T(&X_d);
  if (m + 1 > QQ->size[0]) {
    i1 = 0;
    t7_m = 0;
  } else {
    i1 = m;
    t7_m = QQ->size[0];
  }

  if (m + 1 > QQ->size[1]) {
    m = 0;
    Z0_n = 0;
  } else {
    Z0_n = QQ->size[1];
  }

  //  The component of X corresponding to the gain matrix
  loop_ub = t7_m - i1;
  t7_m = b_D->size[0] * b_D->size[1];
  b_D->size[0] = loop_ub;
  Z0_m = Z0_n - m;
  b_D->size[1] = Z0_m;
  emxEnsureCapacity_real_T(b_D, t7_m);
  for (t7_m = 0; t7_m < Z0_m; t7_m++) {
    for (Z0_n = 0; Z0_n < loop_ub; Z0_n++) {
      b_D->data[Z0_n + b_D->size[0] * t7_m] = -QQ->data[(i1 + Z0_n) + QQ->size[0]
        * (m + t7_m)];
    }
  }

  i1 = QQ->size[0] * QQ->size[1];
  QQ->size[0] = b_D->size[0];
  QQ->size[1] = b_D->size[1];
  emxEnsureCapacity_real_T(QQ, i1);
  loop_ub = b_D->size[0] * b_D->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    QQ->data[i1] = b_D->data[i1];
  }

  if ((i == 1) || (QQ->size[0] == 1)) {
    loop_ub = U->size[0];
    i = b_D->size[0] * b_D->size[1];
    b_D->size[0] = U->size[0];
    b_D->size[1] = QQ->size[1];
    emxEnsureCapacity_real_T(b_D, i);
    for (i = 0; i < loop_ub; i++) {
      Z0_m = QQ->size[1];
      for (i1 = 0; i1 < Z0_m; i1++) {
        b_D->data[i + b_D->size[0] * i1] = 0.0;
        for (t7_m = 0; t7_m <= b_loop_ub; t7_m++) {
          b_D->data[i + b_D->size[0] * i1] += U->data[i + U->size[0] * (dimkerAz
            + t7_m)] * QQ->data[t7_m + QQ->size[0] * i1];
        }
      }
    }
  } else {
    loop_ub = U->size[0] - 1;
    i1 = b_D->size[0] * b_D->size[1];
    b_D->size[0] = U->size[0];
    b_D->size[1] = i;
    emxEnsureCapacity_real_T(b_D, i1);
    for (i = 0; i <= b_loop_ub; i++) {
      for (i1 = 0; i1 <= loop_ub; i1++) {
        b_D->data[i1 + b_D->size[0] * i] = U->data[i1 + U->size[0] * (dimkerAz +
          i)];
      }
    }

    i = U->size[0] * U->size[1];
    U->size[0] = b_D->size[0];
    U->size[1] = b_D->size[1];
    emxEnsureCapacity_real_T(U, i);
    loop_ub = b_D->size[1];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = b_D->size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        U->data[i1 + U->size[0] * i] = b_D->data[i1 + b_D->size[0] * i];
      }
    }

    b_mtimes(U, QQ, b_D);
  }

  emxFree_real_T(&U);
  if ((b_D->size[1] == 1) || (Qt->size[0] == 1)) {
    i = QQ->size[0] * QQ->size[1];
    QQ->size[0] = b_D->size[0];
    QQ->size[1] = Qt->size[1];
    emxEnsureCapacity_real_T(QQ, i);
    loop_ub = b_D->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = Qt->size[1];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        QQ->data[i + QQ->size[0] * i1] = 0.0;
        Z0_m = b_D->size[1];
        for (t7_m = 0; t7_m < Z0_m; t7_m++) {
          QQ->data[i + QQ->size[0] * i1] += b_D->data[i + b_D->size[0] * t7_m] *
            Qt->data[t7_m + Qt->size[0] * i1];
        }
      }
    }
  } else {
    b_mtimes(b_D, Qt, QQ);
  }

  emxFree_real_T(&b_D);
  emxFree_real_T(&Qt);

  //  The formation gain matrix
  //  i
  //  eig(X2)
  //  trace(X2)
  //  3D formation gain matrix
  i = Aopt->size[0] * Aopt->size[1];
  Aopt->size[0] = 3 * adj->size[0];
  Aopt->size[1] = 3 * adj->size[0];
  emxEnsureCapacity_real_T(Aopt, i);
  loop_ub = 3 * adj->size[0] * (3 * adj->size[0]);
  for (i = 0; i < loop_ub; i++) {
    Aopt->data[i] = 0.0;
  }

  i = adj->size[0];
  for (b_i = 0; b_i < i; b_i++) {
    i1 = adj->size[0];
    for (loop_ub = 0; loop_ub < i1; loop_ub++) {
      //  Component corresponding to 2D formation
      idxR = 2.0 * (static_cast<double>(b_i) + 1.0);
      itrr = 2.0 * (static_cast<double>(loop_ub) + 1.0);
      numElmAtot_tmp = 3.0 * (static_cast<double>(b_i) + 1.0);
      d = 3.0 * (static_cast<double>(loop_ub) + 1.0);
      Z0_m = static_cast<int>((itrr + -1.0)) - 1;
      kj = static_cast<int>((d + -2.0)) - 1;
      Z0_n = static_cast<int>((idxR + -1.0)) - 1;
      t7_m = static_cast<int>((numElmAtot_tmp + -2.0)) - 1;
      Aopt->data[t7_m + Aopt->size[0] * kj] = Axy->data[Z0_n + Axy->size[0] *
        Z0_m];
      I0_m = static_cast<int>(idxR) - 1;
      t7_n = static_cast<int>((numElmAtot_tmp + -1.0)) - 1;
      Aopt->data[t7_n + Aopt->size[0] * kj] = Axy->data[I0_m + Axy->size[0] *
        Z0_m];
      Z0_m = static_cast<int>(itrr) - 1;
      kj = static_cast<int>((d + -1.0)) - 1;
      Aopt->data[t7_m + Aopt->size[0] * kj] = Axy->data[Z0_n + Axy->size[0] *
        Z0_m];
      Aopt->data[t7_n + Aopt->size[0] * kj] = Axy->data[I0_m + Axy->size[0] *
        Z0_m];

      //  Component corresponding to altitude
      Aopt->data[(static_cast<int>(numElmAtot_tmp) + Aopt->size[0] * (
        static_cast<int>(d) - 1)) - 1] = QQ->data[b_i + QQ->size[0] * loop_ub];
    }
  }

  emxFree_real_T(&QQ);
  emxFree_real_T(&Axy);

  //  ADMM algorithm--sparse eigendecomposition (use for very large-size problems n>1000) 
  //  As = A'; % Dual operator
  //  AAs = A * As;
  //
  //  mu = 1; % Penalty
  //  epsEig = 1e-5;  % Precision for positive eig vals
  //
  //  % Stop criteria
  //  thresh = 1e-4; % Threshold based on change in X updates
  //  threshTr = 10; % Percentage threshold based on the trace value. If trace of X2 reaches within the specified percentage, the algorithm stops. 
  //  maxItr = 10; % Maximum # of iterations
  //
  //  % Initialize:
  //  X = [I0 I0; I0 I0];
  //  S = Z;
  //  y = sparse(length(b),1);
  //
  //  for i = 1 : maxItr
  //
  //      %%%%%%% Update for y
  //      y = AAs \ (A * reshape(C - S - mu * X, [sizX^2,1]) + mu * b);
  //
  //
  //      %%%%%%% Update for S
  //      W = C - reshape(As*y, [sizX,sizX]) - mu * X;
  //      W = (W + W')/2;
  //
  //      % Find positive eigenvalues of W using an iterative technique
  //      V = zeros(sizX);
  //      d = zeros(sizX, 1);
  //
  //      % Eigenvalues are repeated in paris. We find the corresponding eigvector,  
  //      % then find the other eigenvector by rotating the first one 90 degrees 
  //      [vw1, dw1] = eigs(W,1,'largestreal', 'Tolerance',epsEig, 'FailureTreatment','keep', 'Display', false);  
  //
  //      numPos = 0; % Number of positive eigenvalues
  //      Wold = W;
  //      while dw1 > 10*epsEig %dw(1) > epsEig
  //          % Save positive eigenvalues and eigenvectors
  //          numPos = numPos + 1;
  //          d(numPos) = dw1;
  //          V(:, numPos) = vw1;
  //
  //          % Remove positive eigenvalue components from matrix W
  //          Wnew = Wold - dw1*(vw1*vw1');
  //
  //          % Find most positive eigenvalues of the new W matrix
  //          [vw1, dw1] = eigs(Wnew,1,'largestreal', 'Tolerance',epsEig, 'FailureTreatment','keep', 'Display', false);  
  //
  //          Wold = Wnew;
  //      end
  //
  //      S =  V(:,1:numPos) * diag(d(1:numPos)) * V(:,1:numPos).';
  //
  //      %%%%%%% Update for X
  //      Xold = X;
  //      X = sparse((S - W) / mu);
  //
  //
  //      %%%%%%% Stop criteria
  //      difX = sum(abs(Xold(:) - X(:)));
  //      if difX < thresh % change in X is small
  //          break
  //      end
  //
  //      trPerc = abs(trace( X(m+1:end, m+1:end) ) - trVal) / trVal * 100;
  //      if trPerc < threshTr % trace of X is close to the desired value
  //          break
  //      end
  //
  //  end
  //
  //  % Set S=0 to project the final solution and ensure that it satisfies the linear constraints given by the adjacency matrix 
  //  S = 0 * S;
  //  y = AAs \ (A * reshape(C - S - mu * X, [sizX^2,1]) + mu * b);
  //  W = C - reshape(As*y, [sizX,sizX]) - mu * X;
  //  W = (W + W') / 2;
  //  X = (S - W) / mu;
  //
  //  X = full(X); % Transform X from sparse to full representation
  //
  //  X2 = X(m+1:end, m+1:end); % The componenet of X corresponding to the gain matrix 
  //  Az = Q * (-X2) * Qt; % The formation gain matrix
  //  i
  //  eig(X2)
  //  trace(X2)
  //  Test solution
  //  eig(X2)
  //  eig(Az)
  //  % Enforce zero-gain constraint for non-neighbor agents
  //  Atrim = Az;
  //  for i = 1 : n
  //      for j = 1 : n
  //          if (i~=j) && ~adj(i,j)
  //              Atrim(2*i-1:2*i, 2*j-1:2*j) = zeros(2);
  //          end
  //      end
  //  end
  //
  //  eig(Atrim)
}

//
// File trailer for ADMMGainDesign3D.cpp
//
// [EOF]
//
