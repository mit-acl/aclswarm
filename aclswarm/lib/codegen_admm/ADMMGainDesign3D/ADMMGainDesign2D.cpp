//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign2D.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "ADMMGainDesign2D.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "colon.h"
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
#include "sum.h"
#include "svd.h"
#include "triu.h"
#include "vec.h"
#include "vertcat.h"
#include <cmath>

// Function Declarations
static int div_s32_floor(int numerator, int denominator);

// Function Definitions

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  bool quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator < 0) {
      absNumerator = ~static_cast<unsigned int>(numerator) + 1U;
    } else {
      absNumerator = static_cast<unsigned int>(numerator);
    }

    if (denominator < 0) {
      absDenominator = ~static_cast<unsigned int>(denominator) + 1U;
    } else {
      absDenominator = static_cast<unsigned int>(denominator);
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }

      quotient = -static_cast<int>(tempAbsQuotient);
    } else {
      quotient = static_cast<int>(tempAbsQuotient);
    }
  }

  return quotient;
}

//
// print out size information
// Arguments    : const emxArray_real_T *Qs
//                const emxArray_real_T *adj
//                emxArray_real_T *Aopt
// Return Type  : void
//
void ADMMGainDesign2D(const emxArray_real_T *Qs, const emxArray_real_T *adj,
                      emxArray_real_T *Aopt)
{
  emxArray_real_T *qsbar;
  int n;
  int m;
  int i;
  int loop_ub;
  int i1;
  int i2;
  int t2_n;
  emxArray_int32_T *one1;
  emxArray_real_T *one2;
  emxArray_real_T *b_Qs;
  int bi_idx_0;
  emxArray_real_T *U;
  emxArray_real_T *unusedU0;
  double unusedU1[16];
  unsigned int itra;
  emxArray_real_T *Q;
  int b_loop_ub;
  emxArray_real_T *Qt;
  emxArray_real_T *I0_d;
  coder_internal_sparse expl_temp;
  emxArray_int32_T *I0_colidx;
  emxArray_int32_T *I0_rowidx;
  emxArray_real_T *Z0_d;
  emxArray_int32_T *Z0_colidx;
  emxArray_int32_T *Z0_rowidx;
  emxArray_real_T *Z_d;
  emxArray_int32_T *Z_colidx;
  emxArray_int32_T *Z_rowidx;
  int I0_m;
  int I0_n;
  int Z0_n;
  int Z_m;
  int Z_n;
  emxArray_int32_T *t2_colidx;
  emxArray_int32_T *t2_rowidx;
  emxArray_real_T *Aj;
  int t2_m;
  emxArray_int32_T *t3_colidx;
  emxArray_int32_T *t3_rowidx;
  emxArray_real_T *C_d;
  emxArray_int32_T *C_colidx;
  emxArray_int32_T *C_rowidx;
  emxArray_boolean_T *S;
  double trVal;
  emxArray_real_T *QQ;
  emxArray_boolean_T *r;
  emxArray_boolean_T *t6_d;
  emxArray_int32_T *ii;
  double idxR;
  emxArray_real_T *Av;
  emxArray_real_T *bi;
  emxArray_real_T *bv;
  double itrr;
  unsigned int itrb;
  int b_i;
  int one2_tmp;
  emxArray_real_T *b;
  double sizX;
  double Aj_tmp;
  double b_Aj_tmp;
  double d;
  emxArray_real_T *A_d;
  emxArray_int32_T *A_colidx;
  emxArray_int32_T *A_rowidx;
  int A_m;
  emxArray_int32_T *b_colidx;
  emxArray_int32_T *b_rowidx;
  emxArray_real_T *As_d;
  int b_m;
  int b_n;
  emxArray_int32_T *As_colidx;
  emxArray_int32_T *As_rowidx;
  emxArray_real_T *AAs_d;
  emxArray_int32_T *AAs_colidx;
  emxArray_int32_T *AAs_rowidx;
  int AAs_m;
  int AAs_n;
  emxArray_real_T *X_d;
  emxArray_int32_T *X_colidx;
  emxArray_int32_T *X_rowidx;
  int X_m;
  int X_n;
  emxArray_creal_T *V;
  emxArray_creal_T *D;
  emxArray_creal_T *b_d;
  emxArray_int32_T *r1;
  emxArray_int32_T *r2;
  emxArray_creal_T *y;
  emxArray_creal_T *b_b;
  emxArray_real_T *b_y;
  emxArray_int32_T *t4_colidx;
  emxArray_int32_T *t4_rowidx;
  coder_internal_sparse_1 b_expl_temp;
  emxArray_creal_T *c_d;
  bool exitg1;
  double b_sizX[2];
  emxInit_real_T(&qsbar, 1);

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
  //  Make a vector out of formation points
  //  Number of agents
  n = adj->size[0];
  m = adj->size[0] - 2;

  //  Reduced number
  //  ambient dimension of problem
  //  90-degree rotated desired formation coordinates
  i = qsbar->size[0];
  qsbar->size[0] = 2 * adj->size[0];
  emxEnsureCapacity_real_T(qsbar, i);
  loop_ub = 2 * adj->size[0];
  for (i = 0; i < loop_ub; i++) {
    qsbar->data[i] = 0.0;
  }

  if (2 > (Qs->size[1] << 1)) {
    i = 0;
    i1 = 1;
  } else {
    i = 1;
    i1 = 2;
  }

  if (1 > (adj->size[0] << 1) - 1) {
    i2 = 1;
    t2_n = 1;
  } else {
    i2 = 2;
    t2_n = adj->size[0] << 1;
  }

  loop_ub = div_s32_floor(t2_n - 2, i2) + 1;
  for (t2_n = 0; t2_n < loop_ub; t2_n++) {
    qsbar->data[i2 * t2_n] = -Qs->data[i + i1 * t2_n];
  }

  if (1 > (Qs->size[1] << 1) - 1) {
    i = 1;
    i1 = 1;
  } else {
    i = 2;
    i1 = Qs->size[1] << 1;
  }

  if (2 > qsbar->size[0]) {
    i2 = 0;
    t2_n = 1;
  } else {
    i2 = 1;
    t2_n = 2;
  }

  loop_ub = div_s32_floor(i1 - 2, i);
  for (i1 = 0; i1 <= loop_ub; i1++) {
    qsbar->data[i2 + t2_n * i1] = Qs->data[i * i1];
  }

  emxInit_int32_T(&one1, 1);

  //  One vectors
  i = one1->size[0];
  one1->size[0] = 2 * adj->size[0];
  emxEnsureCapacity_int32_T(one1, i);
  loop_ub = 2 * adj->size[0];
  for (i = 0; i < loop_ub; i++) {
    one1->data[i] = 0;
  }

  if (1 > (adj->size[0] << 1) - 1) {
    i = 1;
    i1 = 1;
  } else {
    i = 2;
    i1 = adj->size[0] << 1;
  }

  loop_ub = div_s32_floor(i1 - 2, i) + 1;
  for (i1 = 0; i1 < loop_ub; i1++) {
    one1->data[i * i1] = 1;
  }

  emxInit_real_T(&one2, 1);
  i = one2->size[0];
  one2->size[0] = 2 * adj->size[0];
  emxEnsureCapacity_real_T(one2, i);
  loop_ub = 2 * adj->size[0];
  for (i = 0; i < loop_ub; i++) {
    one2->data[i] = 0.0;
  }

  if (2 > (adj->size[0] << 1)) {
    i = 0;
    i1 = 1;
    i2 = 0;
  } else {
    i = 1;
    i1 = 2;
    i2 = adj->size[0] << 1;
  }

  loop_ub = div_s32_floor((i2 - i) - 1, i1) + 1;
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i + i1 * i2] = 1.0;
  }

  emxInit_real_T(&b_Qs, 2);

  //  Get orthogonal complement of [z ones(n,1)]
  //  Desired kernel bases
  bi_idx_0 = Qs->size[1] << 1;
  i = b_Qs->size[0] * b_Qs->size[1];
  b_Qs->size[0] = bi_idx_0;
  b_Qs->size[1] = 4;
  emxEnsureCapacity_real_T(b_Qs, i);
  for (i = 0; i < bi_idx_0; i++) {
    b_Qs->data[i] = Qs->data[i];
  }

  loop_ub = qsbar->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_Qs->data[i + b_Qs->size[0]] = qsbar->data[i];
  }

  loop_ub = one1->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_Qs->data[i + b_Qs->size[0] * 2] = one1->data[i];
  }

  loop_ub = one2->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_Qs->data[i + b_Qs->size[0] * 3] = one2->data[i];
  }

  emxInit_real_T(&U, 2);
  emxInit_real_T(&unusedU0, 2);
  svd(b_Qs, U, unusedU0, unusedU1);
  itra = static_cast<unsigned int>(adj->size[0]) << 1;
  emxFree_real_T(&b_Qs);
  emxFree_real_T(&unusedU0);
  if (5U > itra) {
    i = 0;
    i1 = -1;
  } else {
    i = 4;
    i1 = static_cast<int>(itra) - 1;
  }

  emxInit_real_T(&Q, 2);
  loop_ub = U->size[0];
  i2 = Q->size[0] * Q->size[1];
  Q->size[0] = U->size[0];
  b_loop_ub = i1 - i;
  i1 = b_loop_ub + 1;
  Q->size[1] = i1;
  emxEnsureCapacity_real_T(Q, i2);
  for (i2 = 0; i2 <= b_loop_ub; i2++) {
    for (t2_n = 0; t2_n < loop_ub; t2_n++) {
      Q->data[t2_n + Q->size[0] * i2] = U->data[t2_n + U->size[0] * (i + i2)];
    }
  }

  emxInit_real_T(&Qt, 2);

  //  Bases for the range space
  loop_ub = U->size[0];
  i2 = Qt->size[0] * Qt->size[1];
  Qt->size[0] = i1;
  Qt->size[1] = U->size[0];
  emxEnsureCapacity_real_T(Qt, i2);
  for (i2 = 0; i2 < loop_ub; i2++) {
    for (t2_n = 0; t2_n <= b_loop_ub; t2_n++) {
      Qt->data[t2_n + Qt->size[0] * i2] = U->data[i2 + U->size[0] * (i + t2_n)];
    }
  }

  emxInit_real_T(&I0_d, 1);
  c_emxInitStruct_coder_internal_(&expl_temp);

  //  Q transpose
  //  Preallocate variables
  speye(2.0 * (static_cast<double>(adj->size[0]) - 2.0), &expl_temp);
  i2 = I0_d->size[0];
  I0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(I0_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_d->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&I0_colidx, 1);
  i2 = I0_colidx->size[0];
  I0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(I0_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&I0_rowidx, 1);
  i2 = I0_rowidx->size[0];
  I0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(I0_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_real_T(&Z0_d, 1);
  emxInit_int32_T(&Z0_colidx, 1);
  emxInit_int32_T(&Z0_rowidx, 1);
  emxInit_real_T(&Z_d, 1);
  emxInit_int32_T(&Z_colidx, 1);
  emxInit_int32_T(&Z_rowidx, 1);
  I0_m = expl_temp.m;
  I0_n = expl_temp.n;
  sparse(2.0 * (static_cast<double>(adj->size[0]) - 2.0), 2.0 * (static_cast<
          double>(adj->size[0]) - 2.0), Z0_d, Z0_colidx, Z0_rowidx, &bi_idx_0,
         &Z0_n);
  sparse(4.0 * (static_cast<double>(adj->size[0]) - 2.0), 4.0 * (static_cast<
          double>(adj->size[0]) - 2.0), Z_d, Z_colidx, Z_rowidx, &Z_m, &Z_n);

  //  Cost function's coefficient matrix: f = <C,X>
  sparse_horzcat(I0_d, I0_colidx, I0_rowidx, expl_temp.m, expl_temp.n, Z0_d,
                 Z0_colidx, Z0_rowidx, bi_idx_0, Z0_n, &expl_temp);
  i2 = one2->size[0];
  one2->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&t2_colidx, 1);
  i2 = t2_colidx->size[0];
  t2_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t2_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&t2_rowidx, 1);
  i2 = t2_rowidx->size[0];
  t2_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t2_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_real_T(&Aj, 1);
  t2_m = expl_temp.m;
  t2_n = expl_temp.n;
  sparse_horzcat(Z0_d, Z0_colidx, Z0_rowidx, bi_idx_0, Z0_n, Z0_d, Z0_colidx,
                 Z0_rowidx, bi_idx_0, Z0_n, &expl_temp);
  i2 = Aj->size[0];
  Aj->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Aj, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    Aj->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&t3_colidx, 1);
  i2 = t3_colidx->size[0];
  t3_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t3_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t3_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&t3_rowidx, 1);
  i2 = t3_rowidx->size[0];
  t3_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t3_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t3_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_real_T(&C_d, 1);
  sparse_vertcat(one2, t2_colidx, t2_rowidx, t2_m, t2_n, Aj, t3_colidx,
                 t3_rowidx, expl_temp.m, expl_temp.n, &expl_temp);
  i2 = C_d->size[0];
  C_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(C_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    C_d->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&C_colidx, 1);
  i2 = C_colidx->size[0];
  C_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(C_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    C_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&C_rowidx, 1);
  i2 = C_rowidx->size[0];
  C_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(C_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    C_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_boolean_T(&S, 2);

  //  Trace of gain matrix must be the specified value in 'trVal'
  trVal = 2.0 * (static_cast<double>(adj->size[0]) - 2.0);

  //  fixed value for trace
  // %%%%%%%%%%%%%%%%%%%%% Find total number of nonzero elements in A and b matrices 
  //  Number of elements in block [X]_11
  //  Number of elements in block [X]_12
  //  Zero-gain constraints for the given adjacency graph
  i2 = S->size[0] * S->size[1];
  S->size[0] = adj->size[0];
  S->size[1] = adj->size[1];
  emxEnsureCapacity_boolean_T(S, i2);
  loop_ub = adj->size[0] * adj->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    S->data[i2] = !(adj->data[i2] != 0.0);
  }

  emxInit_real_T(&QQ, 2);
  emxInit_boolean_T(&r, 2);
  emxInit_boolean_T(&t6_d, 1);

  //  Upper triangular part
  diag(S, t6_d);
  b_diag(t6_d, r);
  i2 = QQ->size[0] * QQ->size[1];
  QQ->size[0] = S->size[0];
  QQ->size[1] = S->size[1];
  emxEnsureCapacity_real_T(QQ, i2);
  loop_ub = S->size[0] * S->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    QQ->data[i2] = static_cast<double>(S->data[i2]) - static_cast<double>
      (r->data[i2]);
  }

  emxFree_boolean_T(&r);
  emxFree_boolean_T(&S);
  emxInit_int32_T(&ii, 1);
  triu(QQ);
  eml_find(QQ, ii, one1);
  i2 = qsbar->size[0];
  qsbar->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(qsbar, i2);
  loop_ub = ii->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    qsbar->data[i2] = ii->data[i2];
  }

  //  Find location of nonzero entries
  //  Number of constraints
  //  Number of elements in block [X]_22. Found "-m" empirically.
  idxR = 2.0 * (static_cast<double>(adj->size[0]) - 2.0);

  //  Number of elements for trace
  //  Number of elements for symmetry
  //  Number of elements for pinning down the b-vector
  //  Total number of elements
  // %%%%%%%%%%%%%%%%%%%%% Preallocate sparse matrices A & b
  //
  //  Constraint: A * vec(X) = b
  //
  //  Indices of A: entry [Ai(k), Aj(k)] takes value of Av(k)
  //  Each row of matrix A will represent a constraint
  loop_ub = static_cast<int>(((((((2.0 * (static_cast<double>(adj->size[0]) -
    2.0) - 1.0) * 2.0 + 2.0 * (static_cast<double>(adj->size[0]) - 2.0) * (2.0 *
    (static_cast<double>(adj->size[0]) - 2.0) - 1.0) / 2.0) + (2.0 * (
    static_cast<double>(adj->size[0]) - 2.0) + 2.0 * (static_cast<double>
    (adj->size[0]) - 2.0) * (2.0 * (static_cast<double>(adj->size[0]) - 2.0) -
    1.0))) + ((0.5 * (static_cast<double>(adj->size[0]) - 2.0) * ((static_cast<
    double>(adj->size[0]) - 2.0) + 1.0) * 4.0 + static_cast<double>(qsbar->size
    [0]) * (2.0 * (idxR * idxR))) - (static_cast<double>(adj->size[0]) - 2.0)))
    + 2.0 * (static_cast<double>(adj->size[0]) - 2.0)) + 4.0 * (static_cast<
    double>(adj->size[0]) - 2.0) * (4.0 * (static_cast<double>(adj->size[0]) -
    2.0) - 1.0)));
  i2 = one2->size[0];
  one2->size[0] = loop_ub;
  emxEnsureCapacity_real_T(one2, i2);
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = 0.0;
  }

  i2 = Aj->size[0];
  Aj->size[0] = loop_ub;
  emxEnsureCapacity_real_T(Aj, i2);
  for (i2 = 0; i2 < loop_ub; i2++) {
    Aj->data[i2] = 0.0;
  }

  emxInit_real_T(&Av, 1);
  i2 = Av->size[0];
  Av->size[0] = loop_ub;
  emxEnsureCapacity_real_T(Av, i2);
  for (i2 = 0; i2 < loop_ub; i2++) {
    Av->data[i2] = 0.0;
  }

  emxInit_real_T(&bi, 1);
  loop_ub = static_cast<int>(((2.0 * (static_cast<double>(adj->size[0]) - 2.0) +
    1.0) + 1.0));
  i2 = bi->size[0];
  bi->size[0] = loop_ub;
  emxEnsureCapacity_real_T(bi, i2);
  for (i2 = 0; i2 < loop_ub; i2++) {
    bi->data[i2] = 0.0;
  }

  emxInit_real_T(&bv, 1);
  i2 = bv->size[0];
  bv->size[0] = loop_ub;
  emxEnsureCapacity_real_T(bv, i2);
  for (i2 = 0; i2 < loop_ub; i2++) {
    bv->data[i2] = 0.0;
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
  i2 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i <= i2 - 2; b_i++) {
    itrr++;
    itra++;
    one2_tmp = static_cast<int>(itra) - 1;
    one2->data[one2_tmp] = itrr;
    Aj->data[one2_tmp] = 1.0;
    Av->data[one2_tmp] = 1.0;
    itra++;
    one2_tmp = static_cast<int>(itra) - 1;
    one2->data[one2_tmp] = itrr;
    Aj->data[one2_tmp] = ((static_cast<double>(b_i) + 2.0) - 1.0) * (4.0 * (
      static_cast<double>(n) - 2.0)) + (static_cast<double>(b_i) + 2.0);
    Av->data[static_cast<int>(itra) - 1] = -1.0;
  }

  //  Off-diagonal entries should be zero
  i2 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i <= i2 - 2; b_i++) {
    t2_n = (m << 1) - b_i;
    for (Z0_n = 0; Z0_n <= t2_n - 2; Z0_n++) {
      itrr++;
      itra++;
      one2->data[static_cast<int>(itra) - 1] = itrr;
      bi_idx_0 = static_cast<int>(itra) - 1;
      Aj->data[bi_idx_0] = ((static_cast<double>(b_i) + 1.0) - 1.0) * (4.0 * (
        static_cast<double>(n) - 2.0)) + (((static_cast<double>(b_i) + 1.0) +
        1.0) + static_cast<double>(Z0_n));
      Av->data[bi_idx_0] = 1.0;
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_12
  //  Diagonal entries should be 1
  i2 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i < i2; b_i++) {
    itrr++;
    itra++;
    one2_tmp = static_cast<int>(itra) - 1;
    one2->data[one2_tmp] = itrr;
    Aj->data[one2_tmp] = ((static_cast<double>(b_i) + 1.0) - 1.0) * (4.0 * (
      static_cast<double>(n) - 2.0)) + ((static_cast<double>(b_i) + 1.0) + 2.0 *
      static_cast<double>(m));
    Av->data[one2_tmp] = 1.0;
    itrb++;
    bi_idx_0 = static_cast<int>(itrb) - 1;
    bi->data[bi_idx_0] = itrr;
    bv->data[bi_idx_0] = 1.0;
  }

  //  Other entries should be 0
  i2 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i < i2; b_i++) {
    t2_n = m << 1;
    for (Z0_n = 0; Z0_n < t2_n; Z0_n++) {
      if (static_cast<unsigned int>(b_i) != static_cast<unsigned int>(Z0_n)) {
        itrr++;
        itra++;
        one2_tmp = static_cast<int>(itra) - 1;
        one2->data[one2_tmp] = itrr;
        Aj->data[one2_tmp] = ((static_cast<double>(b_i) + 1.0) - 1.0) * (4.0 * (
          static_cast<double>(n) - 2.0)) + ((static_cast<double>(Z0_n) + 1.0) +
          2.0 * static_cast<double>(m));
        Av->data[one2_tmp] = 1.0;
      }
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_22
  //  Scaled rotation matrix structure constraints
  i2 = adj->size[0];
  for (b_i = 0; b_i <= i2 - 3; b_i++) {
    t2_n = n - b_i;
    for (Z0_n = 0; Z0_n <= t2_n - 3; Z0_n++) {
      idxR = static_cast<double>((static_cast<unsigned int>(b_i) + Z0_n)) + 1.0;

      //  Diagonal entries should be equal
      itrr++;
      itra++;
      one2_tmp = static_cast<int>(itra) - 1;
      one2->data[one2_tmp] = itrr;
      sizX = 2.0 * static_cast<double>(m);
      Aj_tmp = sizX + 2.0 * ((static_cast<double>(b_i) + 1.0) - 1.0);
      b_Aj_tmp = sizX + 2.0 * (idxR - 1.0);
      d = 4.0 * (static_cast<double>(n) - 2.0);
      Aj->data[one2_tmp] = ((Aj_tmp + 1.0) - 1.0) * d + (b_Aj_tmp + 1.0);
      Av->data[static_cast<int>(itra) - 1] = 1.0;
      itra++;
      one2_tmp = static_cast<int>(itra) - 1;
      one2->data[one2_tmp] = itrr;
      Aj->data[one2_tmp] = ((Aj_tmp + 2.0) - 1.0) * d + (b_Aj_tmp + 2.0);
      Av->data[one2_tmp] = -1.0;

      //  Off-diagonal entries have same value with different sign
      if (static_cast<unsigned int>((b_i + 1)) == static_cast<unsigned int>(idxR))
      {
        itrr++;
        itra++;
        one2_tmp = static_cast<int>(itra) - 1;
        one2->data[one2_tmp] = itrr;
        Aj->data[one2_tmp] = (((2.0 * static_cast<double>(m) + 2.0 * ((
          static_cast<double>(b_i) + 1.0) - 1.0)) + 1.0) - 1.0) * (4.0 * (
          static_cast<double>(n) - 2.0)) + ((2.0 * static_cast<double>(m) + 2.0 *
          (idxR - 1.0)) + 2.0);
        Av->data[one2_tmp] = 1.0;
      } else {
        itrr++;
        itra++;
        one2_tmp = static_cast<int>(itra) - 1;
        one2->data[one2_tmp] = itrr;
        sizX += 2.0 * (idxR - 1.0);
        Aj->data[one2_tmp] = (((2.0 * static_cast<double>(m) + 2.0 * ((
          static_cast<double>(b_i) + 1.0) - 1.0)) + 1.0) - 1.0) * (4.0 * (
          static_cast<double>(n) - 2.0)) + (sizX + 2.0);
        Av->data[one2_tmp] = 1.0;
        itra++;
        one2_tmp = static_cast<int>(itra) - 1;
        one2->data[one2_tmp] = itrr;
        Aj->data[one2_tmp] = (((2.0 * static_cast<double>(m) + 2.0 * ((
          static_cast<double>(b_i) + 1.0) - 1.0)) + 2.0) - 1.0) * (4.0 * (
          static_cast<double>(n) - 2.0)) + (sizX + 1.0);
        Av->data[one2_tmp] = 1.0;
      }
    }
  }

  //  Zero constraints due to the adjacency matrix
  i2 = qsbar->size[0];
  emxInit_real_T(&b, 2);
  for (b_i = 0; b_i < i2; b_i++) {
    //  Diagonal terms of A_iijj, just do first column/row since a == a
    loop_ub = Qt->size[0];
    t2_n = static_cast<int>((2.0 * (static_cast<double>(one1->data[b_i]) - 1.0)
      + 1.0));
    t2_m = Z0_d->size[0];
    Z0_d->size[0] = Qt->size[0];
    emxEnsureCapacity_real_T(Z0_d, t2_m);
    for (t2_m = 0; t2_m < loop_ub; t2_m++) {
      Z0_d->data[t2_m] = Qt->data[t2_m + Qt->size[0] * (t2_n - 1)];
    }

    d = 2.0 * (qsbar->data[b_i] - 1.0);
    t2_m = static_cast<int>((d + 1.0));
    bi_idx_0 = b->size[0] * b->size[1];
    b->size[0] = 1;
    b->size[1] = b_loop_ub + 1;
    emxEnsureCapacity_real_T(b, bi_idx_0);
    for (bi_idx_0 = 0; bi_idx_0 <= b_loop_ub; bi_idx_0++) {
      b->data[bi_idx_0] = Q->data[(t2_m + Q->size[0] * bi_idx_0) - 1];
    }

    t2_m = QQ->size[0] * QQ->size[1];
    QQ->size[0] = Z0_d->size[0];
    loop_ub = b->size[1];
    QQ->size[1] = b->size[1];
    emxEnsureCapacity_real_T(QQ, t2_m);
    for (t2_m = 0; t2_m < loop_ub; t2_m++) {
      one2_tmp = Z0_d->size[0];
      for (bi_idx_0 = 0; bi_idx_0 < one2_tmp; bi_idx_0++) {
        QQ->data[bi_idx_0 + QQ->size[0] * t2_m] = Z0_d->data[bi_idx_0] * b->
          data[t2_m];
      }
    }

    itrr++;
    t2_m = m << 1;
    for (bi_idx_0 = 0; bi_idx_0 < t2_m; bi_idx_0++) {
      for (Z0_n = 0; Z0_n < t2_m; Z0_n++) {
        itra++;
        one2_tmp = static_cast<int>(itra) - 1;
        one2->data[one2_tmp] = itrr;
        sizX = 2.0 * static_cast<double>(m);
        Aj->data[one2_tmp] = ((sizX + (static_cast<double>(bi_idx_0) + 1.0)) -
                              1.0) * (4.0 * (static_cast<double>(n) - 2.0)) +
          (sizX + (static_cast<double>(Z0_n) + 1.0));
        Av->data[one2_tmp] = QQ->data[bi_idx_0 + QQ->size[0] * Z0_n];
      }
    }

    //  Off-diagonal terms of A_iijj
    loop_ub = Qt->size[0];
    bi_idx_0 = Z0_d->size[0];
    Z0_d->size[0] = Qt->size[0];
    emxEnsureCapacity_real_T(Z0_d, bi_idx_0);
    for (bi_idx_0 = 0; bi_idx_0 < loop_ub; bi_idx_0++) {
      Z0_d->data[bi_idx_0] = Qt->data[bi_idx_0 + Qt->size[0] * (t2_n - 1)];
    }

    t2_n = static_cast<int>((d + 2.0));
    bi_idx_0 = b->size[0] * b->size[1];
    b->size[0] = 1;
    b->size[1] = b_loop_ub + 1;
    emxEnsureCapacity_real_T(b, bi_idx_0);
    for (bi_idx_0 = 0; bi_idx_0 <= b_loop_ub; bi_idx_0++) {
      b->data[bi_idx_0] = Q->data[(t2_n + Q->size[0] * bi_idx_0) - 1];
    }

    t2_n = QQ->size[0] * QQ->size[1];
    QQ->size[0] = Z0_d->size[0];
    loop_ub = b->size[1];
    QQ->size[1] = b->size[1];
    emxEnsureCapacity_real_T(QQ, t2_n);
    for (t2_n = 0; t2_n < loop_ub; t2_n++) {
      one2_tmp = Z0_d->size[0];
      for (bi_idx_0 = 0; bi_idx_0 < one2_tmp; bi_idx_0++) {
        QQ->data[bi_idx_0 + QQ->size[0] * t2_n] = Z0_d->data[bi_idx_0] * b->
          data[t2_n];
      }
    }

    itrr++;
    for (bi_idx_0 = 0; bi_idx_0 < t2_m; bi_idx_0++) {
      for (Z0_n = 0; Z0_n < t2_m; Z0_n++) {
        itra++;
        one2_tmp = static_cast<int>(itra) - 1;
        one2->data[one2_tmp] = itrr;
        sizX = 2.0 * static_cast<double>(m);
        Aj->data[one2_tmp] = ((sizX + (static_cast<double>(bi_idx_0) + 1.0)) -
                              1.0) * (4.0 * (static_cast<double>(n) - 2.0)) +
          (sizX + (static_cast<double>(Z0_n) + 1.0));
        Av->data[one2_tmp] = QQ->data[bi_idx_0 + QQ->size[0] * Z0_n];
      }
    }
  }

  //  Trace of gain materix must be the specified value in 'trVal'
  itrr++;
  i2 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i < i2; b_i++) {
    itra++;
    idxR = 2.0 * (static_cast<double>(n) - 2.0) + (static_cast<double>(b_i) +
      1.0);
    one2_tmp = static_cast<int>(itra) - 1;
    one2->data[one2_tmp] = itrr;
    Aj->data[one2_tmp] = (idxR - 1.0) * (4.0 * (static_cast<double>(n) - 2.0)) +
      idxR;
    Av->data[one2_tmp] = 1.0;
  }

  itrb++;
  bi_idx_0 = static_cast<int>(itrb) - 1;
  bi->data[bi_idx_0] = itrr;
  bv->data[bi_idx_0] = trVal;

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Symmetry
  i2 = (adj->size[0] - 2) << 2;
  for (b_i = 0; b_i <= i2 - 2; b_i++) {
    t2_n = (m << 2) - b_i;
    for (Z0_n = 0; Z0_n <= t2_n - 2; Z0_n++) {
      idxR = ((static_cast<double>(b_i) + 1.0) + 1.0) + static_cast<double>(Z0_n);

      //  Symmetric entries should be equal
      itrr++;
      itra++;
      one2_tmp = static_cast<int>(itra) - 1;
      one2->data[one2_tmp] = itrr;
      d = 4.0 * (static_cast<double>(n) - 2.0);
      Aj->data[one2_tmp] = ((static_cast<double>(b_i) + 1.0) - 1.0) * d + idxR;
      Av->data[one2_tmp] = 1.0;
      itra++;
      bi_idx_0 = static_cast<int>(itra) - 1;
      one2->data[bi_idx_0] = itrr;
      Aj->data[bi_idx_0] = (idxR - 1.0) * d + (static_cast<double>(b_i) + 1.0);
      Av->data[bi_idx_0] = -1.0;
    }
  }

  emxInit_real_T(&A_d, 1);

  //  Last element set to fix the size of b
  itrb++;
  bi_idx_0 = static_cast<int>(itrb) - 1;
  bi->data[bi_idx_0] = itrr;
  bv->data[bi_idx_0] = 0.0;

  //  % Remove any additional entries
  //  Ai(itra+1:end) = [];
  //  Aj(itra+1:end) = [];
  //  Av(itra+1:end) = [];
  //
  //  bi(itrb+1:end) = [];
  //  bv(itrb+1:end) = [];
  //  Make sparse matrices
  b_sparse(one2, Aj, Av, &expl_temp);
  i2 = A_d->size[0];
  A_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(A_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    A_d->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&A_colidx, 1);
  i2 = A_colidx->size[0];
  A_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(A_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    A_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&A_rowidx, 1);
  i2 = A_rowidx->size[0];
  A_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(A_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    A_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  A_m = expl_temp.m;
  bi_idx_0 = expl_temp.n;
  i2 = qsbar->size[0];
  qsbar->size[0] = bi->size[0];
  emxEnsureCapacity_real_T(qsbar, i2);
  loop_ub = bi->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    qsbar->data[i2] = 1.0;
  }

  b_sparse(bi, qsbar, bv, &expl_temp);
  i2 = bv->size[0];
  bv->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(bv, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    bv->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&b_colidx, 1);
  i2 = b_colidx->size[0];
  b_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(b_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    b_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&b_rowidx, 1);
  i2 = b_rowidx->size[0];
  b_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(b_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    b_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_real_T(&As_d, 1);
  b_m = expl_temp.m;
  b_n = expl_temp.n;

  //  Size of optimization variable
  sizX = 4.0 * (static_cast<double>(adj->size[0]) - 2.0);

  //  ADMM algorithm--full eigendecomposition
  sparse_transpose(A_d, A_colidx, A_rowidx, A_m, bi_idx_0, &expl_temp);
  i2 = As_d->size[0];
  As_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(As_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    As_d->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&As_colidx, 1);
  i2 = As_colidx->size[0];
  As_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(As_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    As_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&As_rowidx, 1);
  i2 = As_rowidx->size[0];
  As_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(As_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    As_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_real_T(&AAs_d, 1);
  m = expl_temp.m;

  //  Dual operator
  sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, As_d, As_colidx, As_rowidx,
                expl_temp.n, &expl_temp);
  i2 = AAs_d->size[0];
  AAs_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(AAs_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    AAs_d->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&AAs_colidx, 1);
  i2 = AAs_colidx->size[0];
  AAs_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(AAs_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    AAs_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&AAs_rowidx, 1);
  i2 = AAs_rowidx->size[0];
  AAs_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(AAs_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    AAs_rowidx->data[i2] = expl_temp.rowidx->data[i2];
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
  i2 = one2->size[0];
  one2->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = expl_temp.d->data[i2];
  }

  i2 = t2_colidx->size[0];
  t2_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t2_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = t2_rowidx->size[0];
  t2_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t2_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  t2_m = expl_temp.m;
  t2_n = expl_temp.n;
  sparse_horzcat(I0_d, I0_colidx, I0_rowidx, I0_m, I0_n, I0_d, I0_colidx,
                 I0_rowidx, I0_m, I0_n, &expl_temp);
  i2 = Aj->size[0];
  Aj->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Aj, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    Aj->data[i2] = expl_temp.d->data[i2];
  }

  i2 = t3_colidx->size[0];
  t3_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t3_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t3_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = t3_rowidx->size[0];
  t3_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t3_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t3_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  emxInit_real_T(&X_d, 1);
  sparse_vertcat(one2, t2_colidx, t2_rowidx, t2_m, t2_n, Aj, t3_colidx,
                 t3_rowidx, expl_temp.m, expl_temp.n, &expl_temp);
  i2 = X_d->size[0];
  X_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(X_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    X_d->data[i2] = expl_temp.d->data[i2];
  }

  emxInit_int32_T(&X_colidx, 1);
  i2 = X_colidx->size[0];
  X_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(X_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    X_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  emxInit_int32_T(&X_rowidx, 1);
  i2 = X_rowidx->size[0];
  X_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(X_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    X_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  X_m = expl_temp.m;
  X_n = expl_temp.n;
  b_i = 0;
  emxInit_creal_T(&V, 2);
  emxInit_creal_T(&D, 2);
  emxInit_creal_T(&b_d, 1);
  emxInit_int32_T(&r1, 1);
  emxInit_int32_T(&r2, 1);
  emxInit_creal_T(&y, 2);
  emxInit_creal_T(&b_b, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_int32_T(&t4_colidx, 1);
  emxInit_int32_T(&t4_rowidx, 1);
  d_emxInitStruct_coder_internal_(&b_expl_temp);
  emxInit_creal_T(&c_d, 1);
  exitg1 = false;
  while ((!exitg1) && (b_i < 10)) {
    // %%%%%% Update for y
    sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
    i2 = I0_d->size[0];
    I0_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(I0_d, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      I0_d->data[i2] = expl_temp.d->data[i2];
    }

    i2 = I0_colidx->size[0];
    I0_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(I0_colidx, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      I0_colidx->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = I0_rowidx->size[0];
    I0_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(I0_rowidx, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      I0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
    }

    I0_m = expl_temp.m;
    I0_n = expl_temp.n;
    sparse_times(bv, b_colidx, b_rowidx, b_m, b_n, &expl_temp);
    i2 = Z0_d->size[0];
    Z0_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Z0_d, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Z0_d->data[i2] = expl_temp.d->data[i2];
    }

    i2 = Z0_colidx->size[0];
    Z0_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(Z0_colidx, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Z0_colidx->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = Z0_rowidx->size[0];
    Z0_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(Z0_rowidx, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Z0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
    }

    sparse_minus(C_d, C_colidx, C_rowidx, Z_d, Z_colidx, Z_rowidx, Z_m, Z_n,
                 one2, t2_colidx, t2_rowidx, &t2_m, &t2_n);
    sparse_minus(one2, t2_colidx, t2_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m,
                 I0_n, Aj, t3_colidx, t3_rowidx, &bi_idx_0, &Z0_n);
    vec(Aj, t3_colidx, t3_rowidx, bi_idx_0, Z0_n, bi, t4_colidx, t4_rowidx,
        &one2_tmp);
    b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, bi, t4_colidx, t4_rowidx,
                    &b_expl_temp);
    i2 = one2->size[0];
    one2->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(one2, i2);
    loop_ub = b_expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      one2->data[i2] = b_expl_temp.d->data[i2];
    }

    i2 = t2_colidx->size[0];
    t2_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t2_colidx, i2);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_colidx->data[i2] = b_expl_temp.colidx->data[i2];
    }

    i2 = t2_rowidx->size[0];
    t2_rowidx->size[0] = b_expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t2_rowidx, i2);
    loop_ub = b_expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_rowidx->data[i2] = b_expl_temp.rowidx->data[i2];
    }

    sparse_plus(one2, t2_colidx, t2_rowidx, Z0_d, Z0_colidx, Z0_rowidx,
                expl_temp.m, bi, t4_colidx, t4_rowidx, &one2_tmp);
    sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, bi, t4_colidx,
                    t4_rowidx, one2_tmp, &b_expl_temp);
    i2 = one2->size[0];
    one2->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(one2, i2);
    loop_ub = b_expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      one2->data[i2] = b_expl_temp.d->data[i2];
    }

    i2 = t2_colidx->size[0];
    t2_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t2_colidx, i2);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_colidx->data[i2] = b_expl_temp.colidx->data[i2];
    }

    i2 = t2_rowidx->size[0];
    t2_rowidx->size[0] = b_expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t2_rowidx, i2);
    loop_ub = b_expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_rowidx->data[i2] = b_expl_temp.rowidx->data[i2];
    }

    // %%%%%% Update for S
    sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
    i2 = I0_d->size[0];
    I0_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(I0_d, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      I0_d->data[i2] = expl_temp.d->data[i2];
    }

    i2 = I0_colidx->size[0];
    I0_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(I0_colidx, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      I0_colidx->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = I0_rowidx->size[0];
    I0_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(I0_rowidx, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      I0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
    }

    I0_m = expl_temp.m;
    I0_n = expl_temp.n;
    b_sparse_mtimes(As_d, As_colidx, As_rowidx, m, one2, t2_colidx, t2_rowidx,
                    &b_expl_temp);
    i2 = bi->size[0];
    bi->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(bi, i2);
    loop_ub = b_expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      bi->data[i2] = b_expl_temp.d->data[i2];
    }

    i2 = t4_colidx->size[0];
    t4_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t4_colidx, i2);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t4_colidx->data[i2] = b_expl_temp.colidx->data[i2];
    }

    i2 = t4_rowidx->size[0];
    t4_rowidx->size[0] = b_expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t4_rowidx, i2);
    loop_ub = b_expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t4_rowidx->data[i2] = b_expl_temp.rowidx->data[i2];
    }

    b_sizX[0] = sizX;
    b_sizX[1] = sizX;
    sparse_reshape(bi, t4_colidx, t4_rowidx, b_sizX, &expl_temp);
    i2 = one2->size[0];
    one2->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(one2, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      one2->data[i2] = expl_temp.d->data[i2];
    }

    i2 = t2_colidx->size[0];
    t2_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t2_colidx, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_colidx->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = t2_rowidx->size[0];
    t2_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t2_rowidx, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_rowidx->data[i2] = expl_temp.rowidx->data[i2];
    }

    sparse_minus(C_d, C_colidx, C_rowidx, one2, t2_colidx, t2_rowidx,
                 expl_temp.m, expl_temp.n, Aj, t3_colidx, t3_rowidx, &bi_idx_0,
                 &Z0_n);
    sparse_minus(Aj, t3_colidx, t3_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m,
                 I0_n, qsbar, one1, ii, &bi_idx_0, &Z0_n);
    sparse_transpose(qsbar, one1, ii, bi_idx_0, Z0_n, &expl_temp);
    i2 = one2->size[0];
    one2->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(one2, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      one2->data[i2] = expl_temp.d->data[i2];
    }

    i2 = t2_colidx->size[0];
    t2_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t2_colidx, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_colidx->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = t2_rowidx->size[0];
    t2_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t2_rowidx, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t2_rowidx->data[i2] = expl_temp.rowidx->data[i2];
    }

    b_sparse_plus(qsbar, one1, ii, one2, t2_colidx, t2_rowidx, expl_temp.m,
                  expl_temp.n, Aj, t3_colidx, t3_rowidx, &bi_idx_0, &Z0_n);
    sparse_rdivide(Aj, t3_colidx, t3_rowidx, bi_idx_0, Z0_n, &expl_temp);
    i2 = qsbar->size[0];
    qsbar->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(qsbar, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      qsbar->data[i2] = expl_temp.d->data[i2];
    }

    i2 = one1->size[0];
    one1->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(one1, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      one1->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = ii->size[0];
    ii->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(ii, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      ii->data[i2] = expl_temp.rowidx->data[i2];
    }

    sparse_full(qsbar, one1, ii, expl_temp.m, expl_temp.n, Q);
    eig(Q, V, D);
    c_diag(D, b_d);
    one2_tmp = b_d->size[0] - 1;
    t2_m = 0;
    for (bi_idx_0 = 0; bi_idx_0 <= one2_tmp; bi_idx_0++) {
      if (b_d->data[bi_idx_0].re > 1.0E-5) {
        t2_m++;
      }
    }

    i2 = r1->size[0];
    r1->size[0] = t2_m;
    emxEnsureCapacity_int32_T(r1, i2);
    Z0_n = 0;
    t2_n = b_d->size[0] - 1;
    t2_m = 0;
    for (bi_idx_0 = 0; bi_idx_0 <= one2_tmp; bi_idx_0++) {
      if (b_d->data[bi_idx_0].re > 1.0E-5) {
        r1->data[Z0_n] = bi_idx_0 + 1;
        Z0_n++;
        t2_m++;
      }
    }

    i2 = r2->size[0];
    r2->size[0] = t2_m;
    emxEnsureCapacity_int32_T(r2, i2);
    Z0_n = 0;
    for (bi_idx_0 = 0; bi_idx_0 <= t2_n; bi_idx_0++) {
      if (b_d->data[bi_idx_0].re > 1.0E-5) {
        r2->data[Z0_n] = bi_idx_0 + 1;
        Z0_n++;
      }
    }

    loop_ub = V->size[0];
    i2 = D->size[0] * D->size[1];
    D->size[0] = V->size[0];
    D->size[1] = r2->size[0];
    emxEnsureCapacity_creal_T(D, i2);
    one2_tmp = r2->size[0];
    for (i2 = 0; i2 < one2_tmp; i2++) {
      for (t2_n = 0; t2_n < loop_ub; t2_n++) {
        D->data[t2_n + D->size[0] * i2] = V->data[t2_n + V->size[0] * (r2->
          data[i2] - 1)];
      }
    }

    i2 = c_d->size[0];
    c_d->size[0] = r2->size[0];
    emxEnsureCapacity_creal_T(c_d, i2);
    loop_ub = r2->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      c_d->data[i2] = b_d->data[r2->data[i2] - 1];
    }

    d_diag(c_d, b_b);
    if ((r2->size[0] == 1) || (b_b->size[0] == 1)) {
      i2 = y->size[0] * y->size[1];
      y->size[0] = D->size[0];
      y->size[1] = b_b->size[1];
      emxEnsureCapacity_creal_T(y, i2);
      loop_ub = D->size[0];
      for (i2 = 0; i2 < loop_ub; i2++) {
        one2_tmp = b_b->size[1];
        for (t2_n = 0; t2_n < one2_tmp; t2_n++) {
          y->data[i2 + y->size[0] * t2_n].re = 0.0;
          y->data[i2 + y->size[0] * t2_n].im = 0.0;
          bi_idx_0 = D->size[1];
          for (t2_m = 0; t2_m < bi_idx_0; t2_m++) {
            y->data[i2 + y->size[0] * t2_n].re += D->data[i2 + D->size[0] * t2_m]
              .re * b_b->data[t2_m + b_b->size[0] * t2_n].re - D->data[i2 +
              D->size[0] * t2_m].im * b_b->data[t2_m + b_b->size[0] * t2_n].im;
            y->data[i2 + y->size[0] * t2_n].im += D->data[i2 + D->size[0] * t2_m]
              .re * b_b->data[t2_m + b_b->size[0] * t2_n].im + D->data[i2 +
              D->size[0] * t2_m].im * b_b->data[t2_m + b_b->size[0] * t2_n].re;
          }
        }
      }
    } else {
      mtimes(D, b_b, y);
    }

    loop_ub = V->size[0];
    i2 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = r1->size[0];
    b_b->size[1] = V->size[0];
    emxEnsureCapacity_creal_T(b_b, i2);
    for (i2 = 0; i2 < loop_ub; i2++) {
      one2_tmp = r1->size[0];
      for (t2_n = 0; t2_n < one2_tmp; t2_n++) {
        b_b->data[t2_n + b_b->size[0] * i2] = V->data[i2 + V->size[0] *
          (r1->data[t2_n] - 1)];
      }
    }

    if ((y->size[1] == 1) || (b_b->size[0] == 1)) {
      i2 = D->size[0] * D->size[1];
      D->size[0] = y->size[0];
      D->size[1] = b_b->size[1];
      emxEnsureCapacity_creal_T(D, i2);
      loop_ub = y->size[0];
      for (i2 = 0; i2 < loop_ub; i2++) {
        one2_tmp = b_b->size[1];
        for (t2_n = 0; t2_n < one2_tmp; t2_n++) {
          D->data[i2 + D->size[0] * t2_n].re = 0.0;
          D->data[i2 + D->size[0] * t2_n].im = 0.0;
          bi_idx_0 = y->size[1];
          for (t2_m = 0; t2_m < bi_idx_0; t2_m++) {
            D->data[i2 + D->size[0] * t2_n].re += y->data[i2 + y->size[0] * t2_m]
              .re * b_b->data[t2_m + b_b->size[0] * t2_n].re - y->data[i2 +
              y->size[0] * t2_m].im * b_b->data[t2_m + b_b->size[0] * t2_n].im;
            D->data[i2 + D->size[0] * t2_n].im += y->data[i2 + y->size[0] * t2_m]
              .re * b_b->data[t2_m + b_b->size[0] * t2_n].im + y->data[i2 +
              y->size[0] * t2_m].im * b_b->data[t2_m + b_b->size[0] * t2_n].re;
          }
        }
      }
    } else {
      mtimes(y, b_b, D);
    }

    i2 = Q->size[0] * Q->size[1];
    Q->size[0] = D->size[0];
    Q->size[1] = D->size[1];
    emxEnsureCapacity_real_T(Q, i2);
    loop_ub = D->size[0] * D->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Q->data[i2] = D->data[i2].re;
    }

    c_sparse(Q, Z_d, Z_colidx, Z_rowidx, &Z_m, &Z_n);

    // %%%%%% Update for X
    i2 = Z0_d->size[0];
    Z0_d->size[0] = X_d->size[0];
    emxEnsureCapacity_real_T(Z0_d, i2);
    loop_ub = X_d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Z0_d->data[i2] = X_d->data[i2];
    }

    i2 = Z0_colidx->size[0];
    Z0_colidx->size[0] = X_colidx->size[0];
    emxEnsureCapacity_int32_T(Z0_colidx, i2);
    loop_ub = X_colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Z0_colidx->data[i2] = X_colidx->data[i2];
    }

    i2 = Z0_rowidx->size[0];
    Z0_rowidx->size[0] = X_rowidx->size[0];
    emxEnsureCapacity_int32_T(Z0_rowidx, i2);
    loop_ub = X_rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Z0_rowidx->data[i2] = X_rowidx->data[i2];
    }

    bi_idx_0 = X_m;
    Z0_n = X_n;
    sparse_minus(Z_d, Z_colidx, Z_rowidx, qsbar, one1, ii, expl_temp.m,
                 expl_temp.n, one2, t2_colidx, t2_rowidx, &t2_m, &t2_n);
    sparse_times(one2, t2_colidx, t2_rowidx, t2_m, t2_n, &expl_temp);
    i2 = X_d->size[0];
    X_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(X_d, i2);
    loop_ub = expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      X_d->data[i2] = expl_temp.d->data[i2];
    }

    i2 = X_colidx->size[0];
    X_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(X_colidx, i2);
    loop_ub = expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      X_colidx->data[i2] = expl_temp.colidx->data[i2];
    }

    i2 = X_rowidx->size[0];
    X_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(X_rowidx, i2);
    loop_ub = expl_temp.rowidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      X_rowidx->data[i2] = expl_temp.rowidx->data[i2];
    }

    X_m = expl_temp.m;
    X_n = expl_temp.n;

    // %%%%%% Stop criteria
    b_sparse_parenReference(Z0_d, Z0_colidx, Z0_rowidx, bi_idx_0, Z0_n, bi,
      t4_colidx, t4_rowidx, &one2_tmp);
    b_sparse_parenReference(X_d, X_colidx, X_rowidx, expl_temp.m, expl_temp.n,
      one2, t2_colidx, t2_rowidx, &bi_idx_0);
    b_sparse_minus(bi, t4_colidx, t4_rowidx, one2, t2_colidx, t2_rowidx,
                   bi_idx_0, qsbar, one1, ii, &Z0_n);
    sparse_abs(qsbar, one1, ii, Z0_n, &b_expl_temp);
    i2 = bi->size[0];
    bi->size[0] = b_expl_temp.d->size[0];
    emxEnsureCapacity_real_T(bi, i2);
    loop_ub = b_expl_temp.d->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      bi->data[i2] = b_expl_temp.d->data[i2];
    }

    i2 = t4_colidx->size[0];
    t4_colidx->size[0] = b_expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t4_colidx, i2);
    loop_ub = b_expl_temp.colidx->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      t4_colidx->data[i2] = b_expl_temp.colidx->data[i2];
    }

    sum(bi, t4_colidx, b_expl_temp.m, qsbar, ii, t2_colidx);
    sparse_lt(qsbar, ii, t6_d, one1, t2_colidx);
    if (b_sparse_full(t6_d, one1)) {
      //  change in X is small
      exitg1 = true;
    } else {
      //  trace of sparse matrix for MATLAB Coder
      idxR = 2.0 * (static_cast<double>(n) - 2.0) + 1.0;
      if (expl_temp.m < idxR) {
        b->size[0] = 1;
        b->size[1] = 0;
      } else if (idxR == idxR) {
        i2 = b->size[0] * b->size[1];
        b->size[0] = 1;
        loop_ub = static_cast<int>((static_cast<double>(expl_temp.m) - idxR));
        b->size[1] = loop_ub + 1;
        emxEnsureCapacity_real_T(b, i2);
        for (i2 = 0; i2 <= loop_ub; i2++) {
          b->data[i2] = idxR + static_cast<double>(i2);
        }
      } else {
        eml_float_colon(idxR, static_cast<double>(expl_temp.m), b);
      }

      idxR = 2.0 * (static_cast<double>(n) - 2.0) + 1.0;
      if (expl_temp.n < idxR) {
        b_y->size[0] = 1;
        b_y->size[1] = 0;
      } else if (idxR == idxR) {
        i2 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        loop_ub = static_cast<int>((static_cast<double>(expl_temp.n) - idxR));
        b_y->size[1] = loop_ub + 1;
        emxEnsureCapacity_real_T(b_y, i2);
        for (i2 = 0; i2 <= loop_ub; i2++) {
          b_y->data[i2] = idxR + static_cast<double>(i2);
        }
      } else {
        eml_float_colon(idxR, static_cast<double>(expl_temp.n), b_y);
      }

      c_sparse_parenReference(X_d, X_colidx, X_rowidx, b, b_y, one2, t2_colidx,
        t2_rowidx, &t2_m, &t2_n);
      sparse_diag(one2, t2_colidx, t2_rowidx, t2_m, t2_n, bi, t4_colidx,
                  t4_rowidx, &one2_tmp);
      sum(bi, t4_colidx, one2_tmp, qsbar, ii, t2_colidx);
      if (std::abs(c_sparse_full(qsbar, ii) - trVal) / trVal * 100.0 < 10.0) {
        //  trace of X is close to the desired value
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }

  emxFree_creal_T(&c_d);
  emxFree_boolean_T(&t6_d);
  emxFree_real_T(&b_y);
  emxFree_creal_T(&b_b);
  emxFree_creal_T(&y);
  emxFree_real_T(&b);
  emxFree_int32_T(&r2);
  emxFree_int32_T(&r1);
  emxFree_creal_T(&b_d);
  emxFree_creal_T(&D);
  emxFree_creal_T(&V);

  //  Set S=0 to project the final solution and ensure that it satisfies the linear constraints given by the adjacency matrix 
  b_sparse_times(Z_d, Z_colidx, Z_rowidx, Z_m, Z_n, &expl_temp);
  i2 = Av->size[0];
  Av->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Av, i2);
  loop_ub = expl_temp.d->size[0];
  emxFree_real_T(&Z_d);
  for (i2 = 0; i2 < loop_ub; i2++) {
    Av->data[i2] = expl_temp.d->data[i2];
  }

  i2 = Z_colidx->size[0];
  Z_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(Z_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    Z_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = Z_rowidx->size[0];
  Z_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(Z_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    Z_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  bi_idx_0 = expl_temp.m;
  Z0_n = expl_temp.n;
  sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
  i2 = I0_d->size[0];
  I0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(I0_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_d->data[i2] = expl_temp.d->data[i2];
  }

  i2 = I0_colidx->size[0];
  I0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(I0_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = I0_rowidx->size[0];
  I0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(I0_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  I0_m = expl_temp.m;
  I0_n = expl_temp.n;
  sparse_times(bv, b_colidx, b_rowidx, b_m, b_n, &expl_temp);
  i2 = Z0_d->size[0];
  Z0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Z0_d, i2);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&b_rowidx);
  emxFree_int32_T(&b_colidx);
  emxFree_real_T(&bv);
  for (i2 = 0; i2 < loop_ub; i2++) {
    Z0_d->data[i2] = expl_temp.d->data[i2];
  }

  i2 = Z0_colidx->size[0];
  Z0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(Z0_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    Z0_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = Z0_rowidx->size[0];
  Z0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(Z0_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    Z0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  sparse_minus(C_d, C_colidx, C_rowidx, Av, Z_colidx, Z_rowidx, bi_idx_0, Z0_n,
               one2, t2_colidx, t2_rowidx, &t2_m, &t2_n);
  sparse_minus(one2, t2_colidx, t2_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m,
               I0_n, Aj, t3_colidx, t3_rowidx, &bi_idx_0, &Z0_n);
  vec(Aj, t3_colidx, t3_rowidx, bi_idx_0, Z0_n, bi, t4_colidx, t4_rowidx,
      &one2_tmp);
  b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, bi, t4_colidx, t4_rowidx,
                  &b_expl_temp);
  i2 = one2->size[0];
  one2->size[0] = b_expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i2);
  loop_ub = b_expl_temp.d->size[0];
  emxFree_int32_T(&A_rowidx);
  emxFree_int32_T(&A_colidx);
  emxFree_real_T(&A_d);
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = b_expl_temp.d->data[i2];
  }

  i2 = t2_colidx->size[0];
  t2_colidx->size[0] = b_expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t2_colidx, i2);
  loop_ub = b_expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_colidx->data[i2] = b_expl_temp.colidx->data[i2];
  }

  i2 = t2_rowidx->size[0];
  t2_rowidx->size[0] = b_expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t2_rowidx, i2);
  loop_ub = b_expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_rowidx->data[i2] = b_expl_temp.rowidx->data[i2];
  }

  sparse_plus(one2, t2_colidx, t2_rowidx, Z0_d, Z0_colidx, Z0_rowidx,
              expl_temp.m, bi, t4_colidx, t4_rowidx, &one2_tmp);
  sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, bi, t4_colidx,
                  t4_rowidx, one2_tmp, &b_expl_temp);
  i2 = one2->size[0];
  one2->size[0] = b_expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i2);
  loop_ub = b_expl_temp.d->size[0];
  emxFree_int32_T(&AAs_rowidx);
  emxFree_int32_T(&AAs_colidx);
  emxFree_real_T(&AAs_d);
  emxFree_int32_T(&Z0_rowidx);
  emxFree_int32_T(&Z0_colidx);
  emxFree_real_T(&Z0_d);
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = b_expl_temp.d->data[i2];
  }

  i2 = t2_colidx->size[0];
  t2_colidx->size[0] = b_expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t2_colidx, i2);
  loop_ub = b_expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_colidx->data[i2] = b_expl_temp.colidx->data[i2];
  }

  i2 = t2_rowidx->size[0];
  t2_rowidx->size[0] = b_expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t2_rowidx, i2);
  loop_ub = b_expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_rowidx->data[i2] = b_expl_temp.rowidx->data[i2];
  }

  sparse_times(X_d, X_colidx, X_rowidx, X_m, X_n, &expl_temp);
  i2 = I0_d->size[0];
  I0_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(I0_d, i2);
  loop_ub = expl_temp.d->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_d->data[i2] = expl_temp.d->data[i2];
  }

  i2 = I0_colidx->size[0];
  I0_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(I0_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = I0_rowidx->size[0];
  I0_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(I0_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    I0_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  I0_m = expl_temp.m;
  I0_n = expl_temp.n;
  b_sparse_mtimes(As_d, As_colidx, As_rowidx, m, one2, t2_colidx, t2_rowidx,
                  &b_expl_temp);
  i2 = bi->size[0];
  bi->size[0] = b_expl_temp.d->size[0];
  emxEnsureCapacity_real_T(bi, i2);
  loop_ub = b_expl_temp.d->size[0];
  emxFree_int32_T(&As_rowidx);
  emxFree_int32_T(&As_colidx);
  emxFree_real_T(&As_d);
  for (i2 = 0; i2 < loop_ub; i2++) {
    bi->data[i2] = b_expl_temp.d->data[i2];
  }

  i2 = t4_colidx->size[0];
  t4_colidx->size[0] = b_expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t4_colidx, i2);
  loop_ub = b_expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t4_colidx->data[i2] = b_expl_temp.colidx->data[i2];
  }

  i2 = t4_rowidx->size[0];
  t4_rowidx->size[0] = b_expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t4_rowidx, i2);
  loop_ub = b_expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t4_rowidx->data[i2] = b_expl_temp.rowidx->data[i2];
  }

  d_emxFreeStruct_coder_internal_(&b_expl_temp);
  b_sizX[0] = sizX;
  b_sizX[1] = sizX;
  sparse_reshape(bi, t4_colidx, t4_rowidx, b_sizX, &expl_temp);
  i2 = one2->size[0];
  one2->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i2);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&t4_rowidx);
  emxFree_int32_T(&t4_colidx);
  emxFree_real_T(&bi);
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = expl_temp.d->data[i2];
  }

  i2 = t2_colidx->size[0];
  t2_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t2_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = t2_rowidx->size[0];
  t2_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t2_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  sparse_minus(C_d, C_colidx, C_rowidx, one2, t2_colidx, t2_rowidx, expl_temp.m,
               expl_temp.n, Aj, t3_colidx, t3_rowidx, &bi_idx_0, &Z0_n);
  sparse_minus(Aj, t3_colidx, t3_rowidx, I0_d, I0_colidx, I0_rowidx, I0_m, I0_n,
               qsbar, one1, ii, &bi_idx_0, &Z0_n);
  sparse_transpose(qsbar, one1, ii, bi_idx_0, Z0_n, &expl_temp);
  i2 = one2->size[0];
  one2->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i2);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&C_rowidx);
  emxFree_int32_T(&C_colidx);
  emxFree_real_T(&C_d);
  emxFree_int32_T(&I0_rowidx);
  emxFree_int32_T(&I0_colidx);
  emxFree_real_T(&I0_d);
  for (i2 = 0; i2 < loop_ub; i2++) {
    one2->data[i2] = expl_temp.d->data[i2];
  }

  i2 = t2_colidx->size[0];
  t2_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t2_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = t2_rowidx->size[0];
  t2_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t2_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    t2_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  b_sparse_plus(qsbar, one1, ii, one2, t2_colidx, t2_rowidx, expl_temp.m,
                expl_temp.n, Aj, t3_colidx, t3_rowidx, &bi_idx_0, &Z0_n);
  sparse_rdivide(Aj, t3_colidx, t3_rowidx, bi_idx_0, Z0_n, &expl_temp);
  i2 = qsbar->size[0];
  qsbar->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(qsbar, i2);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&t3_rowidx);
  emxFree_int32_T(&t3_colidx);
  emxFree_real_T(&Aj);
  for (i2 = 0; i2 < loop_ub; i2++) {
    qsbar->data[i2] = expl_temp.d->data[i2];
  }

  i2 = one1->size[0];
  one1->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(one1, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    one1->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = ii->size[0];
  ii->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(ii, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    ii->data[i2] = expl_temp.rowidx->data[i2];
  }

  sparse_minus(Av, Z_colidx, Z_rowidx, qsbar, one1, ii, expl_temp.m, expl_temp.n,
               one2, t2_colidx, t2_rowidx, &t2_m, &t2_n);
  sparse_times(one2, t2_colidx, t2_rowidx, t2_m, t2_n, &expl_temp);
  i2 = X_d->size[0];
  X_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(X_d, i2);
  loop_ub = expl_temp.d->size[0];
  emxFree_int32_T(&t2_rowidx);
  emxFree_int32_T(&t2_colidx);
  emxFree_int32_T(&ii);
  emxFree_real_T(&Av);
  emxFree_int32_T(&Z_rowidx);
  emxFree_int32_T(&Z_colidx);
  emxFree_real_T(&one2);
  emxFree_int32_T(&one1);
  emxFree_real_T(&qsbar);
  for (i2 = 0; i2 < loop_ub; i2++) {
    X_d->data[i2] = expl_temp.d->data[i2];
  }

  i2 = X_colidx->size[0];
  X_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(X_colidx, i2);
  loop_ub = expl_temp.colidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    X_colidx->data[i2] = expl_temp.colidx->data[i2];
  }

  i2 = X_rowidx->size[0];
  X_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(X_rowidx, i2);
  loop_ub = expl_temp.rowidx->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    X_rowidx->data[i2] = expl_temp.rowidx->data[i2];
  }

  sparse_full(X_d, X_colidx, X_rowidx, expl_temp.m, expl_temp.n, QQ);

  //  Transform X from sparse to full representation
  d = 2.0 * (static_cast<double>(adj->size[0]) - 2.0) + 1.0;
  c_emxFreeStruct_coder_internal_(&expl_temp);
  emxFree_int32_T(&X_rowidx);
  emxFree_int32_T(&X_colidx);
  emxFree_real_T(&X_d);
  if (d > QQ->size[0]) {
    i2 = 0;
    t2_n = 0;
  } else {
    i2 = static_cast<int>(d) - 1;
    t2_n = QQ->size[0];
  }

  d = 2.0 * (static_cast<double>(adj->size[0]) - 2.0) + 1.0;
  if (d > QQ->size[1]) {
    t2_m = 0;
    bi_idx_0 = 0;
  } else {
    t2_m = static_cast<int>(d) - 1;
    bi_idx_0 = QQ->size[1];
  }

  //  The componenet of X corresponding to the gain matrix
  loop_ub = t2_n - i2;
  t2_n = Q->size[0] * Q->size[1];
  Q->size[0] = loop_ub;
  one2_tmp = bi_idx_0 - t2_m;
  Q->size[1] = one2_tmp;
  emxEnsureCapacity_real_T(Q, t2_n);
  for (t2_n = 0; t2_n < one2_tmp; t2_n++) {
    for (bi_idx_0 = 0; bi_idx_0 < loop_ub; bi_idx_0++) {
      Q->data[bi_idx_0 + Q->size[0] * t2_n] = -QQ->data[(i2 + bi_idx_0) +
        QQ->size[0] * (t2_m + t2_n)];
    }
  }

  i2 = QQ->size[0] * QQ->size[1];
  QQ->size[0] = Q->size[0];
  QQ->size[1] = Q->size[1];
  emxEnsureCapacity_real_T(QQ, i2);
  loop_ub = Q->size[0] * Q->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    QQ->data[i2] = Q->data[i2];
  }

  if ((i1 == 1) || (QQ->size[0] == 1)) {
    loop_ub = U->size[0];
    i1 = Q->size[0] * Q->size[1];
    Q->size[0] = U->size[0];
    Q->size[1] = QQ->size[1];
    emxEnsureCapacity_real_T(Q, i1);
    for (i1 = 0; i1 < loop_ub; i1++) {
      one2_tmp = QQ->size[1];
      for (i2 = 0; i2 < one2_tmp; i2++) {
        Q->data[i1 + Q->size[0] * i2] = 0.0;
        for (t2_n = 0; t2_n <= b_loop_ub; t2_n++) {
          Q->data[i1 + Q->size[0] * i2] += U->data[i1 + U->size[0] * (i + t2_n)]
            * QQ->data[t2_n + QQ->size[0] * i2];
        }
      }
    }
  } else {
    loop_ub = U->size[0] - 1;
    i2 = Q->size[0] * Q->size[1];
    Q->size[0] = U->size[0];
    Q->size[1] = i1;
    emxEnsureCapacity_real_T(Q, i2);
    for (i1 = 0; i1 <= b_loop_ub; i1++) {
      for (i2 = 0; i2 <= loop_ub; i2++) {
        Q->data[i2 + Q->size[0] * i1] = U->data[i2 + U->size[0] * (i + i1)];
      }
    }

    i = U->size[0] * U->size[1];
    U->size[0] = Q->size[0];
    U->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(U, i);
    loop_ub = Q->size[1];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = Q->size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        U->data[i1 + U->size[0] * i] = Q->data[i1 + Q->size[0] * i];
      }
    }

    b_mtimes(U, QQ, Q);
  }

  emxFree_real_T(&U);
  emxFree_real_T(&QQ);
  if ((Q->size[1] == 1) || (Qt->size[0] == 1)) {
    i = Aopt->size[0] * Aopt->size[1];
    Aopt->size[0] = Q->size[0];
    Aopt->size[1] = Qt->size[1];
    emxEnsureCapacity_real_T(Aopt, i);
    loop_ub = Q->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = Qt->size[1];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        Aopt->data[i + Aopt->size[0] * i1] = 0.0;
        one2_tmp = Q->size[1];
        for (i2 = 0; i2 < one2_tmp; i2++) {
          Aopt->data[i + Aopt->size[0] * i1] += Q->data[i + Q->size[0] * i2] *
            Qt->data[i2 + Qt->size[0] * i1];
        }
      }
    }
  } else {
    b_mtimes(Q, Qt, Aopt);
  }

  emxFree_real_T(&Qt);
  emxFree_real_T(&Q);

  //  The formation gain matrix
  //  i
  //  eig(X2)
  //  trace(X2)
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
  //      dw2 = dw1; % The second eigenvalue
  //      % The second eigenvector
  //      vw2 = zeros(4*m,1);
  //      for j = 1 : 2*m
  //          vw2(j*2-1:j*2) = [0 -1; 1 0] * vw1(j*2-1:j*2);
  //      end
  //
  //      numPos = 0; % Number of positive eigenvalues
  //      Wold = W;
  //      while dw1 > 10*epsEig %dw(1) > epsEig
  //          % Save positive eigenvalues and eigenvectors
  //          numPos = numPos + 1;
  //          d(numPos*2-1:numPos*2) = [dw1; dw2];
  //          V(:, numPos*2-1:numPos*2) = [vw1, vw2];
  //
  //          % Remove positive eigenvalue components from matrix W
  //          Wnew = Wold - dw1*vw1*vw1' - dw2*vw2*vw2.';
  //
  //          % Find most positive eigenvalues of the new W matrix
  //          [vw1, dw1] = eigs(Wnew,1,'largestreal', 'Tolerance',epsEig, 'FailureTreatment','keep', 'Display', false);  
  //          dw2 = dw1; % The second eigenvalue
  //          % The second eigenvector
  //          vw2 = zeros(4*m,1);
  //          for j = 1 : 2*m
  //              vw2(j*2-1:j*2) = [0 -1; 1 0] * vw1(j*2-1:j*2);
  //          end
  //
  //          Wold = Wnew;
  //      end
  //
  //      S =  V(:,1:numPos*2) * diag(d(1:numPos*2)) * V(:,1:numPos*2).';
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
  //      trPerc = abs(trace( X(2*m+1:end, 2*m+1:end) ) - trVal) / trVal * 100;
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
  //  X2 = X(2*m+1:end,2*m+1:end); % The componenet of X corresponding to the gain matrix 
  //  Aopt = Q * (-X2) * Qt; % The formation gain matrix
  //
  //  % i
  //  % eig(X2)
  //  % trace(X2)
  //  Test solution
  //  eig(X2)
  //  eig(Aopt)
  //  % Enforce zero-gain constraint for non-neighbor agents
  //  Atrim = Aopt;
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
// File trailer for ADMMGainDesign2D.cpp
//
// [EOF]
//
