//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign2D.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign2D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "sparse.h"
#include "reshape.h"
#include "mtimes.h"
#include "fillIn.h"
#include "vec.h"
#include "sum.h"
#include "diag.h"
#include "sparse1.h"
#include "diag1.h"
#include "eig.h"
#include "vertcat.h"
#include "horzcat.h"
#include "triu.h"
#include "speye.h"
#include "svd.h"
#include "ADMMGainDesign3D_rtwutil.h"

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
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }

    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }

      quotient = -(int)tempAbsQuotient;
    } else {
      quotient = (int)tempAbsQuotient;
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
  int i3;
  int idx;
  int i4;
  int i5;
  int i6;
  emxArray_int32_T *r2;
  int nx;
  emxArray_int32_T *one1;
  emxArray_real_T *one2;
  emxArray_real_T *b_Qs;
  int ii;
  emxArray_real_T *U;
  emxArray_real_T *unusedU0;
  double unusedU1[16];
  unsigned int itra;
  emxArray_real_T *Q;
  int loop_ub;
  emxArray_real_T *Qt;
  coder_internal_sparse I0;
  int jj;
  coder_internal_sparse Z0;
  emxArray_real_T *Z_d;
  emxArray_int32_T *Z_colidx;
  emxArray_int32_T *Z_rowidx;
  emxArray_real_T *C_d;
  emxArray_real_T *Aj;
  emxArray_real_T *Av;
  emxArray_int32_T *t3_colidx;
  emxArray_int32_T *t3_rowidx;
  emxArray_int32_T *t4_colidx;
  emxArray_int32_T *t4_rowidx;
  int aoffset;
  int Z_n;
  int boffset;
  emxArray_int32_T *C_colidx;
  emxArray_int32_T *C_rowidx;
  emxArray_boolean_T *S;
  double trVal;
  double numElmB2;
  emxArray_real_T *Su;
  emxArray_boolean_T *r3;
  emxArray_boolean_T *t7_d;
  emxArray_int32_T *i;
  bool exitg1;
  bool guard1 = false;
  double a;
  double numElmAtot;
  emxArray_real_T *bi;
  emxArray_real_T *bv;
  double itrr;
  unsigned int itrb;
  int b_i;
  int j;
  emxArray_real_T *b_a;
  emxArray_real_T *b;
  double cdiff;
  emxArray_real_T *A_d;
  emxArray_int32_T *A_colidx;
  emxArray_int32_T *A_rowidx;
  int A_m;
  emxArray_int32_T *b_colidx;
  emxArray_int32_T *b_rowidx;
  emxArray_int32_T *As_colidx;
  emxArray_int32_T *As_rowidx;
  emxArray_real_T *AAs_d;
  int b_m;
  int b_n;
  int As_m;
  emxArray_int32_T *AAs_colidx;
  emxArray_int32_T *AAs_rowidx;
  coder_internal_sparse X;
  int AAs_m;
  int AAs_n;
  emxArray_creal_T *V;
  emxArray_creal_T *D;
  emxArray_creal_T *d;
  emxArray_int32_T *r4;
  emxArray_creal_T *y;
  emxArray_creal_T *b_b;
  emxArray_real_T *b_y;
  emxArray_int32_T *t5_colidx;
  emxArray_int32_T *t5_rowidx;
  coder_internal_sparse_1 expl_temp;
  emxArray_creal_T *b_d;
  coder_internal_sparse b_S;
  int k;
  double sizX[2];
  int W_m;
  int W_n;
  int c_n;
  emxArray_real_T *c_b;
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
  i3 = qsbar->size[0];
  qsbar->size[0] = 2 * adj->size[0];
  emxEnsureCapacity_real_T(qsbar, i3);
  idx = 2 * adj->size[0];
  for (i3 = 0; i3 < idx; i3++) {
    qsbar->data[i3] = 0.0;
  }

  if (2 > (Qs->size[1] << 1)) {
    i3 = 0;
    i4 = 1;
  } else {
    i3 = 1;
    i4 = 2;
  }

  if (1 > (adj->size[0] << 1) - 1) {
    i5 = 1;
    i6 = 0;
  } else {
    i5 = 2;
    i6 = (adj->size[0] << 1) - 1;
  }

  emxInit_int32_T(&r2, 2);
  nx = r2->size[0] * r2->size[1];
  r2->size[0] = 1;
  idx = div_s32_floor(i6 - 1, i5);
  r2->size[1] = idx + 1;
  emxEnsureCapacity_int32_T(r2, nx);
  for (i6 = 0; i6 <= idx; i6++) {
    r2->data[i6] = i5 * i6;
  }

  idx = r2->size[0] * r2->size[1];
  for (i5 = 0; i5 < idx; i5++) {
    qsbar->data[r2->data[i5]] = -Qs->data[i3 + i4 * i5];
  }

  if (1 > (Qs->size[1] << 1) - 1) {
    i3 = 1;
    i4 = 0;
  } else {
    i3 = 2;
    i4 = (Qs->size[1] << 1) - 1;
  }

  if (2 > qsbar->size[0]) {
    i5 = 0;
    i6 = 1;
  } else {
    i5 = 1;
    i6 = 2;
  }

  idx = div_s32_floor(i4 - 1, i3);
  for (i4 = 0; i4 <= idx; i4++) {
    qsbar->data[i5 + i6 * i4] = Qs->data[i3 * i4];
  }

  emxInit_int32_T(&one1, 1);

  //  One vectors
  i3 = one1->size[0];
  one1->size[0] = 2 * adj->size[0];
  emxEnsureCapacity_int32_T(one1, i3);
  idx = 2 * adj->size[0];
  for (i3 = 0; i3 < idx; i3++) {
    one1->data[i3] = 0;
  }

  if (1 > (adj->size[0] << 1) - 1) {
    i3 = 1;
    i4 = 0;
  } else {
    i3 = 2;
    i4 = (adj->size[0] << 1) - 1;
  }

  i5 = r2->size[0] * r2->size[1];
  r2->size[0] = 1;
  idx = div_s32_floor(i4 - 1, i3);
  r2->size[1] = idx + 1;
  emxEnsureCapacity_int32_T(r2, i5);
  for (i4 = 0; i4 <= idx; i4++) {
    r2->data[i4] = i3 * i4;
  }

  idx = r2->size[0] * r2->size[1];
  for (i3 = 0; i3 < idx; i3++) {
    one1->data[r2->data[i3]] = 1;
  }

  emxInit_real_T(&one2, 1);
  i3 = one2->size[0];
  one2->size[0] = 2 * adj->size[0];
  emxEnsureCapacity_real_T(one2, i3);
  idx = 2 * adj->size[0];
  for (i3 = 0; i3 < idx; i3++) {
    one2->data[i3] = 0.0;
  }

  if (2 > (adj->size[0] << 1)) {
    i3 = 1;
    i4 = 1;
    i5 = 0;
  } else {
    i3 = 2;
    i4 = 2;
    i5 = adj->size[0] << 1;
  }

  i6 = r2->size[0] * r2->size[1];
  r2->size[0] = 1;
  idx = div_s32_floor(i5 - i3, i4);
  r2->size[1] = idx + 1;
  emxEnsureCapacity_int32_T(r2, i6);
  for (i5 = 0; i5 <= idx; i5++) {
    r2->data[i5] = (i3 + i4 * i5) - 1;
  }

  idx = r2->size[0] * r2->size[1];
  for (i3 = 0; i3 < idx; i3++) {
    one2->data[r2->data[i3]] = 1.0;
  }

  emxFree_int32_T(&r2);
  emxInit_real_T(&b_Qs, 2);

  //  Get orthogonal complement of [z ones(n,1)]
  //  Desired kernel bases
  ii = Qs->size[1] << 1;
  i3 = b_Qs->size[0] * b_Qs->size[1];
  b_Qs->size[0] = ii;
  b_Qs->size[1] = 4;
  emxEnsureCapacity_real_T(b_Qs, i3);
  for (i3 = 0; i3 < ii; i3++) {
    b_Qs->data[i3] = Qs->data[i3];
  }

  idx = qsbar->size[0];
  for (i3 = 0; i3 < idx; i3++) {
    b_Qs->data[i3 + b_Qs->size[0]] = qsbar->data[i3];
  }

  idx = one1->size[0];
  for (i3 = 0; i3 < idx; i3++) {
    b_Qs->data[i3 + (b_Qs->size[0] << 1)] = one1->data[i3];
  }

  idx = one2->size[0];
  for (i3 = 0; i3 < idx; i3++) {
    b_Qs->data[i3 + b_Qs->size[0] * 3] = one2->data[i3];
  }

  emxInit_real_T(&U, 2);
  emxInit_real_T(&unusedU0, 2);
  svd(b_Qs, U, unusedU0, unusedU1);
  itra = (unsigned int)adj->size[0] << 1;
  emxFree_real_T(&b_Qs);
  emxFree_real_T(&unusedU0);
  if (5U > itra) {
    i3 = 0;
    i4 = 0;
  } else {
    i3 = 4;
    i4 = (int)itra;
  }

  emxInit_real_T(&Q, 2);
  idx = U->size[0];
  i5 = Q->size[0] * Q->size[1];
  Q->size[0] = idx;
  loop_ub = i4 - i3;
  Q->size[1] = loop_ub;
  emxEnsureCapacity_real_T(Q, i5);
  for (i4 = 0; i4 < loop_ub; i4++) {
    for (i5 = 0; i5 < idx; i5++) {
      Q->data[i5 + Q->size[0] * i4] = U->data[i5 + U->size[0] * (i3 + i4)];
    }
  }

  emxInit_real_T(&Qt, 2);

  //  Bases for the range space
  i4 = Qt->size[0] * Qt->size[1];
  Qt->size[0] = Q->size[1];
  Qt->size[1] = Q->size[0];
  emxEnsureCapacity_real_T(Qt, i4);
  idx = Q->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    jj = Q->size[1];
    for (i5 = 0; i5 < jj; i5++) {
      Qt->data[i5 + Qt->size[0] * i4] = Q->data[i4 + Q->size[0] * i5];
    }
  }

  c_emxInitStruct_coder_internal_(&I0);
  c_emxInitStruct_coder_internal_(&Z0);
  emxInit_real_T(&Z_d, 1);
  emxInit_int32_T(&Z_colidx, 1);
  emxInit_int32_T(&Z_rowidx, 1);
  emxInit_real_T(&C_d, 1);
  emxInit_real_T(&Aj, 1);
  emxInit_real_T(&Av, 1);
  emxInit_int32_T(&t3_colidx, 1);
  emxInit_int32_T(&t3_rowidx, 1);
  emxInit_int32_T(&t4_colidx, 1);
  emxInit_int32_T(&t4_rowidx, 1);

  //  Q transpose
  //  Preallocate variables
  speye(2.0 * ((double)adj->size[0] - 2.0), &I0);
  sparse(2.0 * ((double)adj->size[0] - 2.0), 2.0 * ((double)adj->size[0] - 2.0),
         Z0.d, Z0.colidx, Z0.rowidx, &Z0.m, &Z0.n);
  sparse(4.0 * ((double)adj->size[0] - 2.0), 4.0 * ((double)adj->size[0] - 2.0),
         Z_d, Z_colidx, Z_rowidx, &aoffset, &Z_n);

  //  Cost function's coefficient matrix: f = <C,X>
  sparse_horzcat(I0.d, I0.colidx, I0.rowidx, I0.m, I0.n, Z0.d, Z0.colidx,
                 Z0.rowidx, Z0.m, Z0.n, Aj, t3_colidx, t3_rowidx, &idx, &boffset);
  sparse_horzcat(Z0.d, Z0.colidx, Z0.rowidx, Z0.m, Z0.n, Z0.d, Z0.colidx,
                 Z0.rowidx, Z0.m, Z0.n, Av, t4_colidx, t4_rowidx, &jj, &nx);
  sparse_vertcat(Aj, t3_colidx, t3_rowidx, idx, boffset, Av, t4_colidx,
                 t4_rowidx, jj, nx, &Z0);
  i4 = C_d->size[0];
  C_d->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(C_d, i4);
  idx = Z0.d->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    C_d->data[i4] = Z0.d->data[i4];
  }

  emxInit_int32_T(&C_colidx, 1);
  i4 = C_colidx->size[0];
  C_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(C_colidx, i4);
  idx = Z0.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    C_colidx->data[i4] = Z0.colidx->data[i4];
  }

  emxInit_int32_T(&C_rowidx, 1);
  i4 = C_rowidx->size[0];
  C_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(C_rowidx, i4);
  idx = Z0.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    C_rowidx->data[i4] = Z0.rowidx->data[i4];
  }

  emxInit_boolean_T(&S, 2);

  //  Trace of gain matrix must be the specified value in 'trVal'
  trVal = 2.0 * ((double)adj->size[0] - 2.0);

  //  fixed value for trace
  // %%%%%%%%%%%%%%%%%%%%% Find total number of nonzero elements in A and b matrices 
  //  Number of elements in block [X]_11
  //  Number of elements in block [X]_12
  numElmB2 = 2.0 * ((double)adj->size[0] - 2.0);

  //  Zero-gain constraints for the given adjacency graph
  i4 = S->size[0] * S->size[1];
  S->size[0] = adj->size[0];
  S->size[1] = adj->size[1];
  emxEnsureCapacity_boolean_T(S, i4);
  idx = adj->size[0] * adj->size[1];
  for (i4 = 0; i4 < idx; i4++) {
    S->data[i4] = !(adj->data[i4] != 0.0);
  }

  emxInit_real_T(&Su, 2);
  emxInit_boolean_T(&r3, 2);
  emxInit_boolean_T(&t7_d, 1);
  diag(S, t7_d);
  b_diag(t7_d, r3);
  i4 = Su->size[0] * Su->size[1];
  Su->size[0] = S->size[0];
  Su->size[1] = S->size[1];
  emxEnsureCapacity_real_T(Su, i4);
  idx = S->size[0] * S->size[1];
  for (i4 = 0; i4 < idx; i4++) {
    Su->data[i4] = (double)S->data[i4] - (double)r3->data[i4];
  }

  emxFree_boolean_T(&r3);
  emxFree_boolean_T(&S);
  triu(Su);

  //  Upper triangular part
  nx = Su->size[0] * Su->size[1];
  emxInit_int32_T(&i, 1);
  if (nx == 0) {
    i->size[0] = 0;
    one1->size[0] = 0;
  } else {
    idx = 0;
    i4 = i->size[0];
    i->size[0] = nx;
    emxEnsureCapacity_int32_T(i, i4);
    i4 = one1->size[0];
    one1->size[0] = nx;
    emxEnsureCapacity_int32_T(one1, i4);
    ii = 1;
    jj = 1;
    exitg1 = false;
    while ((!exitg1) && (jj <= Su->size[1])) {
      guard1 = false;
      if (Su->data[(ii + Su->size[0] * (jj - 1)) - 1] != 0.0) {
        idx++;
        i->data[idx - 1] = ii;
        one1->data[idx - 1] = jj;
        if (idx >= nx) {
          exitg1 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        ii++;
        if (ii > Su->size[0]) {
          ii = 1;
          jj++;
        }
      }
    }

    if (nx == 1) {
      if (idx == 0) {
        i->size[0] = 0;
        one1->size[0] = 0;
      }
    } else if (1 > idx) {
      i->size[0] = 0;
      one1->size[0] = 0;
    } else {
      i4 = i->size[0];
      i->size[0] = idx;
      emxEnsureCapacity_int32_T(i, i4);
      i4 = one1->size[0];
      one1->size[0] = idx;
      emxEnsureCapacity_int32_T(one1, i4);
    }
  }

  i4 = qsbar->size[0];
  qsbar->size[0] = i->size[0];
  emxEnsureCapacity_real_T(qsbar, i4);
  idx = i->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    qsbar->data[i4] = i->data[i4];
  }

  //  Find location of nonzero entries
  //  Number of constraints
  //  Number of elements in block [X]_22. Found "-m" empirically.
  a = 2.0 * ((double)adj->size[0] - 2.0);

  //  Number of elements for trace
  //  Number of elements for symmetry
  //  Number of elements for pinning down the b-vector
  //  Total number of elements
  numElmAtot = (((((2.0 * ((double)adj->size[0] - 2.0) - 1.0) * 2.0 + 2.0 *
                   ((double)adj->size[0] - 2.0) * (2.0 * ((double)adj->size[0] -
    2.0) - 1.0) / 2.0) + (2.0 * ((double)adj->size[0] - 2.0) + 2.0 * ((double)
    adj->size[0] - 2.0) * (2.0 * ((double)adj->size[0] - 2.0) - 1.0))) + ((0.5 *
    ((double)adj->size[0] - 2.0) * (((double)adj->size[0] - 2.0) + 1.0) * 4.0 +
    (double)qsbar->size[0] * (2.0 * (a * a))) - ((double)adj->size[0] - 2.0))) +
                2.0 * ((double)adj->size[0] - 2.0)) + 4.0 * ((double)adj->size[0]
    - 2.0) * (4.0 * ((double)adj->size[0] - 2.0) - 1.0);

  // %%%%%%%%%%%%%%%%%%%%% Preallocate sparse matrices A & b
  //
  //  Constraint: A * vec(X) = b
  //
  //  Indices of A: entry [Ai(k), Aj(k)] takes value of Av(k)
  //  Each row of matrix A will represent a constraint
  i4 = one2->size[0];
  idx = (int)numElmAtot;
  one2->size[0] = idx;
  emxEnsureCapacity_real_T(one2, i4);
  for (i4 = 0; i4 < idx; i4++) {
    one2->data[i4] = 0.0;
  }

  i4 = Aj->size[0];
  Aj->size[0] = idx;
  emxEnsureCapacity_real_T(Aj, i4);
  for (i4 = 0; i4 < idx; i4++) {
    Aj->data[i4] = 0.0;
  }

  i4 = Av->size[0];
  Av->size[0] = idx;
  emxEnsureCapacity_real_T(Av, i4);
  for (i4 = 0; i4 < idx; i4++) {
    Av->data[i4] = 0.0;
  }

  emxInit_real_T(&bi, 1);
  i4 = bi->size[0];
  idx = (int)((numElmB2 + 1.0) + 1.0);
  bi->size[0] = idx;
  emxEnsureCapacity_real_T(bi, i4);
  for (i4 = 0; i4 < idx; i4++) {
    bi->data[i4] = 0.0;
  }

  emxInit_real_T(&bv, 1);
  i4 = bv->size[0];
  bv->size[0] = idx;
  emxEnsureCapacity_real_T(bv, i4);
  for (i4 = 0; i4 < idx; i4++) {
    bv->data[i4] = 0.0;
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
  i4 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i <= i4 - 2; b_i++) {
    itrr++;
    itra++;
    i5 = (int)itra - 1;
    one2->data[i5] = itrr;
    Aj->data[i5] = 1.0;
    Av->data[i5] = 1.0;
    itra++;
    i5 = (int)itra - 1;
    one2->data[i5] = itrr;
    Aj->data[i5] = ((2.0 + (double)b_i) - 1.0) * (4.0 * ((double)n - 2.0)) +
      (2.0 + (double)b_i);
    Av->data[i5] = -1.0;
  }

  //  Off-diagonal entries should be zero
  i4 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i <= i4 - 2; b_i++) {
    i5 = (m << 1) - b_i;
    for (j = 0; j <= i5 - 2; j++) {
      itrr++;
      itra++;
      i6 = (int)itra - 1;
      one2->data[i6] = itrr;
      Aj->data[i6] = ((1.0 + (double)b_i) - 1.0) * (4.0 * ((double)n - 2.0)) +
        (((1.0 + (double)b_i) + 1.0) + (double)j);
      Av->data[i6] = 1.0;
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_12
  //  Diagonal entries should be 1
  i4 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i < i4; b_i++) {
    itrr++;
    itra++;
    i5 = (int)itra - 1;
    one2->data[i5] = itrr;
    Aj->data[i5] = ((1.0 + (double)b_i) - 1.0) * (4.0 * ((double)n - 2.0)) +
      ((1.0 + (double)b_i) + 2.0 * (double)m);
    Av->data[i5] = 1.0;
    itrb++;
    i5 = (int)itrb - 1;
    bi->data[i5] = itrr;
    bv->data[i5] = 1.0;
  }

  //  Other entries should be 0
  i4 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i < i4; b_i++) {
    i5 = m << 1;
    for (j = 0; j < i5; j++) {
      if ((unsigned int)b_i != (unsigned int)j) {
        itrr++;
        itra++;
        i6 = (int)itra - 1;
        one2->data[i6] = itrr;
        Aj->data[i6] = ((1.0 + (double)b_i) - 1.0) * (4.0 * ((double)n - 2.0)) +
          ((1.0 + (double)j) + 2.0 * (double)m);
        Av->data[i6] = 1.0;
      }
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_22
  //  Scaled rotation matrix structure constraints
  i4 = adj->size[0];
  for (b_i = 0; b_i <= i4 - 3; b_i++) {
    i5 = n - b_i;
    for (j = 0; j <= i5 - 3; j++) {
      numElmAtot = (double)((unsigned int)b_i + j) + 1.0;

      //  Diagonal entries should be equal
      itrr++;
      itra++;
      i6 = (int)itra - 1;
      one2->data[i6] = itrr;
      numElmB2 = 2.0 * (double)m + 2.0 * ((1.0 + (double)b_i) - 1.0);
      cdiff = 2.0 * (double)m + 2.0 * (numElmAtot - 1.0);
      a = ((numElmB2 + 1.0) - 1.0) * (4.0 * ((double)n - 2.0));
      Aj->data[i6] = a + (cdiff + 1.0);
      Av->data[i6] = 1.0;
      itra++;
      i6 = (int)itra - 1;
      one2->data[i6] = itrr;
      numElmB2 = ((numElmB2 + 2.0) - 1.0) * (4.0 * ((double)n - 2.0));
      Aj->data[i6] = numElmB2 + (cdiff + 2.0);
      Av->data[i6] = -1.0;

      //  Off-diagonal entries have same value with different sign
      if ((unsigned int)(1 + b_i) == (unsigned int)numElmAtot) {
        itrr++;
        itra++;
        i6 = (int)itra - 1;
        one2->data[i6] = itrr;
        Aj->data[i6] = (((2.0 * (double)m + 2.0 * ((1.0 + (double)b_i) - 1.0)) +
                         1.0) - 1.0) * (4.0 * ((double)n - 2.0)) + ((2.0 *
          (double)m + 2.0 * (numElmAtot - 1.0)) + 2.0);
        Av->data[i6] = 1.0;
      } else {
        itrr++;
        itra++;
        i6 = (int)itra - 1;
        one2->data[i6] = itrr;
        Aj->data[i6] = a + ((2.0 * (double)m + 2.0 * (numElmAtot - 1.0)) + 2.0);
        Av->data[i6] = 1.0;
        itra++;
        i6 = (int)itra - 1;
        one2->data[i6] = itrr;
        Aj->data[i6] = numElmB2 + ((2.0 * (double)m + 2.0 * (numElmAtot - 1.0))
          + 1.0);
        Av->data[i6] = 1.0;
      }
    }
  }

  //  Zero constraints due to the adjacency matrix
  i4 = qsbar->size[0];
  emxInit_real_T(&b_a, 1);
  emxInit_real_T(&b, 2);
  for (b_i = 0; b_i < i4; b_i++) {
    //  Diagonal terms of A_iijj, just do first column/row since a == a
    idx = Qt->size[0];
    i5 = 2 * (one1->data[b_i] - 1) + 1;
    i6 = b_a->size[0];
    b_a->size[0] = idx;
    emxEnsureCapacity_real_T(b_a, i6);
    for (i6 = 0; i6 < idx; i6++) {
      b_a->data[i6] = Qt->data[i6 + Qt->size[0] * (i5 - 1)];
    }

    i5 = (int)(2.0 * (qsbar->data[b_i] - 1.0) + 1.0);
    i6 = b->size[0] * b->size[1];
    b->size[0] = 1;
    b->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b, i6);
    for (i6 = 0; i6 < loop_ub; i6++) {
      b->data[i6] = Q->data[(i5 + Q->size[0] * i6) - 1];
    }

    i5 = Su->size[0] * Su->size[1];
    Su->size[0] = b_a->size[0];
    Su->size[1] = b->size[1];
    emxEnsureCapacity_real_T(Su, i5);
    idx = b_a->size[0];
    for (i5 = 0; i5 < idx; i5++) {
      jj = b->size[1];
      for (i6 = 0; i6 < jj; i6++) {
        Su->data[i5 + Su->size[0] * i6] = b_a->data[i5] * b->data[i6];
      }
    }

    itrr++;
    i5 = m << 1;
    for (ii = 0; ii < i5; ii++) {
      i6 = m << 1;
      for (jj = 0; jj < i6; jj++) {
        itra++;
        nx = (int)itra - 1;
        one2->data[nx] = itrr;
        Aj->data[nx] = ((2.0 * (double)m + (1.0 + (double)ii)) - 1.0) * (4.0 *
          ((double)n - 2.0)) + (2.0 * (double)m + (1.0 + (double)jj));
        Av->data[nx] = Su->data[ii + Su->size[0] * jj];
      }
    }

    //  Off-diagonal terms of A_iijj
    idx = Qt->size[0];
    i5 = 2 * (one1->data[b_i] - 1) + 1;
    i6 = b_a->size[0];
    b_a->size[0] = idx;
    emxEnsureCapacity_real_T(b_a, i6);
    for (i6 = 0; i6 < idx; i6++) {
      b_a->data[i6] = Qt->data[i6 + Qt->size[0] * (i5 - 1)];
    }

    i5 = (int)(2.0 * (qsbar->data[b_i] - 1.0) + 2.0);
    i6 = b->size[0] * b->size[1];
    b->size[0] = 1;
    b->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b, i6);
    for (i6 = 0; i6 < loop_ub; i6++) {
      b->data[i6] = Q->data[(i5 + Q->size[0] * i6) - 1];
    }

    i5 = Su->size[0] * Su->size[1];
    Su->size[0] = b_a->size[0];
    Su->size[1] = b->size[1];
    emxEnsureCapacity_real_T(Su, i5);
    idx = b_a->size[0];
    for (i5 = 0; i5 < idx; i5++) {
      jj = b->size[1];
      for (i6 = 0; i6 < jj; i6++) {
        Su->data[i5 + Su->size[0] * i6] = b_a->data[i5] * b->data[i6];
      }
    }

    itrr++;
    i5 = m << 1;
    for (ii = 0; ii < i5; ii++) {
      i6 = m << 1;
      for (jj = 0; jj < i6; jj++) {
        itra++;
        one2->data[(int)itra - 1] = itrr;
        Aj->data[(int)itra - 1] = ((2.0 * (double)m + (1.0 + (double)ii)) - 1.0)
          * (4.0 * ((double)n - 2.0)) + (2.0 * (double)m + (1.0 + (double)jj));
        Av->data[(int)itra - 1] = Su->data[ii + Su->size[0] * jj];
      }
    }
  }

  //  Trace of gain materix must be the specified value in 'trVal'
  itrr++;
  i4 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i < i4; b_i++) {
    itra++;
    numElmAtot = 2.0 * ((double)n - 2.0) + (1.0 + (double)b_i);
    i5 = (int)itra - 1;
    one2->data[i5] = itrr;
    Aj->data[i5] = (numElmAtot - 1.0) * (4.0 * ((double)n - 2.0)) + numElmAtot;
    Av->data[i5] = 1.0;
  }

  itrb++;
  bi->data[(int)itrb - 1] = itrr;
  bv->data[(int)itrb - 1] = trVal;

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Symmetry
  i4 = (adj->size[0] - 2) << 2;
  for (b_i = 0; b_i <= i4 - 2; b_i++) {
    i5 = (m << 2) - b_i;
    for (j = 0; j <= i5 - 2; j++) {
      numElmAtot = ((1.0 + (double)b_i) + 1.0) + (double)j;

      //  Symmetric entries should be equal
      itrr++;
      itra++;
      i6 = (int)itra - 1;
      one2->data[i6] = itrr;
      Aj->data[i6] = ((1.0 + (double)b_i) - 1.0) * (4.0 * ((double)n - 2.0)) +
        numElmAtot;
      Av->data[i6] = 1.0;
      itra++;
      i6 = (int)itra - 1;
      one2->data[i6] = itrr;
      Aj->data[i6] = (numElmAtot - 1.0) * (4.0 * ((double)n - 2.0)) + (1.0 +
        (double)b_i);
      Av->data[i6] = -1.0;
    }
  }

  emxInit_real_T(&A_d, 1);

  //  Last element set to fix the size of b
  itrb++;
  i4 = (int)itrb - 1;
  bi->data[i4] = itrr;
  bv->data[i4] = 0.0;

  //  % Remove any additional entries
  //  Ai(itra+1:end) = [];
  //  Aj(itra+1:end) = [];
  //  Av(itra+1:end) = [];
  //
  //  bi(itrb+1:end) = [];
  //  bv(itrb+1:end) = [];
  //  Make sparse matrices
  b_sparse(one2, Aj, Av, &Z0);
  i4 = A_d->size[0];
  A_d->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(A_d, i4);
  idx = Z0.d->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    A_d->data[i4] = Z0.d->data[i4];
  }

  emxInit_int32_T(&A_colidx, 1);
  i4 = A_colidx->size[0];
  A_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(A_colidx, i4);
  idx = Z0.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    A_colidx->data[i4] = Z0.colidx->data[i4];
  }

  emxInit_int32_T(&A_rowidx, 1);
  i4 = A_rowidx->size[0];
  A_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(A_rowidx, i4);
  idx = Z0.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    A_rowidx->data[i4] = Z0.rowidx->data[i4];
  }

  A_m = Z0.m;
  jj = Z0.n;
  i4 = Aj->size[0];
  Aj->size[0] = bi->size[0];
  emxEnsureCapacity_real_T(Aj, i4);
  idx = bi->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    Aj->data[i4] = 1.0;
  }

  b_sparse(bi, Aj, bv, &Z0);
  i4 = bi->size[0];
  bi->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(bi, i4);
  idx = Z0.d->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    bi->data[i4] = Z0.d->data[i4];
  }

  emxInit_int32_T(&b_colidx, 1);
  i4 = b_colidx->size[0];
  b_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(b_colidx, i4);
  idx = Z0.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    b_colidx->data[i4] = Z0.colidx->data[i4];
  }

  emxInit_int32_T(&b_rowidx, 1);
  i4 = b_rowidx->size[0];
  b_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(b_rowidx, i4);
  idx = Z0.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    b_rowidx->data[i4] = Z0.rowidx->data[i4];
  }

  emxInit_int32_T(&As_colidx, 1);
  emxInit_int32_T(&As_rowidx, 1);
  emxInit_real_T(&AAs_d, 1);
  b_m = Z0.m;
  b_n = Z0.n;

  //  Size of optimization variable
  itrr = 4.0 * ((double)adj->size[0] - 2.0);

  //  ADMM algorithm--full eigendecomposition
  sparse_transpose(A_d, A_colidx, A_rowidx, A_m, jj, bv, As_colidx, As_rowidx,
                   &As_m, &ii);

  //  Dual operator
  sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, bv, As_colidx, As_rowidx, ii, &Z0);
  i4 = AAs_d->size[0];
  AAs_d->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(AAs_d, i4);
  idx = Z0.d->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    AAs_d->data[i4] = Z0.d->data[i4];
  }

  emxInit_int32_T(&AAs_colidx, 1);
  i4 = AAs_colidx->size[0];
  AAs_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(AAs_colidx, i4);
  idx = Z0.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    AAs_colidx->data[i4] = Z0.colidx->data[i4];
  }

  emxInit_int32_T(&AAs_rowidx, 1);
  i4 = AAs_rowidx->size[0];
  AAs_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(AAs_rowidx, i4);
  idx = Z0.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    AAs_rowidx->data[i4] = Z0.rowidx->data[i4];
  }

  c_emxInitStruct_coder_internal_(&X);
  AAs_m = Z0.m;
  AAs_n = Z0.n;

  //  Penalty
  //  Precision for positive eig vals
  //  Stop criteria
  //  Threshold based on change in X updates
  //  Percentage threshold based on the trace value. If trace of X2 reaches within the specified percentage, the algorithm stops. 
  //  Maximum # of iterations
  //  Initialize:
  sparse_horzcat(I0.d, I0.colidx, I0.rowidx, I0.m, I0.n, I0.d, I0.colidx,
                 I0.rowidx, I0.m, I0.n, Aj, t3_colidx, t3_rowidx, &idx, &boffset);
  sparse_horzcat(I0.d, I0.colidx, I0.rowidx, I0.m, I0.n, I0.d, I0.colidx,
                 I0.rowidx, I0.m, I0.n, Av, t4_colidx, t4_rowidx, &jj, &nx);
  sparse_vertcat(Aj, t3_colidx, t3_rowidx, idx, boffset, Av, t4_colidx,
                 t4_rowidx, jj, nx, &X);
  b_i = 0;
  emxInit_creal_T(&V, 2);
  emxInit_creal_T(&D, 2);
  emxInit_creal_T(&d, 1);
  emxInit_int32_T(&r4, 1);
  emxInit_creal_T(&y, 2);
  emxInit_creal_T(&b_b, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_int32_T(&t5_colidx, 1);
  emxInit_int32_T(&t5_rowidx, 1);
  d_emxInitStruct_coder_internal_(&expl_temp);
  emxInit_creal_T(&b_d, 1);
  exitg1 = false;
  while ((!exitg1) && (b_i < 10)) {
    // %%%%%% Update for y
    sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
    jj = X.colidx->data[X.colidx->size[0] - 1];
    if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
      ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
    } else {
      ii = 0;
    }

    i4 = I0.d->size[0];
    I0.d->size[0] = ii + 1;
    emxEnsureCapacity_real_T(I0.d, i4);
    for (i4 = 0; i4 <= ii; i4++) {
      I0.d->data[i4] = 0.0;
    }

    for (k = 0; k <= jj - 2; k++) {
      I0.d->data[k] = X.d->data[k];
    }

    b_sparse_fillIn(&I0);
    sparse_copy(b_colidx, b_rowidx, b_m, b_n, &Z0);
    jj = b_colidx->data[b_colidx->size[0] - 1];
    if (b_colidx->data[b_colidx->size[0] - 1] - 1 >= 1) {
      ii = b_colidx->data[b_colidx->size[0] - 1] - 2;
    } else {
      ii = 0;
    }

    i4 = Z0.d->size[0];
    Z0.d->size[0] = ii + 1;
    emxEnsureCapacity_real_T(Z0.d, i4);
    for (i4 = 0; i4 <= ii; i4++) {
      Z0.d->data[i4] = 0.0;
    }

    for (k = 0; k <= jj - 2; k++) {
      Z0.d->data[k] = bi->data[k];
    }

    b_sparse_fillIn(&Z0);
    sparse_minus(C_d, C_colidx, C_rowidx, Z_d, Z_colidx, Z_rowidx, aoffset, Z_n,
                 Aj, t3_colidx, t3_rowidx, &idx, &boffset);
    sparse_minus(Aj, t3_colidx, t3_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m,
                 I0.n, Av, t4_colidx, t4_rowidx, &jj, &nx);
    vec(Av, t4_colidx, t4_rowidx, jj, nx, b_a, t5_colidx, t5_rowidx, &ii);
    b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, b_a, t5_colidx, t5_rowidx,
                    &expl_temp);
    i4 = one2->size[0];
    one2->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(one2, i4);
    idx = expl_temp.d->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      one2->data[i4] = expl_temp.d->data[i4];
    }

    i4 = one1->size[0];
    one1->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(one1, i4);
    idx = expl_temp.colidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      one1->data[i4] = expl_temp.colidx->data[i4];
    }

    i4 = i->size[0];
    i->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(i, i4);
    idx = expl_temp.rowidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      i->data[i4] = expl_temp.rowidx->data[i4];
    }

    sparse_plus(one2, one1, i, Z0.d, Z0.colidx, Z0.rowidx, Z0.m, b_a, t5_colidx,
                t5_rowidx, &ii);
    sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, b_a, t5_colidx,
                    t5_rowidx, ii, &expl_temp);
    i4 = one2->size[0];
    one2->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(one2, i4);
    idx = expl_temp.d->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      one2->data[i4] = expl_temp.d->data[i4];
    }

    i4 = one1->size[0];
    one1->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(one1, i4);
    idx = expl_temp.colidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      one1->data[i4] = expl_temp.colidx->data[i4];
    }

    i4 = i->size[0];
    i->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(i, i4);
    idx = expl_temp.rowidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      i->data[i4] = expl_temp.rowidx->data[i4];
    }

    // %%%%%% Update for S
    sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
    jj = X.colidx->data[X.colidx->size[0] - 1];
    if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
      ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
    } else {
      ii = 0;
    }

    i4 = I0.d->size[0];
    I0.d->size[0] = ii + 1;
    emxEnsureCapacity_real_T(I0.d, i4);
    for (i4 = 0; i4 <= ii; i4++) {
      I0.d->data[i4] = 0.0;
    }

    for (k = 0; k <= jj - 2; k++) {
      I0.d->data[k] = X.d->data[k];
    }

    b_sparse_fillIn(&I0);
    b_sparse_mtimes(bv, As_colidx, As_rowidx, As_m, one2, one1, i, &expl_temp);
    i4 = b_a->size[0];
    b_a->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(b_a, i4);
    idx = expl_temp.d->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      b_a->data[i4] = expl_temp.d->data[i4];
    }

    i4 = t5_colidx->size[0];
    t5_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t5_colidx, i4);
    idx = expl_temp.colidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      t5_colidx->data[i4] = expl_temp.colidx->data[i4];
    }

    i4 = t5_rowidx->size[0];
    t5_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t5_rowidx, i4);
    idx = expl_temp.rowidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      t5_rowidx->data[i4] = expl_temp.rowidx->data[i4];
    }

    sizX[0] = itrr;
    sizX[1] = itrr;
    sparse_reshape(b_a, t5_colidx, t5_rowidx, sizX, Aj, t3_colidx, t3_rowidx,
                   &idx, &boffset);
    sparse_minus(C_d, C_colidx, C_rowidx, Aj, t3_colidx, t3_rowidx, idx, boffset,
                 Av, t4_colidx, t4_rowidx, &jj, &nx);
    sparse_minus(Av, t4_colidx, t4_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m,
                 I0.n, one2, one1, i, &W_m, &W_n);
    sparse_transpose(one2, one1, i, W_m, W_n, Aj, t3_colidx, t3_rowidx, &idx,
                     &boffset);
    b_sparse_plus(one2, one1, i, Aj, t3_colidx, t3_rowidx, idx, boffset, Av,
                  t4_colidx, t4_rowidx, &jj, &nx);
    sparse_rdivide(Av, t4_colidx, t4_rowidx, jj, nx, &Z0);
    i4 = one2->size[0];
    one2->size[0] = Z0.d->size[0];
    emxEnsureCapacity_real_T(one2, i4);
    idx = Z0.d->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      one2->data[i4] = Z0.d->data[i4];
    }

    i4 = one1->size[0];
    one1->size[0] = Z0.colidx->size[0];
    emxEnsureCapacity_int32_T(one1, i4);
    idx = Z0.colidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      one1->data[i4] = Z0.colidx->data[i4];
    }

    i4 = i->size[0];
    i->size[0] = Z0.rowidx->size[0];
    emxEnsureCapacity_int32_T(i, i4);
    idx = Z0.rowidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      i->data[i4] = Z0.rowidx->data[i4];
    }

    W_m = Z0.m;
    W_n = Z0.n;
    b_sparse_full(one2, one1, i, Z0.m, Z0.n, Su);
    eig(Su, V, D);
    c_diag(D, d);
    jj = d->size[0] - 1;
    ii = 0;
    for (nx = 0; nx <= jj; nx++) {
      if (d->data[nx].re > 1.0E-5) {
        ii++;
      }
    }

    i4 = r4->size[0];
    r4->size[0] = ii;
    emxEnsureCapacity_int32_T(r4, i4);
    ii = 0;
    for (nx = 0; nx <= jj; nx++) {
      if (d->data[nx].re > 1.0E-5) {
        r4->data[ii] = nx + 1;
        ii++;
      }
    }

    idx = V->size[0];
    i4 = D->size[0] * D->size[1];
    D->size[0] = idx;
    D->size[1] = r4->size[0];
    emxEnsureCapacity_creal_T(D, i4);
    jj = r4->size[0];
    for (i4 = 0; i4 < jj; i4++) {
      for (i5 = 0; i5 < idx; i5++) {
        D->data[i5 + D->size[0] * i4] = V->data[i5 + V->size[0] * (r4->data[i4]
          - 1)];
      }
    }

    i4 = b_d->size[0];
    b_d->size[0] = r4->size[0];
    emxEnsureCapacity_creal_T(b_d, i4);
    idx = r4->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      b_d->data[i4] = d->data[r4->data[i4] - 1];
    }

    d_diag(b_d, b_b);
    if ((r4->size[0] == 1) || (b_b->size[0] == 1)) {
      i4 = y->size[0] * y->size[1];
      y->size[0] = D->size[0];
      y->size[1] = b_b->size[1];
      emxEnsureCapacity_creal_T(y, i4);
      idx = D->size[0];
      for (i4 = 0; i4 < idx; i4++) {
        jj = b_b->size[1];
        for (i5 = 0; i5 < jj; i5++) {
          y->data[i4 + y->size[0] * i5].re = 0.0;
          y->data[i4 + y->size[0] * i5].im = 0.0;
          ii = D->size[1];
          for (i6 = 0; i6 < ii; i6++) {
            numElmAtot = D->data[i4 + D->size[0] * i6].re * b_b->data[i6 +
              b_b->size[0] * i5].re - D->data[i4 + D->size[0] * i6].im *
              b_b->data[i6 + b_b->size[0] * i5].im;
            numElmB2 = D->data[i4 + D->size[0] * i6].re * b_b->data[i6 +
              b_b->size[0] * i5].im + D->data[i4 + D->size[0] * i6].im *
              b_b->data[i6 + b_b->size[0] * i5].re;
            y->data[i4 + y->size[0] * i5].re += numElmAtot;
            y->data[i4 + y->size[0] * i5].im += numElmB2;
          }
        }
      }
    } else {
      i4 = V->size[0];
      jj = r4->size[0];
      c_n = b_b->size[1];
      i5 = V->size[0];
      i6 = y->size[0] * y->size[1];
      y->size[0] = i5;
      y->size[1] = b_b->size[1];
      emxEnsureCapacity_creal_T(y, i6);
      for (j = 0; j < c_n; j++) {
        idx = j * i4;
        boffset = j * jj;
        for (nx = 0; nx < i4; nx++) {
          i5 = idx + nx;
          y->data[i5].re = 0.0;
          y->data[i5].im = 0.0;
        }

        for (k = 0; k < jj; k++) {
          aoffset = k * i4;
          ii = boffset + k;
          numElmAtot = b_b->data[ii].re;
          numElmB2 = b_b->data[ii].im;
          for (nx = 0; nx < i4; nx++) {
            ii = aoffset + nx;
            cdiff = numElmAtot * D->data[ii].re - numElmB2 * D->data[ii].im;
            a = numElmAtot * D->data[ii].im + numElmB2 * D->data[ii].re;
            i5 = idx + nx;
            y->data[i5].re += cdiff;
            y->data[i5].im += a;
          }
        }
      }
    }

    idx = V->size[0];
    i4 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = r4->size[0];
    b_b->size[1] = idx;
    emxEnsureCapacity_creal_T(b_b, i4);
    for (i4 = 0; i4 < idx; i4++) {
      jj = r4->size[0];
      for (i5 = 0; i5 < jj; i5++) {
        b_b->data[i5 + b_b->size[0] * i4] = V->data[i4 + V->size[0] * (r4->
          data[i5] - 1)];
      }
    }

    if ((y->size[1] == 1) || (b_b->size[0] == 1)) {
      i4 = D->size[0] * D->size[1];
      D->size[0] = y->size[0];
      D->size[1] = b_b->size[1];
      emxEnsureCapacity_creal_T(D, i4);
      idx = y->size[0];
      for (i4 = 0; i4 < idx; i4++) {
        jj = b_b->size[1];
        for (i5 = 0; i5 < jj; i5++) {
          D->data[i4 + D->size[0] * i5].re = 0.0;
          D->data[i4 + D->size[0] * i5].im = 0.0;
          ii = y->size[1];
          for (i6 = 0; i6 < ii; i6++) {
            numElmAtot = y->data[i4 + y->size[0] * i6].re * b_b->data[i6 +
              b_b->size[0] * i5].re - y->data[i4 + y->size[0] * i6].im *
              b_b->data[i6 + b_b->size[0] * i5].im;
            numElmB2 = y->data[i4 + y->size[0] * i6].re * b_b->data[i6 +
              b_b->size[0] * i5].im + y->data[i4 + y->size[0] * i6].im *
              b_b->data[i6 + b_b->size[0] * i5].re;
            D->data[i4 + D->size[0] * i5].re += numElmAtot;
            D->data[i4 + D->size[0] * i5].im += numElmB2;
          }
        }
      }
    } else {
      m = y->size[0];
      jj = y->size[1];
      c_n = b_b->size[1];
      i4 = D->size[0] * D->size[1];
      D->size[0] = y->size[0];
      D->size[1] = b_b->size[1];
      emxEnsureCapacity_creal_T(D, i4);
      for (j = 0; j < c_n; j++) {
        idx = j * m;
        boffset = j * jj;
        for (nx = 0; nx < m; nx++) {
          i4 = idx + nx;
          D->data[i4].re = 0.0;
          D->data[i4].im = 0.0;
        }

        for (k = 0; k < jj; k++) {
          aoffset = k * m;
          ii = boffset + k;
          numElmAtot = b_b->data[ii].re;
          numElmB2 = b_b->data[ii].im;
          for (nx = 0; nx < m; nx++) {
            ii = aoffset + nx;
            cdiff = numElmAtot * y->data[ii].re - numElmB2 * y->data[ii].im;
            a = numElmAtot * y->data[ii].im + numElmB2 * y->data[ii].re;
            i4 = idx + nx;
            D->data[i4].re += cdiff;
            D->data[i4].im += a;
          }
        }
      }
    }

    i4 = Su->size[0] * Su->size[1];
    Su->size[0] = D->size[0];
    Su->size[1] = D->size[1];
    emxEnsureCapacity_real_T(Su, i4);
    idx = D->size[0] * D->size[1];
    for (i4 = 0; i4 < idx; i4++) {
      Su->data[i4] = D->data[i4].re;
    }

    c_sparse(Su, Z_d, Z_colidx, Z_rowidx, &aoffset, &Z_n);

    // %%%%%% Update for X
    c_emxCopyStruct_coder_internal_(&Z0, &X);
    sparse_minus(Z_d, Z_colidx, Z_rowidx, one2, one1, i, W_m, W_n, Aj, t3_colidx,
                 t3_rowidx, &idx, &boffset);
    b_sparse_rdivide(Aj, t3_colidx, t3_rowidx, idx, boffset, &X);

    // %%%%%% Stop criteria
    b_sparse_parenReference(Z0.d, Z0.colidx, Z0.rowidx, Z0.m, Z0.n, b_a,
      t5_colidx, t5_rowidx, &ii);
    b_sparse_parenReference(X.d, X.colidx, X.rowidx, X.m, X.n, one2, one1, i,
      &ii);
    b_sparse_minus(b_a, t5_colidx, t5_rowidx, one2, one1, i, ii, Aj, t3_colidx,
                   t3_rowidx, &jj);
    sparse_abs(Aj, t3_colidx, t3_rowidx, jj, &expl_temp);
    i4 = b_a->size[0];
    b_a->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(b_a, i4);
    idx = expl_temp.d->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      b_a->data[i4] = expl_temp.d->data[i4];
    }

    i4 = t5_colidx->size[0];
    t5_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t5_colidx, i4);
    idx = expl_temp.colidx->size[0];
    for (i4 = 0; i4 < idx; i4++) {
      t5_colidx->data[i4] = expl_temp.colidx->data[i4];
    }

    ii = expl_temp.m;
    sum(b_a, t5_colidx, expl_temp.m, one2, i, t3_colidx);
    sparse_lt(one2, i, t7_d, one1, t3_colidx);
    if (c_sparse_full(t7_d, one1)) {
      //  change in X is small
      exitg1 = true;
    } else {
      //  trace of sparse matrix for MATLAB Coder
      a = 2.0 * ((double)n - 2.0) + 1.0;
      if (X.m < a) {
        b->size[0] = 1;
        b->size[1] = 0;
      } else if (a == a) {
        i4 = b->size[0] * b->size[1];
        b->size[0] = 1;
        idx = X.m - (int)a;
        b->size[1] = idx + 1;
        emxEnsureCapacity_real_T(b, i4);
        for (i4 = 0; i4 <= idx; i4++) {
          b->data[i4] = a + (double)i4;
        }
      } else {
        numElmAtot = std::floor(((double)X.m - a) + 0.5);
        numElmB2 = a + numElmAtot;
        cdiff = numElmB2 - (double)X.m;
        ii = (int)std::abs(a);
        jj = (int)std::abs((double)X.m);
        if (ii > jj) {
          jj = ii;
        }

        if (std::abs(cdiff) < 4.4408920985006262E-16 * (double)jj) {
          numElmAtot++;
          numElmB2 = X.m;
        } else if (cdiff > 0.0) {
          numElmB2 = a + (numElmAtot - 1.0);
        } else {
          numElmAtot++;
        }

        if (numElmAtot >= 0.0) {
          c_n = (int)numElmAtot;
        } else {
          c_n = 0;
        }

        i4 = b->size[0] * b->size[1];
        b->size[0] = 1;
        b->size[1] = c_n;
        emxEnsureCapacity_real_T(b, i4);
        if (c_n > 0) {
          b->data[0] = a;
          if (c_n > 1) {
            b->data[c_n - 1] = numElmB2;
            ii = (c_n - 1) >> 1;
            for (k = 0; k <= ii - 2; k++) {
              b->data[1 + k] = a + (1.0 + (double)k);
              b->data[(c_n - k) - 2] = numElmB2 - (1.0 + (double)k);
            }

            if (ii << 1 == c_n - 1) {
              b->data[ii] = (a + numElmB2) / 2.0;
            } else {
              b->data[ii] = a + (double)ii;
              b->data[ii + 1] = numElmB2 - (double)ii;
            }
          }
        }
      }

      a = 2.0 * ((double)n - 2.0) + 1.0;
      if (X.n < a) {
        b_y->size[0] = 1;
        b_y->size[1] = 0;
      } else if (a == a) {
        i4 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        idx = X.n - (int)a;
        b_y->size[1] = idx + 1;
        emxEnsureCapacity_real_T(b_y, i4);
        for (i4 = 0; i4 <= idx; i4++) {
          b_y->data[i4] = a + (double)i4;
        }
      } else {
        numElmAtot = std::floor(((double)X.n - a) + 0.5);
        numElmB2 = a + numElmAtot;
        cdiff = numElmB2 - (double)X.n;
        ii = (int)std::abs(a);
        jj = (int)std::abs((double)X.n);
        if (ii > jj) {
          jj = ii;
        }

        if (std::abs(cdiff) < 4.4408920985006262E-16 * (double)jj) {
          numElmAtot++;
          numElmB2 = X.n;
        } else if (cdiff > 0.0) {
          numElmB2 = a + (numElmAtot - 1.0);
        } else {
          numElmAtot++;
        }

        if (numElmAtot >= 0.0) {
          c_n = (int)numElmAtot;
        } else {
          c_n = 0;
        }

        i4 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        b_y->size[1] = c_n;
        emxEnsureCapacity_real_T(b_y, i4);
        if (c_n > 0) {
          b_y->data[0] = a;
          if (c_n > 1) {
            b_y->data[c_n - 1] = numElmB2;
            ii = (c_n - 1) >> 1;
            for (k = 0; k <= ii - 2; k++) {
              b_y->data[1 + k] = a + (1.0 + (double)k);
              b_y->data[(c_n - k) - 2] = numElmB2 - (1.0 + (double)k);
            }

            if (ii << 1 == c_n - 1) {
              b_y->data[ii] = (a + numElmB2) / 2.0;
            } else {
              b_y->data[ii] = a + (double)ii;
              b_y->data[ii + 1] = numElmB2 - (double)ii;
            }
          }
        }
      }

      c_sparse_parenReference(X.d, X.colidx, X.rowidx, b, b_y, Aj, t3_colidx,
        t3_rowidx, &idx, &boffset);
      sparse_diag(Aj, t3_colidx, t3_rowidx, idx, boffset, b_a, t5_colidx,
                  t5_rowidx, &ii);
      sum(b_a, t5_colidx, ii, one2, i, t3_colidx);
      numElmAtot = d_sparse_full(one2, i) - trVal;
      if (std::abs(numElmAtot) / trVal * 100.0 < 10.0) {
        //  trace of X is close to the desired value
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }

  emxFree_creal_T(&b_d);
  emxFree_boolean_T(&t7_d);
  emxFree_real_T(&b_y);
  emxFree_creal_T(&b_b);
  emxFree_creal_T(&y);
  emxFree_real_T(&b);
  emxFree_int32_T(&r4);
  emxFree_creal_T(&d);
  emxFree_creal_T(&D);
  emxFree_creal_T(&V);
  c_emxInitStruct_coder_internal_(&b_S);

  //  Set S=0 to project the final solution and ensure that it satisfies the linear constraints given by the adjacency matrix 
  sparse_copy(Z_colidx, Z_rowidx, aoffset, Z_n, &b_S);
  jj = Z_colidx->data[Z_colidx->size[0] - 1];
  emxFree_int32_T(&Z_rowidx);
  if (1 > Z_colidx->data[Z_colidx->size[0] - 1] - 1) {
    idx = 0;
  } else {
    idx = Z_colidx->data[Z_colidx->size[0] - 1] - 1;
  }

  i4 = qsbar->size[0];
  qsbar->size[0] = idx;
  emxEnsureCapacity_real_T(qsbar, i4);
  for (i4 = 0; i4 < idx; i4++) {
    qsbar->data[i4] = 0.0 * Z_d->data[i4];
  }

  emxFree_real_T(&Z_d);
  if (Z_colidx->data[Z_colidx->size[0] - 1] - 1 >= 1) {
    ii = Z_colidx->data[Z_colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  emxFree_int32_T(&Z_colidx);
  i4 = b_S.d->size[0];
  b_S.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(b_S.d, i4);
  for (i4 = 0; i4 <= ii; i4++) {
    b_S.d->data[i4] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    b_S.d->data[k] = qsbar->data[k];
  }

  emxFree_real_T(&qsbar);
  b_sparse_fillIn(&b_S);
  sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
  jj = X.colidx->data[X.colidx->size[0] - 1];
  if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
    ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  i4 = I0.d->size[0];
  I0.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(I0.d, i4);
  for (i4 = 0; i4 <= ii; i4++) {
    I0.d->data[i4] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    I0.d->data[k] = X.d->data[k];
  }

  b_sparse_fillIn(&I0);
  sparse_copy(b_colidx, b_rowidx, b_m, b_n, &Z0);
  jj = b_colidx->data[b_colidx->size[0] - 1];
  emxFree_int32_T(&b_rowidx);
  if (b_colidx->data[b_colidx->size[0] - 1] - 1 >= 1) {
    ii = b_colidx->data[b_colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  emxFree_int32_T(&b_colidx);
  i4 = Z0.d->size[0];
  Z0.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(Z0.d, i4);
  for (i4 = 0; i4 <= ii; i4++) {
    Z0.d->data[i4] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    Z0.d->data[k] = bi->data[k];
  }

  emxFree_real_T(&bi);
  b_sparse_fillIn(&Z0);
  sparse_minus(C_d, C_colidx, C_rowidx, b_S.d, b_S.colidx, b_S.rowidx, b_S.m,
               b_S.n, Aj, t3_colidx, t3_rowidx, &idx, &boffset);
  sparse_minus(Aj, t3_colidx, t3_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m, I0.n,
               Av, t4_colidx, t4_rowidx, &jj, &nx);
  vec(Av, t4_colidx, t4_rowidx, jj, nx, b_a, t5_colidx, t5_rowidx, &ii);
  b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, b_a, t5_colidx, t5_rowidx,
                  &expl_temp);
  i4 = one2->size[0];
  one2->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i4);
  idx = expl_temp.d->size[0];
  emxFree_int32_T(&A_rowidx);
  emxFree_int32_T(&A_colidx);
  emxFree_real_T(&A_d);
  for (i4 = 0; i4 < idx; i4++) {
    one2->data[i4] = expl_temp.d->data[i4];
  }

  i4 = one1->size[0];
  one1->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(one1, i4);
  idx = expl_temp.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    one1->data[i4] = expl_temp.colidx->data[i4];
  }

  i4 = i->size[0];
  i->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(i, i4);
  idx = expl_temp.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    i->data[i4] = expl_temp.rowidx->data[i4];
  }

  sparse_plus(one2, one1, i, Z0.d, Z0.colidx, Z0.rowidx, Z0.m, b_a, t5_colidx,
              t5_rowidx, &ii);
  sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, b_a, t5_colidx,
                  t5_rowidx, ii, &expl_temp);
  i4 = one2->size[0];
  one2->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(one2, i4);
  idx = expl_temp.d->size[0];
  emxFree_int32_T(&AAs_rowidx);
  emxFree_int32_T(&AAs_colidx);
  emxFree_real_T(&AAs_d);
  for (i4 = 0; i4 < idx; i4++) {
    one2->data[i4] = expl_temp.d->data[i4];
  }

  i4 = one1->size[0];
  one1->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(one1, i4);
  idx = expl_temp.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    one1->data[i4] = expl_temp.colidx->data[i4];
  }

  i4 = i->size[0];
  i->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(i, i4);
  idx = expl_temp.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    i->data[i4] = expl_temp.rowidx->data[i4];
  }

  sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
  jj = X.colidx->data[X.colidx->size[0] - 1];
  if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
    ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  i4 = I0.d->size[0];
  I0.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(I0.d, i4);
  for (i4 = 0; i4 <= ii; i4++) {
    I0.d->data[i4] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    I0.d->data[k] = X.d->data[k];
  }

  b_sparse_fillIn(&I0);
  b_sparse_mtimes(bv, As_colidx, As_rowidx, As_m, one2, one1, i, &expl_temp);
  i4 = b_a->size[0];
  b_a->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(b_a, i4);
  idx = expl_temp.d->size[0];
  emxFree_int32_T(&As_rowidx);
  emxFree_int32_T(&As_colidx);
  emxFree_real_T(&bv);
  for (i4 = 0; i4 < idx; i4++) {
    b_a->data[i4] = expl_temp.d->data[i4];
  }

  i4 = t5_colidx->size[0];
  t5_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t5_colidx, i4);
  idx = expl_temp.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    t5_colidx->data[i4] = expl_temp.colidx->data[i4];
  }

  i4 = t5_rowidx->size[0];
  t5_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t5_rowidx, i4);
  idx = expl_temp.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    t5_rowidx->data[i4] = expl_temp.rowidx->data[i4];
  }

  d_emxFreeStruct_coder_internal_(&expl_temp);
  sizX[0] = itrr;
  sizX[1] = itrr;
  sparse_reshape(b_a, t5_colidx, t5_rowidx, sizX, Aj, t3_colidx, t3_rowidx, &idx,
                 &boffset);
  sparse_minus(C_d, C_colidx, C_rowidx, Aj, t3_colidx, t3_rowidx, idx, boffset,
               Av, t4_colidx, t4_rowidx, &jj, &nx);
  sparse_minus(Av, t4_colidx, t4_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m, I0.n,
               one2, one1, i, &W_m, &W_n);
  sparse_transpose(one2, one1, i, W_m, W_n, Aj, t3_colidx, t3_rowidx, &idx,
                   &boffset);
  b_sparse_plus(one2, one1, i, Aj, t3_colidx, t3_rowidx, idx, boffset, Av,
                t4_colidx, t4_rowidx, &jj, &nx);
  sparse_rdivide(Av, t4_colidx, t4_rowidx, jj, nx, &Z0);
  i4 = one2->size[0];
  one2->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(one2, i4);
  idx = Z0.d->size[0];
  emxFree_int32_T(&t5_rowidx);
  emxFree_int32_T(&t5_colidx);
  emxFree_int32_T(&t4_rowidx);
  emxFree_int32_T(&t4_colidx);
  emxFree_real_T(&b_a);
  emxFree_real_T(&Av);
  emxFree_int32_T(&C_rowidx);
  emxFree_int32_T(&C_colidx);
  emxFree_real_T(&C_d);
  c_emxFreeStruct_coder_internal_(&I0);
  for (i4 = 0; i4 < idx; i4++) {
    one2->data[i4] = Z0.d->data[i4];
  }

  i4 = one1->size[0];
  one1->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(one1, i4);
  idx = Z0.colidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    one1->data[i4] = Z0.colidx->data[i4];
  }

  i4 = i->size[0];
  i->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(i, i4);
  idx = Z0.rowidx->size[0];
  for (i4 = 0; i4 < idx; i4++) {
    i->data[i4] = Z0.rowidx->data[i4];
  }

  sparse_minus(b_S.d, b_S.colidx, b_S.rowidx, one2, one1, i, Z0.m, Z0.n, Aj,
               t3_colidx, t3_rowidx, &idx, &boffset);
  b_sparse_rdivide(Aj, t3_colidx, t3_rowidx, idx, boffset, &X);
  b_sparse_full(X.d, X.colidx, X.rowidx, X.m, X.n, Su);

  //  Transform X from sparse to full representation
  numElmB2 = 2.0 * ((double)adj->size[0] - 2.0) + 1.0;
  emxFree_int32_T(&t3_rowidx);
  emxFree_int32_T(&t3_colidx);
  emxFree_int32_T(&i);
  c_emxFreeStruct_coder_internal_(&b_S);
  c_emxFreeStruct_coder_internal_(&X);
  emxFree_real_T(&Aj);
  c_emxFreeStruct_coder_internal_(&Z0);
  emxFree_real_T(&one2);
  emxFree_int32_T(&one1);
  if (numElmB2 > Su->size[0]) {
    i4 = 0;
    i5 = 0;
  } else {
    i4 = (int)numElmB2 - 1;
    i5 = Su->size[0];
  }

  numElmB2 = 2.0 * ((double)adj->size[0] - 2.0) + 1.0;
  if (numElmB2 > Su->size[1]) {
    i6 = 0;
    nx = 0;
  } else {
    i6 = (int)numElmB2 - 1;
    nx = Su->size[1];
  }

  emxInit_real_T(&c_b, 2);

  //  The componenet of X corresponding to the gain matrix
  ii = c_b->size[0] * c_b->size[1];
  idx = i5 - i4;
  c_b->size[0] = idx;
  jj = nx - i6;
  c_b->size[1] = jj;
  emxEnsureCapacity_real_T(c_b, ii);
  for (i5 = 0; i5 < jj; i5++) {
    for (nx = 0; nx < idx; nx++) {
      c_b->data[nx + c_b->size[0] * i5] = -Su->data[(i4 + nx) + Su->size[0] *
        (i6 + i5)];
    }
  }

  if ((loop_ub == 1) || (c_b->size[0] == 1)) {
    i3 = Su->size[0] * Su->size[1];
    Su->size[0] = Q->size[0];
    Su->size[1] = c_b->size[1];
    emxEnsureCapacity_real_T(Su, i3);
    idx = Q->size[0];
    for (i3 = 0; i3 < idx; i3++) {
      loop_ub = c_b->size[1];
      for (i4 = 0; i4 < loop_ub; i4++) {
        Su->data[i3 + Su->size[0] * i4] = 0.0;
        jj = Q->size[1];
        for (i5 = 0; i5 < jj; i5++) {
          Su->data[i3 + Su->size[0] * i4] += Q->data[i3 + Q->size[0] * i5] *
            c_b->data[i5 + c_b->size[0] * i4];
        }
      }
    }
  } else {
    i4 = U->size[0];
    jj = loop_ub - 1;
    n = c_b->size[1];
    i5 = U->size[0];
    i6 = Su->size[0] * Su->size[1];
    Su->size[0] = i5;
    Su->size[1] = c_b->size[1];
    emxEnsureCapacity_real_T(Su, i6);
    for (j = 0; j < n; j++) {
      idx = j * i4;
      boffset = j * (jj + 1);
      for (b_i = 0; b_i < i4; b_i++) {
        Su->data[idx + b_i] = 0.0;
      }

      for (k = 0; k <= jj; k++) {
        aoffset = k * i4;
        numElmAtot = c_b->data[boffset + k];
        for (b_i = 0; b_i < i4; b_i++) {
          i5 = U->size[0];
          i6 = aoffset + b_i;
          nx = idx + b_i;
          Su->data[nx] += numElmAtot * U->data[i6 % i5 + U->size[0] * (i3 +
            div_nzp_s32_floor(i6, i5))];
        }
      }
    }
  }

  emxFree_real_T(&c_b);
  emxFree_real_T(&U);
  emxFree_real_T(&Q);
  if ((Su->size[1] == 1) || (Qt->size[0] == 1)) {
    i3 = Aopt->size[0] * Aopt->size[1];
    Aopt->size[0] = Su->size[0];
    Aopt->size[1] = Qt->size[1];
    emxEnsureCapacity_real_T(Aopt, i3);
    idx = Su->size[0];
    for (i3 = 0; i3 < idx; i3++) {
      loop_ub = Qt->size[1];
      for (i4 = 0; i4 < loop_ub; i4++) {
        Aopt->data[i3 + Aopt->size[0] * i4] = 0.0;
        jj = Su->size[1];
        for (i5 = 0; i5 < jj; i5++) {
          Aopt->data[i3 + Aopt->size[0] * i4] += Su->data[i3 + Su->size[0] * i5]
            * Qt->data[i5 + Qt->size[0] * i4];
        }
      }
    }
  } else {
    m = Su->size[0];
    jj = Su->size[1];
    n = Qt->size[1];
    i3 = Aopt->size[0] * Aopt->size[1];
    Aopt->size[0] = Su->size[0];
    Aopt->size[1] = Qt->size[1];
    emxEnsureCapacity_real_T(Aopt, i3);
    for (j = 0; j < n; j++) {
      idx = j * m;
      boffset = j * jj;
      for (b_i = 0; b_i < m; b_i++) {
        Aopt->data[idx + b_i] = 0.0;
      }

      for (k = 0; k < jj; k++) {
        aoffset = k * m;
        numElmAtot = Qt->data[boffset + k];
        for (b_i = 0; b_i < m; b_i++) {
          i3 = idx + b_i;
          Aopt->data[i3] += numElmAtot * Su->data[aoffset + b_i];
        }
      }
    }
  }

  emxFree_real_T(&Su);
  emxFree_real_T(&Qt);

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
