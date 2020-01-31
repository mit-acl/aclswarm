//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign3D.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
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
#include "ADMMGainDesign2D.h"
#include "ADMMGainDesign3D_rtwutil.h"

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
  int idx;
  int i0;
  emxArray_real_T *Axy;
  int i1;
  emxArray_real_T *c_Qs;
  int n;
  int m;
  int ii;
  emxArray_real_T *U;
  emxArray_real_T *unusedU0;
  double unusedU1[4];
  emxArray_real_T *Q;
  int i2;
  int loop_ub;
  emxArray_real_T *Qt;
  coder_internal_sparse I0;
  int nx;
  coder_internal_sparse Z0;
  emxArray_real_T *Z_d;
  emxArray_int32_T *Z_colidx;
  emxArray_int32_T *Z_rowidx;
  emxArray_real_T *C_d;
  emxArray_real_T *Aj;
  emxArray_real_T *Av;
  emxArray_int32_T *t8_colidx;
  emxArray_int32_T *t8_rowidx;
  emxArray_int32_T *t9_colidx;
  emxArray_int32_T *t9_rowidx;
  int aoffset;
  int Z_n;
  int boffset;
  int jj;
  emxArray_int32_T *C_colidx;
  emxArray_int32_T *C_rowidx;
  emxArray_boolean_T *S;
  emxArray_real_T *Su;
  emxArray_boolean_T *r0;
  emxArray_boolean_T *t12_d;
  emxArray_int32_T *i;
  emxArray_int32_T *j;
  emxArray_real_T *tmpd;
  bool exitg1;
  emxArray_real_T *Ai;
  bool guard1 = false;
  double numElmAtot;
  emxArray_real_T *bi;
  emxArray_real_T *bv;
  double itrr;
  unsigned int itra;
  unsigned int itrb;
  int b_i;
  int b_j;
  int c_i;
  emxArray_real_T *b_Qt;
  emxArray_real_T *b_Q;
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
  double sizX;
  int As_m;
  emxArray_int32_T *AAs_colidx;
  emxArray_int32_T *AAs_rowidx;
  coder_internal_sparse X;
  int AAs_m;
  int AAs_n;
  emxArray_creal_T *dd;
  emxArray_creal_T *V;
  emxArray_creal_T *D;
  emxArray_int32_T *r1;
  emxArray_creal_T *y;
  emxArray_creal_T *b;
  emxArray_real_T *b_y;
  emxArray_real_T *c_y;
  emxArray_int32_T *t10_colidx;
  emxArray_int32_T *t10_rowidx;
  coder_internal_sparse_1 expl_temp;
  emxArray_creal_T *b_dd;
  coder_internal_sparse b_S;
  int k;
  double b_sizX[2];
  int W_m;
  int W_n;
  int c_n;
  double cdiff;
  double temp_im;
  emxArray_real_T *d_y;
  emxArray_real_T *b_b;
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
  idx = Qs->size[1];
  i0 = b_Qs->size[0] * b_Qs->size[1];
  b_Qs->size[0] = 2;
  b_Qs->size[1] = idx;
  emxEnsureCapacity_real_T(b_Qs, i0);
  for (i0 = 0; i0 < idx; i0++) {
    i1 = i0 << 1;
    b_Qs->data[i1] = Qs->data[3 * i0];
    b_Qs->data[1 + i1] = Qs->data[1 + 3 * i0];
  }

  emxInit_real_T(&Axy, 2);
  emxInit_real_T(&c_Qs, 2);
  ADMMGainDesign2D(b_Qs, adj, Axy);

  //  Gain design for the altitude
  //  Number of agents
  n = adj->size[0];
  m = adj->size[0] - 2;

  //  Reduced dimension
  //  Vector of ones
  //  Vector of z-coordinates
  //  ambient dimension of problem
  //  Kernel of gain matrix
  //  Get orthogonal complement of N
  ii = adj->size[0];
  idx = Qs->size[1];
  i0 = c_Qs->size[0] * c_Qs->size[1];
  c_Qs->size[0] = idx;
  c_Qs->size[1] = 2;
  emxEnsureCapacity_real_T(c_Qs, i0);
  emxFree_real_T(&b_Qs);
  for (i0 = 0; i0 < idx; i0++) {
    c_Qs->data[i0] = Qs->data[2 + 3 * i0];
  }

  for (i0 = 0; i0 < ii; i0++) {
    c_Qs->data[i0 + c_Qs->size[0]] = 1.0;
  }

  emxInit_real_T(&U, 2);
  emxInit_real_T(&unusedU0, 2);
  c_svd(c_Qs, U, unusedU0, unusedU1);
  emxFree_real_T(&c_Qs);
  emxFree_real_T(&unusedU0);
  if (3 > adj->size[0]) {
    i0 = 0;
    i1 = 0;
  } else {
    i0 = 2;
    i1 = adj->size[0];
  }

  emxInit_real_T(&Q, 2);
  idx = U->size[0];
  i2 = Q->size[0] * Q->size[1];
  Q->size[0] = idx;
  loop_ub = i1 - i0;
  Q->size[1] = loop_ub;
  emxEnsureCapacity_real_T(Q, i2);
  for (i1 = 0; i1 < loop_ub; i1++) {
    for (i2 = 0; i2 < idx; i2++) {
      Q->data[i2 + Q->size[0] * i1] = U->data[i2 + U->size[0] * (i0 + i1)];
    }
  }

  emxInit_real_T(&Qt, 2);
  i1 = Qt->size[0] * Qt->size[1];
  Qt->size[0] = Q->size[1];
  Qt->size[1] = Q->size[0];
  emxEnsureCapacity_real_T(Qt, i1);
  idx = Q->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    nx = Q->size[1];
    for (i2 = 0; i2 < nx; i2++) {
      Qt->data[i2 + Qt->size[0] * i1] = Q->data[i1 + Q->size[0] * i2];
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
  emxInit_int32_T(&t8_colidx, 1);
  emxInit_int32_T(&t8_rowidx, 1);
  emxInit_int32_T(&t9_colidx, 1);
  emxInit_int32_T(&t9_rowidx, 1);

  //  Preallocate variables
  speye((double)adj->size[0] - 2.0, &I0);
  sparse((double)adj->size[0] - 2.0, (double)adj->size[0] - 2.0, Z0.d, Z0.colidx,
         Z0.rowidx, &Z0.m, &Z0.n);
  sparse(2.0 * ((double)adj->size[0] - 2.0), 2.0 * ((double)adj->size[0] - 2.0),
         Z_d, Z_colidx, Z_rowidx, &aoffset, &Z_n);

  //  Cost function's coefficient matrix: f = <C,X>
  sparse_horzcat(I0.d, I0.colidx, I0.rowidx, I0.m, I0.n, Z0.d, Z0.colidx,
                 Z0.rowidx, Z0.m, Z0.n, Aj, t8_colidx, t8_rowidx, &idx, &boffset);
  sparse_horzcat(Z0.d, Z0.colidx, Z0.rowidx, Z0.m, Z0.n, Z0.d, Z0.colidx,
                 Z0.rowidx, Z0.m, Z0.n, Av, t9_colidx, t9_rowidx, &ii, &jj);
  sparse_vertcat(Aj, t8_colidx, t8_rowidx, idx, boffset, Av, t9_colidx,
                 t9_rowidx, ii, jj, &Z0);
  i1 = C_d->size[0];
  C_d->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(C_d, i1);
  idx = Z0.d->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    C_d->data[i1] = Z0.d->data[i1];
  }

  emxInit_int32_T(&C_colidx, 1);
  i1 = C_colidx->size[0];
  C_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(C_colidx, i1);
  idx = Z0.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    C_colidx->data[i1] = Z0.colidx->data[i1];
  }

  emxInit_int32_T(&C_rowidx, 1);
  i1 = C_rowidx->size[0];
  C_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(C_rowidx, i1);
  idx = Z0.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    C_rowidx->data[i1] = Z0.rowidx->data[i1];
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
  idx = adj->size[0] * adj->size[1];
  for (i1 = 0; i1 < idx; i1++) {
    S->data[i1] = !(adj->data[i1] != 0.0);
  }

  emxInit_real_T(&Su, 2);
  emxInit_boolean_T(&r0, 2);
  emxInit_boolean_T(&t12_d, 1);
  diag(S, t12_d);
  b_diag(t12_d, r0);
  i1 = Su->size[0] * Su->size[1];
  Su->size[0] = S->size[0];
  Su->size[1] = S->size[1];
  emxEnsureCapacity_real_T(Su, i1);
  idx = S->size[0] * S->size[1];
  for (i1 = 0; i1 < idx; i1++) {
    Su->data[i1] = (double)S->data[i1] - (double)r0->data[i1];
  }

  emxFree_boolean_T(&r0);
  emxFree_boolean_T(&S);
  triu(Su);

  //  Upper triangular part
  nx = Su->size[0] * Su->size[1];
  emxInit_int32_T(&i, 1);
  emxInit_int32_T(&j, 1);
  if (nx == 0) {
    i->size[0] = 0;
    j->size[0] = 0;
  } else {
    idx = 0;
    i1 = i->size[0];
    i->size[0] = nx;
    emxEnsureCapacity_int32_T(i, i1);
    i1 = j->size[0];
    j->size[0] = nx;
    emxEnsureCapacity_int32_T(j, i1);
    ii = 1;
    jj = 1;
    exitg1 = false;
    while ((!exitg1) && (jj <= Su->size[1])) {
      guard1 = false;
      if (Su->data[(ii + Su->size[0] * (jj - 1)) - 1] != 0.0) {
        idx++;
        i->data[idx - 1] = ii;
        j->data[idx - 1] = jj;
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
        j->size[0] = 0;
      }
    } else if (1 > idx) {
      i->size[0] = 0;
      j->size[0] = 0;
    } else {
      i1 = i->size[0];
      i->size[0] = idx;
      emxEnsureCapacity_int32_T(i, i1);
      i1 = j->size[0];
      j->size[0] = idx;
      emxEnsureCapacity_int32_T(j, i1);
    }
  }

  emxInit_real_T(&tmpd, 1);
  i1 = tmpd->size[0];
  tmpd->size[0] = i->size[0];
  emxEnsureCapacity_real_T(tmpd, i1);
  idx = i->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    tmpd->data[i1] = i->data[i1];
  }

  emxInit_real_T(&Ai, 1);

  //  Find location of nonzero entries
  //  Number of constraints
  //  Number of elements in block [X]_22
  //  Number of elements for trace of [X]_22
  //  Number of elements for symmetry
  //  Number of elements for pinning down the b-vector
  //  Total number of elements
  numElmAtot = (((((((double)adj->size[0] - 2.0) - 1.0) * 2.0 + ((double)
    adj->size[0] - 2.0) * (((double)adj->size[0] - 2.0) - 1.0) / 2.0) +
                  (((double)adj->size[0] - 2.0) + ((double)adj->size[0] - 2.0) *
                   (((double)adj->size[0] - 2.0) - 1.0))) + (double)tmpd->size[0]
                 * (((double)adj->size[0] - 2.0) * ((double)adj->size[0] - 2.0)))
                + ((double)adj->size[0] - 2.0)) + 2.0 * ((double)adj->size[0] -
    2.0) * (2.0 * ((double)adj->size[0] - 2.0) - 1.0);

  // %%%%%%%%%%%%%%%%%%%%% Preallocate sparse matrices A & b
  //
  //  Constraint: A * vec(X) = b
  //
  //  Indices of A: entry [Ai(k), Aj(k)] takes value of Av(k)
  //  Each row of matrix A will represent a constraint
  i1 = Ai->size[0];
  idx = (int)numElmAtot;
  Ai->size[0] = idx;
  emxEnsureCapacity_real_T(Ai, i1);
  for (i1 = 0; i1 < idx; i1++) {
    Ai->data[i1] = 0.0;
  }

  i1 = Aj->size[0];
  Aj->size[0] = idx;
  emxEnsureCapacity_real_T(Aj, i1);
  for (i1 = 0; i1 < idx; i1++) {
    Aj->data[i1] = 0.0;
  }

  i1 = Av->size[0];
  Av->size[0] = idx;
  emxEnsureCapacity_real_T(Av, i1);
  for (i1 = 0; i1 < idx; i1++) {
    Av->data[i1] = 0.0;
  }

  emxInit_real_T(&bi, 1);
  i1 = bi->size[0];
  bi->size[0] = adj->size[0];
  emxEnsureCapacity_real_T(bi, i1);
  idx = adj->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    bi->data[i1] = 0.0;
  }

  emxInit_real_T(&bv, 1);
  i1 = bv->size[0];
  bv->size[0] = adj->size[0];
  emxEnsureCapacity_real_T(bv, i1);
  idx = adj->size[0];
  for (i1 = 0; i1 < idx; i1++) {
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
  i1 = adj->size[0];
  for (b_i = 0; b_i <= i1 - 4; b_i++) {
    itrr++;
    itra++;
    i2 = (int)itra - 1;
    Ai->data[i2] = itrr;
    Aj->data[i2] = 1.0;
    Av->data[i2] = 1.0;
    itra++;
    i2 = (int)itra - 1;
    Ai->data[i2] = itrr;
    Aj->data[i2] = ((2.0 + (double)b_i) - 1.0) * (2.0 * ((double)n - 2.0)) +
      (2.0 + (double)b_i);
    Av->data[i2] = -1.0;
  }

  //  Off-diagonal entries should be zero
  i1 = adj->size[0];
  for (b_i = 0; b_i <= i1 - 4; b_i++) {
    i2 = n - b_i;
    for (b_j = 0; b_j <= i2 - 4; b_j++) {
      itrr++;
      itra++;
      c_i = (int)itra - 1;
      Ai->data[c_i] = itrr;
      Aj->data[c_i] = ((1.0 + (double)b_i) - 1.0) * (2.0 * ((double)n - 2.0)) +
        (double)(((unsigned int)b_i + b_j) + 2U);
      Av->data[c_i] = 1.0;
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_12
  //  Diagonal entries should be 1
  i1 = adj->size[0];
  for (b_i = 0; b_i <= i1 - 3; b_i++) {
    itrr++;
    itra++;
    i2 = (int)itra - 1;
    Ai->data[i2] = itrr;
    Aj->data[i2] = ((1.0 + (double)b_i) - 1.0) * (2.0 * ((double)n - 2.0)) +
      ((1.0 + (double)b_i) + (double)m);
    Av->data[i2] = 1.0;
    itrb++;
    i2 = (int)itrb - 1;
    bi->data[i2] = itrr;
    bv->data[i2] = 1.0;
  }

  //  Other entries should be 0
  i1 = adj->size[0];
  for (b_i = 0; b_i <= i1 - 3; b_i++) {
    for (b_j = 0; b_j <= n - 3; b_j++) {
      if (1 + b_i != 1 + b_j) {
        itrr++;
        itra++;
        i2 = (int)itra - 1;
        Ai->data[i2] = itrr;
        Aj->data[i2] = ((1.0 + (double)b_i) - 1.0) * (2.0 * ((double)n - 2.0)) +
          ((1.0 + (double)b_j) + (double)m);
        Av->data[i2] = 1.0;
      }
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_22
  //  Zero constraints due to the adjacency matrix
  i1 = tmpd->size[0];
  emxInit_real_T(&b_Qt, 1);
  emxInit_real_T(&b_Q, 2);
  for (b_i = 0; b_i < i1; b_i++) {
    //  Term corresponding to row ii and column jj
    idx = Qt->size[0];
    b_j = j->data[b_i];
    i2 = b_Qt->size[0];
    b_Qt->size[0] = idx;
    emxEnsureCapacity_real_T(b_Qt, i2);
    for (i2 = 0; i2 < idx; i2++) {
      b_Qt->data[i2] = Qt->data[i2 + Qt->size[0] * (b_j - 1)];
    }

    ii = (int)((tmpd->data[b_i] - 1.0) + 1.0);
    i2 = b_Q->size[0] * b_Q->size[1];
    b_Q->size[0] = 1;
    b_Q->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b_Q, i2);
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_Q->data[i2] = Q->data[(ii + Q->size[0] * i2) - 1];
    }

    i2 = Su->size[0] * Su->size[1];
    Su->size[0] = b_Qt->size[0];
    Su->size[1] = b_Q->size[1];
    emxEnsureCapacity_real_T(Su, i2);
    idx = b_Qt->size[0];
    for (i2 = 0; i2 < idx; i2++) {
      nx = b_Q->size[1];
      for (c_i = 0; c_i < nx; c_i++) {
        Su->data[i2 + Su->size[0] * c_i] = b_Qt->data[i2] * b_Q->data[c_i];
      }
    }

    itrr++;
    for (ii = 0; ii <= n - 3; ii++) {
      for (jj = 0; jj <= n - 3; jj++) {
        itra++;
        i2 = (int)itra - 1;
        Ai->data[i2] = itrr;
        Aj->data[i2] = (((double)m + (1.0 + (double)ii)) - 1.0) * (2.0 *
          ((double)n - 2.0)) + ((double)m + (1.0 + (double)jj));
        Av->data[i2] = Su->data[ii + Su->size[0] * jj];
      }
    }
  }

  emxFree_real_T(&b_Q);

  //  Trace of gain matrix must be the specified value in 'trVal'
  itrr++;
  i1 = adj->size[0];
  for (b_i = 0; b_i <= i1 - 3; b_i++) {
    itra++;
    numElmAtot = ((double)n - 2.0) + (1.0 + (double)b_i);
    i2 = (int)itra - 1;
    Ai->data[i2] = itrr;
    Aj->data[i2] = (numElmAtot - 1.0) * (2.0 * ((double)n - 2.0)) + numElmAtot;
    Av->data[i2] = 1.0;
  }

  itrb++;
  bi->data[(int)itrb - 1] = itrr;
  bv->data[(int)itrb - 1] = (double)adj->size[0] - 2.0;

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%% Symmetry
  i1 = (adj->size[0] - 2) << 1;
  for (b_i = 0; b_i <= i1 - 2; b_i++) {
    i2 = (m << 1) - b_i;
    for (b_j = 0; b_j <= i2 - 2; b_j++) {
      numElmAtot = ((1.0 + (double)b_i) + 1.0) + (double)b_j;

      //  Symmetric entries should be equal
      itrr++;
      itra++;
      c_i = (int)itra - 1;
      Ai->data[c_i] = itrr;
      Aj->data[c_i] = ((1.0 + (double)b_i) - 1.0) * (2.0 * ((double)n - 2.0)) +
        numElmAtot;
      Av->data[c_i] = 1.0;
      itra++;
      c_i = (int)itra - 1;
      Ai->data[c_i] = itrr;
      Aj->data[c_i] = (numElmAtot - 1.0) * (2.0 * ((double)n - 2.0)) + (1.0 +
        (double)b_i);
      Av->data[c_i] = -1.0;
    }
  }

  emxInit_real_T(&A_d, 1);

  //  Last element set to fix the size of b
  itrb++;
  i1 = (int)itrb - 1;
  bi->data[i1] = itrr;
  bv->data[i1] = 0.0;

  //  % Remove any additional entries
  //  Ai(itra+1:end) = [];
  //  Aj(itra+1:end) = [];
  //  Av(itra+1:end) = [];
  //
  //  bi(itrb+1:end) = [];
  //  bv(itrb+1:end) = [];
  //  Make sparse matrices
  b_sparse(Ai, Aj, Av, &Z0);
  i1 = A_d->size[0];
  A_d->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(A_d, i1);
  idx = Z0.d->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    A_d->data[i1] = Z0.d->data[i1];
  }

  emxInit_int32_T(&A_colidx, 1);
  i1 = A_colidx->size[0];
  A_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(A_colidx, i1);
  idx = Z0.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    A_colidx->data[i1] = Z0.colidx->data[i1];
  }

  emxInit_int32_T(&A_rowidx, 1);
  i1 = A_rowidx->size[0];
  A_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(A_rowidx, i1);
  idx = Z0.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    A_rowidx->data[i1] = Z0.rowidx->data[i1];
  }

  A_m = Z0.m;
  jj = Z0.n;
  i1 = Aj->size[0];
  Aj->size[0] = bi->size[0];
  emxEnsureCapacity_real_T(Aj, i1);
  idx = bi->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    Aj->data[i1] = 1.0;
  }

  b_sparse(bi, Aj, bv, &Z0);
  i1 = bi->size[0];
  bi->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(bi, i1);
  idx = Z0.d->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    bi->data[i1] = Z0.d->data[i1];
  }

  emxInit_int32_T(&b_colidx, 1);
  i1 = b_colidx->size[0];
  b_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(b_colidx, i1);
  idx = Z0.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    b_colidx->data[i1] = Z0.colidx->data[i1];
  }

  emxInit_int32_T(&b_rowidx, 1);
  i1 = b_rowidx->size[0];
  b_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(b_rowidx, i1);
  idx = Z0.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    b_rowidx->data[i1] = Z0.rowidx->data[i1];
  }

  emxInit_int32_T(&As_colidx, 1);
  emxInit_int32_T(&As_rowidx, 1);
  emxInit_real_T(&AAs_d, 1);
  b_m = Z0.m;
  b_n = Z0.n;

  //  Size of optimization variable
  sizX = 2.0 * ((double)adj->size[0] - 2.0);

  //  ADMM algorithm--full eigendecomposition
  sparse_transpose(A_d, A_colidx, A_rowidx, A_m, jj, bv, As_colidx, As_rowidx,
                   &As_m, &ii);

  //  Dual operator
  sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, bv, As_colidx, As_rowidx, ii, &Z0);
  i1 = AAs_d->size[0];
  AAs_d->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(AAs_d, i1);
  idx = Z0.d->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    AAs_d->data[i1] = Z0.d->data[i1];
  }

  emxInit_int32_T(&AAs_colidx, 1);
  i1 = AAs_colidx->size[0];
  AAs_colidx->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(AAs_colidx, i1);
  idx = Z0.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    AAs_colidx->data[i1] = Z0.colidx->data[i1];
  }

  emxInit_int32_T(&AAs_rowidx, 1);
  i1 = AAs_rowidx->size[0];
  AAs_rowidx->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(AAs_rowidx, i1);
  idx = Z0.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    AAs_rowidx->data[i1] = Z0.rowidx->data[i1];
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
                 I0.rowidx, I0.m, I0.n, Aj, t8_colidx, t8_rowidx, &idx, &boffset);
  sparse_horzcat(I0.d, I0.colidx, I0.rowidx, I0.m, I0.n, I0.d, I0.colidx,
                 I0.rowidx, I0.m, I0.n, Av, t9_colidx, t9_rowidx, &ii, &jj);
  sparse_vertcat(Aj, t8_colidx, t8_rowidx, idx, boffset, Av, t9_colidx,
                 t9_rowidx, ii, jj, &X);
  b_i = 0;
  emxInit_creal_T(&dd, 1);
  emxInit_creal_T(&V, 2);
  emxInit_creal_T(&D, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_creal_T(&y, 2);
  emxInit_creal_T(&b, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_real_T(&c_y, 2);
  emxInit_int32_T(&t10_colidx, 1);
  emxInit_int32_T(&t10_rowidx, 1);
  d_emxInitStruct_coder_internal_(&expl_temp);
  emxInit_creal_T(&b_dd, 1);
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

    i1 = I0.d->size[0];
    I0.d->size[0] = ii + 1;
    emxEnsureCapacity_real_T(I0.d, i1);
    for (i1 = 0; i1 <= ii; i1++) {
      I0.d->data[i1] = 0.0;
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

    i1 = Z0.d->size[0];
    Z0.d->size[0] = ii + 1;
    emxEnsureCapacity_real_T(Z0.d, i1);
    for (i1 = 0; i1 <= ii; i1++) {
      Z0.d->data[i1] = 0.0;
    }

    for (k = 0; k <= jj - 2; k++) {
      Z0.d->data[k] = bi->data[k];
    }

    b_sparse_fillIn(&Z0);
    sparse_minus(C_d, C_colidx, C_rowidx, Z_d, Z_colidx, Z_rowidx, aoffset, Z_n,
                 Aj, t8_colidx, t8_rowidx, &idx, &boffset);
    sparse_minus(Aj, t8_colidx, t8_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m,
                 I0.n, Av, t9_colidx, t9_rowidx, &ii, &jj);
    vec(Av, t9_colidx, t9_rowidx, ii, jj, b_Qt, t10_colidx, t10_rowidx, &nx);
    b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, b_Qt, t10_colidx, t10_rowidx,
                    &expl_temp);
    i1 = Ai->size[0];
    Ai->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    idx = expl_temp.d->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      Ai->data[i1] = expl_temp.d->data[i1];
    }

    i1 = i->size[0];
    i->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(i, i1);
    idx = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      i->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = j->size[0];
    j->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(j, i1);
    idx = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      j->data[i1] = expl_temp.rowidx->data[i1];
    }

    sparse_plus(Ai, i, j, Z0.d, Z0.colidx, Z0.rowidx, Z0.m, b_Qt, t10_colidx,
                t10_rowidx, &nx);
    sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, b_Qt,
                    t10_colidx, t10_rowidx, nx, &expl_temp);
    i1 = Ai->size[0];
    Ai->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    idx = expl_temp.d->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      Ai->data[i1] = expl_temp.d->data[i1];
    }

    i1 = i->size[0];
    i->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(i, i1);
    idx = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      i->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = j->size[0];
    j->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(j, i1);
    idx = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      j->data[i1] = expl_temp.rowidx->data[i1];
    }

    // %%%%%% Update for S
    sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
    jj = X.colidx->data[X.colidx->size[0] - 1];
    if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
      ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
    } else {
      ii = 0;
    }

    i1 = I0.d->size[0];
    I0.d->size[0] = ii + 1;
    emxEnsureCapacity_real_T(I0.d, i1);
    for (i1 = 0; i1 <= ii; i1++) {
      I0.d->data[i1] = 0.0;
    }

    for (k = 0; k <= jj - 2; k++) {
      I0.d->data[k] = X.d->data[k];
    }

    b_sparse_fillIn(&I0);
    b_sparse_mtimes(bv, As_colidx, As_rowidx, As_m, Ai, i, j, &expl_temp);
    i1 = b_Qt->size[0];
    b_Qt->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(b_Qt, i1);
    idx = expl_temp.d->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      b_Qt->data[i1] = expl_temp.d->data[i1];
    }

    i1 = t10_colidx->size[0];
    t10_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t10_colidx, i1);
    idx = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      t10_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    i1 = t10_rowidx->size[0];
    t10_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(t10_rowidx, i1);
    idx = expl_temp.rowidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      t10_rowidx->data[i1] = expl_temp.rowidx->data[i1];
    }

    b_sizX[0] = sizX;
    b_sizX[1] = sizX;
    sparse_reshape(b_Qt, t10_colidx, t10_rowidx, b_sizX, Aj, t8_colidx,
                   t8_rowidx, &idx, &boffset);
    sparse_minus(C_d, C_colidx, C_rowidx, Aj, t8_colidx, t8_rowidx, idx, boffset,
                 Av, t9_colidx, t9_rowidx, &ii, &jj);
    sparse_minus(Av, t9_colidx, t9_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m,
                 I0.n, Ai, i, j, &W_m, &W_n);
    sparse_transpose(Ai, i, j, W_m, W_n, Aj, t8_colidx, t8_rowidx, &idx,
                     &boffset);
    b_sparse_plus(Ai, i, j, Aj, t8_colidx, t8_rowidx, idx, boffset, Av,
                  t9_colidx, t9_rowidx, &ii, &jj);
    sparse_rdivide(Av, t9_colidx, t9_rowidx, ii, jj, &Z0);
    i1 = Ai->size[0];
    Ai->size[0] = Z0.d->size[0];
    emxEnsureCapacity_real_T(Ai, i1);
    idx = Z0.d->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      Ai->data[i1] = Z0.d->data[i1];
    }

    i1 = i->size[0];
    i->size[0] = Z0.colidx->size[0];
    emxEnsureCapacity_int32_T(i, i1);
    idx = Z0.colidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      i->data[i1] = Z0.colidx->data[i1];
    }

    i1 = j->size[0];
    j->size[0] = Z0.rowidx->size[0];
    emxEnsureCapacity_int32_T(j, i1);
    idx = Z0.rowidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      j->data[i1] = Z0.rowidx->data[i1];
    }

    W_m = Z0.m;
    W_n = Z0.n;
    b_sparse_full(Ai, i, j, Z0.m, Z0.n, Su);
    eig(Su, V, D);
    c_diag(D, dd);
    jj = dd->size[0] - 1;
    ii = 0;
    for (c_i = 0; c_i <= jj; c_i++) {
      if (dd->data[c_i].re > 1.0E-5) {
        ii++;
      }
    }

    i1 = r1->size[0];
    r1->size[0] = ii;
    emxEnsureCapacity_int32_T(r1, i1);
    ii = 0;
    for (c_i = 0; c_i <= jj; c_i++) {
      if (dd->data[c_i].re > 1.0E-5) {
        r1->data[ii] = c_i + 1;
        ii++;
      }
    }

    idx = V->size[0];
    i1 = D->size[0] * D->size[1];
    D->size[0] = idx;
    D->size[1] = r1->size[0];
    emxEnsureCapacity_creal_T(D, i1);
    nx = r1->size[0];
    for (i1 = 0; i1 < nx; i1++) {
      for (i2 = 0; i2 < idx; i2++) {
        D->data[i2 + D->size[0] * i1] = V->data[i2 + V->size[0] * (r1->data[i1]
          - 1)];
      }
    }

    i1 = b_dd->size[0];
    b_dd->size[0] = r1->size[0];
    emxEnsureCapacity_creal_T(b_dd, i1);
    idx = r1->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      b_dd->data[i1] = dd->data[r1->data[i1] - 1];
    }

    d_diag(b_dd, b);
    if ((r1->size[0] == 1) || (b->size[0] == 1)) {
      i1 = y->size[0] * y->size[1];
      y->size[0] = D->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity_creal_T(y, i1);
      idx = D->size[0];
      for (i1 = 0; i1 < idx; i1++) {
        nx = b->size[1];
        for (i2 = 0; i2 < nx; i2++) {
          y->data[i1 + y->size[0] * i2].re = 0.0;
          y->data[i1 + y->size[0] * i2].im = 0.0;
          ii = D->size[1];
          for (c_i = 0; c_i < ii; c_i++) {
            numElmAtot = D->data[i1 + D->size[0] * c_i].re * b->data[c_i +
              b->size[0] * i2].re - D->data[i1 + D->size[0] * c_i].im * b->
              data[c_i + b->size[0] * i2].im;
            itrr = D->data[i1 + D->size[0] * c_i].re * b->data[c_i + b->size[0] *
              i2].im + D->data[i1 + D->size[0] * c_i].im * b->data[c_i + b->
              size[0] * i2].re;
            y->data[i1 + y->size[0] * i2].re += numElmAtot;
            y->data[i1 + y->size[0] * i2].im += itrr;
          }
        }
      }
    } else {
      i1 = V->size[0];
      nx = r1->size[0];
      c_n = b->size[1];
      i2 = V->size[0];
      c_i = y->size[0] * y->size[1];
      y->size[0] = i2;
      y->size[1] = b->size[1];
      emxEnsureCapacity_creal_T(y, c_i);
      for (b_j = 0; b_j < c_n; b_j++) {
        idx = b_j * i1;
        boffset = b_j * nx;
        for (c_i = 0; c_i < i1; c_i++) {
          i2 = idx + c_i;
          y->data[i2].re = 0.0;
          y->data[i2].im = 0.0;
        }

        for (k = 0; k < nx; k++) {
          aoffset = k * i1;
          ii = boffset + k;
          numElmAtot = b->data[ii].re;
          itrr = b->data[ii].im;
          for (c_i = 0; c_i < i1; c_i++) {
            ii = aoffset + c_i;
            cdiff = numElmAtot * D->data[ii].re - itrr * D->data[ii].im;
            temp_im = numElmAtot * D->data[ii].im + itrr * D->data[ii].re;
            i2 = idx + c_i;
            y->data[i2].re += cdiff;
            y->data[i2].im += temp_im;
          }
        }
      }
    }

    idx = V->size[0];
    i1 = b->size[0] * b->size[1];
    b->size[0] = r1->size[0];
    b->size[1] = idx;
    emxEnsureCapacity_creal_T(b, i1);
    for (i1 = 0; i1 < idx; i1++) {
      nx = r1->size[0];
      for (i2 = 0; i2 < nx; i2++) {
        b->data[i2 + b->size[0] * i1] = V->data[i1 + V->size[0] * (r1->data[i2]
          - 1)];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i1 = D->size[0] * D->size[1];
      D->size[0] = y->size[0];
      D->size[1] = b->size[1];
      emxEnsureCapacity_creal_T(D, i1);
      idx = y->size[0];
      for (i1 = 0; i1 < idx; i1++) {
        nx = b->size[1];
        for (i2 = 0; i2 < nx; i2++) {
          D->data[i1 + D->size[0] * i2].re = 0.0;
          D->data[i1 + D->size[0] * i2].im = 0.0;
          ii = y->size[1];
          for (c_i = 0; c_i < ii; c_i++) {
            numElmAtot = y->data[i1 + y->size[0] * c_i].re * b->data[c_i +
              b->size[0] * i2].re - y->data[i1 + y->size[0] * c_i].im * b->
              data[c_i + b->size[0] * i2].im;
            itrr = y->data[i1 + y->size[0] * c_i].re * b->data[c_i + b->size[0] *
              i2].im + y->data[i1 + y->size[0] * c_i].im * b->data[c_i + b->
              size[0] * i2].re;
            D->data[i1 + D->size[0] * i2].re += numElmAtot;
            D->data[i1 + D->size[0] * i2].im += itrr;
          }
        }
      }
    } else {
      jj = y->size[0];
      nx = y->size[1];
      c_n = b->size[1];
      i1 = D->size[0] * D->size[1];
      D->size[0] = y->size[0];
      D->size[1] = b->size[1];
      emxEnsureCapacity_creal_T(D, i1);
      for (b_j = 0; b_j < c_n; b_j++) {
        idx = b_j * jj;
        boffset = b_j * nx;
        for (c_i = 0; c_i < jj; c_i++) {
          i1 = idx + c_i;
          D->data[i1].re = 0.0;
          D->data[i1].im = 0.0;
        }

        for (k = 0; k < nx; k++) {
          aoffset = k * jj;
          ii = boffset + k;
          numElmAtot = b->data[ii].re;
          itrr = b->data[ii].im;
          for (c_i = 0; c_i < jj; c_i++) {
            ii = aoffset + c_i;
            cdiff = numElmAtot * y->data[ii].re - itrr * y->data[ii].im;
            temp_im = numElmAtot * y->data[ii].im + itrr * y->data[ii].re;
            i1 = idx + c_i;
            D->data[i1].re += cdiff;
            D->data[i1].im += temp_im;
          }
        }
      }
    }

    i1 = Su->size[0] * Su->size[1];
    Su->size[0] = D->size[0];
    Su->size[1] = D->size[1];
    emxEnsureCapacity_real_T(Su, i1);
    idx = D->size[0] * D->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      Su->data[i1] = D->data[i1].re;
    }

    c_sparse(Su, Z_d, Z_colidx, Z_rowidx, &aoffset, &Z_n);

    // %%%%%% Update for X
    c_emxCopyStruct_coder_internal_(&Z0, &X);
    sparse_minus(Z_d, Z_colidx, Z_rowidx, Ai, i, j, W_m, W_n, Aj, t8_colidx,
                 t8_rowidx, &idx, &boffset);
    b_sparse_rdivide(Aj, t8_colidx, t8_rowidx, idx, boffset, &X);

    // %%%%%% Stop criteria
    b_sparse_parenReference(Z0.d, Z0.colidx, Z0.rowidx, Z0.m, Z0.n, b_Qt,
      t10_colidx, t10_rowidx, &nx);
    b_sparse_parenReference(X.d, X.colidx, X.rowidx, X.m, X.n, Ai, i, j, &ii);
    b_sparse_minus(b_Qt, t10_colidx, t10_rowidx, Ai, i, j, ii, Aj, t8_colidx,
                   t8_rowidx, &jj);
    sparse_abs(Aj, t8_colidx, t8_rowidx, jj, &expl_temp);
    i1 = b_Qt->size[0];
    b_Qt->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(b_Qt, i1);
    idx = expl_temp.d->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      b_Qt->data[i1] = expl_temp.d->data[i1];
    }

    i1 = t10_colidx->size[0];
    t10_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(t10_colidx, i1);
    idx = expl_temp.colidx->size[0];
    for (i1 = 0; i1 < idx; i1++) {
      t10_colidx->data[i1] = expl_temp.colidx->data[i1];
    }

    nx = expl_temp.m;
    sum(b_Qt, t10_colidx, expl_temp.m, Ai, j, t8_colidx);
    sparse_lt(Ai, j, t12_d, i, t8_colidx);
    if (c_sparse_full(t12_d, i)) {
      //  change in X is small
      exitg1 = true;
    } else {
      //  trace of sparse matrix for MATLAB Coder
      if (X.m < m + 1) {
        b_y->size[0] = 1;
        b_y->size[1] = 0;
      } else if (n - 1 == m + 1) {
        i1 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        idx = X.m - m;
        b_y->size[1] = idx;
        emxEnsureCapacity_real_T(b_y, i1);
        for (i1 = 0; i1 < idx; i1++) {
          b_y->data[i1] = (m + i1) + 1;
        }
      } else {
        numElmAtot = std::floor(((double)X.m - ((double)m + 1.0)) + 0.5);
        itrr = ((double)m + 1.0) + numElmAtot;
        cdiff = itrr - (double)X.m;
        ii = (int)std::abs((double)m + 1.0);
        jj = (int)std::abs((double)X.m);
        if (ii > jj) {
          jj = ii;
        }

        if (std::abs(cdiff) < 4.4408920985006262E-16 * (double)jj) {
          numElmAtot++;
          itrr = X.m;
        } else if (cdiff > 0.0) {
          itrr = ((double)m + 1.0) + (numElmAtot - 1.0);
        } else {
          numElmAtot++;
        }

        if (numElmAtot >= 0.0) {
          c_n = (int)numElmAtot;
        } else {
          c_n = 0;
        }

        i1 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        b_y->size[1] = c_n;
        emxEnsureCapacity_real_T(b_y, i1);
        if (c_n > 0) {
          b_y->data[0] = (double)m + 1.0;
          if (c_n > 1) {
            b_y->data[c_n - 1] = itrr;
            ii = (c_n - 1) >> 1;
            for (k = 0; k <= ii - 2; k++) {
              b_y->data[1 + k] = ((double)m + 1.0) + (1.0 + (double)k);
              b_y->data[(c_n - k) - 2] = itrr - (1.0 + (double)k);
            }

            if (ii << 1 == c_n - 1) {
              b_y->data[ii] = (((double)m + 1.0) + itrr) / 2.0;
            } else {
              b_y->data[ii] = ((double)m + 1.0) + (double)ii;
              b_y->data[ii + 1] = itrr - (double)ii;
            }
          }
        }
      }

      if (X.n < m + 1) {
        c_y->size[0] = 1;
        c_y->size[1] = 0;
      } else if (n - 1 == m + 1) {
        i1 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = 1;
        idx = X.n - m;
        c_y->size[1] = idx;
        emxEnsureCapacity_real_T(c_y, i1);
        for (i1 = 0; i1 < idx; i1++) {
          c_y->data[i1] = (m + i1) + 1;
        }
      } else {
        numElmAtot = std::floor(((double)X.n - ((double)m + 1.0)) + 0.5);
        itrr = ((double)m + 1.0) + numElmAtot;
        cdiff = itrr - (double)X.n;
        ii = (int)std::abs((double)m + 1.0);
        jj = (int)std::abs((double)X.n);
        if (ii > jj) {
          jj = ii;
        }

        if (std::abs(cdiff) < 4.4408920985006262E-16 * (double)jj) {
          numElmAtot++;
          itrr = X.n;
        } else if (cdiff > 0.0) {
          itrr = ((double)m + 1.0) + (numElmAtot - 1.0);
        } else {
          numElmAtot++;
        }

        if (numElmAtot >= 0.0) {
          c_n = (int)numElmAtot;
        } else {
          c_n = 0;
        }

        i1 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = 1;
        c_y->size[1] = c_n;
        emxEnsureCapacity_real_T(c_y, i1);
        if (c_n > 0) {
          c_y->data[0] = (double)m + 1.0;
          if (c_n > 1) {
            c_y->data[c_n - 1] = itrr;
            ii = (c_n - 1) >> 1;
            for (k = 0; k <= ii - 2; k++) {
              c_y->data[1 + k] = ((double)m + 1.0) + (1.0 + (double)k);
              c_y->data[(c_n - k) - 2] = itrr - (1.0 + (double)k);
            }

            if (ii << 1 == c_n - 1) {
              c_y->data[ii] = (((double)m + 1.0) + itrr) / 2.0;
            } else {
              c_y->data[ii] = ((double)m + 1.0) + (double)ii;
              c_y->data[ii + 1] = itrr - (double)ii;
            }
          }
        }
      }

      c_sparse_parenReference(X.d, X.colidx, X.rowidx, b_y, c_y, Aj, t8_colidx,
        t8_rowidx, &idx, &boffset);
      sparse_diag(Aj, t8_colidx, t8_rowidx, idx, boffset, b_Qt, t10_colidx,
                  t10_rowidx, &nx);
      sum(b_Qt, t10_colidx, nx, Ai, j, t8_colidx);
      numElmAtot = d_sparse_full(Ai, j) - ((double)n - 2.0);
      if (std::abs(numElmAtot) / (double)m * 100.0 < 10.0) {
        //  trace of X is close to the desired value
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }

  emxFree_creal_T(&b_dd);
  emxFree_boolean_T(&t12_d);
  emxFree_real_T(&c_y);
  emxFree_real_T(&b_y);
  emxFree_creal_T(&b);
  emxFree_creal_T(&y);
  emxFree_int32_T(&r1);
  emxFree_creal_T(&D);
  emxFree_creal_T(&V);
  emxFree_creal_T(&dd);
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

  i1 = tmpd->size[0];
  tmpd->size[0] = idx;
  emxEnsureCapacity_real_T(tmpd, i1);
  for (i1 = 0; i1 < idx; i1++) {
    tmpd->data[i1] = 0.0 * Z_d->data[i1];
  }

  emxFree_real_T(&Z_d);
  if (Z_colidx->data[Z_colidx->size[0] - 1] - 1 >= 1) {
    ii = Z_colidx->data[Z_colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  emxFree_int32_T(&Z_colidx);
  i1 = b_S.d->size[0];
  b_S.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(b_S.d, i1);
  for (i1 = 0; i1 <= ii; i1++) {
    b_S.d->data[i1] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    b_S.d->data[k] = tmpd->data[k];
  }

  emxFree_real_T(&tmpd);
  b_sparse_fillIn(&b_S);
  sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
  jj = X.colidx->data[X.colidx->size[0] - 1];
  if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
    ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  i1 = I0.d->size[0];
  I0.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(I0.d, i1);
  for (i1 = 0; i1 <= ii; i1++) {
    I0.d->data[i1] = 0.0;
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
  i1 = Z0.d->size[0];
  Z0.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(Z0.d, i1);
  for (i1 = 0; i1 <= ii; i1++) {
    Z0.d->data[i1] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    Z0.d->data[k] = bi->data[k];
  }

  emxFree_real_T(&bi);
  b_sparse_fillIn(&Z0);
  sparse_minus(C_d, C_colidx, C_rowidx, b_S.d, b_S.colidx, b_S.rowidx, b_S.m,
               b_S.n, Aj, t8_colidx, t8_rowidx, &idx, &boffset);
  sparse_minus(Aj, t8_colidx, t8_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m, I0.n,
               Av, t9_colidx, t9_rowidx, &ii, &jj);
  vec(Av, t9_colidx, t9_rowidx, ii, jj, b_Qt, t10_colidx, t10_rowidx, &nx);
  b_sparse_mtimes(A_d, A_colidx, A_rowidx, A_m, b_Qt, t10_colidx, t10_rowidx,
                  &expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  idx = expl_temp.d->size[0];
  emxFree_int32_T(&A_rowidx);
  emxFree_int32_T(&A_colidx);
  emxFree_real_T(&A_d);
  for (i1 = 0; i1 < idx; i1++) {
    Ai->data[i1] = expl_temp.d->data[i1];
  }

  i1 = i->size[0];
  i->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(i, i1);
  idx = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    i->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = j->size[0];
  j->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(j, i1);
  idx = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    j->data[i1] = expl_temp.rowidx->data[i1];
  }

  sparse_plus(Ai, i, j, Z0.d, Z0.colidx, Z0.rowidx, Z0.m, b_Qt, t10_colidx,
              t10_rowidx, &nx);
  sparse_mldivide(AAs_d, AAs_colidx, AAs_rowidx, AAs_m, AAs_n, b_Qt, t10_colidx,
                  t10_rowidx, nx, &expl_temp);
  i1 = Ai->size[0];
  Ai->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  idx = expl_temp.d->size[0];
  emxFree_int32_T(&AAs_rowidx);
  emxFree_int32_T(&AAs_colidx);
  emxFree_real_T(&AAs_d);
  for (i1 = 0; i1 < idx; i1++) {
    Ai->data[i1] = expl_temp.d->data[i1];
  }

  i1 = i->size[0];
  i->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(i, i1);
  idx = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    i->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = j->size[0];
  j->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(j, i1);
  idx = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    j->data[i1] = expl_temp.rowidx->data[i1];
  }

  sparse_copy(X.colidx, X.rowidx, X.m, X.n, &I0);
  jj = X.colidx->data[X.colidx->size[0] - 1];
  if (X.colidx->data[X.colidx->size[0] - 1] - 1 >= 1) {
    ii = X.colidx->data[X.colidx->size[0] - 1] - 2;
  } else {
    ii = 0;
  }

  i1 = I0.d->size[0];
  I0.d->size[0] = ii + 1;
  emxEnsureCapacity_real_T(I0.d, i1);
  for (i1 = 0; i1 <= ii; i1++) {
    I0.d->data[i1] = 0.0;
  }

  for (k = 0; k <= jj - 2; k++) {
    I0.d->data[k] = X.d->data[k];
  }

  b_sparse_fillIn(&I0);
  b_sparse_mtimes(bv, As_colidx, As_rowidx, As_m, Ai, i, j, &expl_temp);
  i1 = b_Qt->size[0];
  b_Qt->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(b_Qt, i1);
  idx = expl_temp.d->size[0];
  emxFree_int32_T(&As_rowidx);
  emxFree_int32_T(&As_colidx);
  emxFree_real_T(&bv);
  for (i1 = 0; i1 < idx; i1++) {
    b_Qt->data[i1] = expl_temp.d->data[i1];
  }

  i1 = t10_colidx->size[0];
  t10_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(t10_colidx, i1);
  idx = expl_temp.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    t10_colidx->data[i1] = expl_temp.colidx->data[i1];
  }

  i1 = t10_rowidx->size[0];
  t10_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(t10_rowidx, i1);
  idx = expl_temp.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    t10_rowidx->data[i1] = expl_temp.rowidx->data[i1];
  }

  d_emxFreeStruct_coder_internal_(&expl_temp);
  b_sizX[0] = sizX;
  b_sizX[1] = sizX;
  sparse_reshape(b_Qt, t10_colidx, t10_rowidx, b_sizX, Aj, t8_colidx, t8_rowidx,
                 &idx, &boffset);
  sparse_minus(C_d, C_colidx, C_rowidx, Aj, t8_colidx, t8_rowidx, idx, boffset,
               Av, t9_colidx, t9_rowidx, &ii, &jj);
  sparse_minus(Av, t9_colidx, t9_rowidx, I0.d, I0.colidx, I0.rowidx, I0.m, I0.n,
               Ai, i, j, &W_m, &W_n);
  sparse_transpose(Ai, i, j, W_m, W_n, Aj, t8_colidx, t8_rowidx, &idx, &boffset);
  b_sparse_plus(Ai, i, j, Aj, t8_colidx, t8_rowidx, idx, boffset, Av, t9_colidx,
                t9_rowidx, &ii, &jj);
  sparse_rdivide(Av, t9_colidx, t9_rowidx, ii, jj, &Z0);
  i1 = Ai->size[0];
  Ai->size[0] = Z0.d->size[0];
  emxEnsureCapacity_real_T(Ai, i1);
  idx = Z0.d->size[0];
  emxFree_real_T(&b_Qt);
  emxFree_int32_T(&t10_rowidx);
  emxFree_int32_T(&t10_colidx);
  emxFree_int32_T(&t9_rowidx);
  emxFree_int32_T(&t9_colidx);
  emxFree_real_T(&Av);
  emxFree_int32_T(&C_rowidx);
  emxFree_int32_T(&C_colidx);
  emxFree_real_T(&C_d);
  c_emxFreeStruct_coder_internal_(&I0);
  for (i1 = 0; i1 < idx; i1++) {
    Ai->data[i1] = Z0.d->data[i1];
  }

  i1 = i->size[0];
  i->size[0] = Z0.colidx->size[0];
  emxEnsureCapacity_int32_T(i, i1);
  idx = Z0.colidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    i->data[i1] = Z0.colidx->data[i1];
  }

  i1 = j->size[0];
  j->size[0] = Z0.rowidx->size[0];
  emxEnsureCapacity_int32_T(j, i1);
  idx = Z0.rowidx->size[0];
  for (i1 = 0; i1 < idx; i1++) {
    j->data[i1] = Z0.rowidx->data[i1];
  }

  sparse_minus(b_S.d, b_S.colidx, b_S.rowidx, Ai, i, j, Z0.m, Z0.n, Aj,
               t8_colidx, t8_rowidx, &idx, &boffset);
  b_sparse_rdivide(Aj, t8_colidx, t8_rowidx, idx, boffset, &X);
  b_sparse_full(X.d, X.colidx, X.rowidx, X.m, X.n, Su);

  //  Transform X from sparse to full representation
  emxFree_int32_T(&t8_rowidx);
  emxFree_int32_T(&t8_colidx);
  emxFree_int32_T(&j);
  emxFree_int32_T(&i);
  c_emxFreeStruct_coder_internal_(&b_S);
  c_emxFreeStruct_coder_internal_(&X);
  emxFree_real_T(&Aj);
  emxFree_real_T(&Ai);
  c_emxFreeStruct_coder_internal_(&Z0);
  if (adj->size[0] - 1 > Su->size[0]) {
    i1 = 1;
    i2 = 0;
  } else {
    i1 = adj->size[0] - 1;
    i2 = Su->size[0];
  }

  if (adj->size[0] - 1 > Su->size[1]) {
    c_i = 1;
    jj = 0;
  } else {
    c_i = adj->size[0] - 1;
    jj = Su->size[1];
  }

  emxInit_real_T(&d_y, 2);
  emxInit_real_T(&b_b, 2);

  //  The component of X corresponding to the gain matrix
  ii = b_b->size[0] * b_b->size[1];
  idx = (i2 - i1) + 1;
  b_b->size[0] = idx;
  nx = (jj - c_i) + 1;
  b_b->size[1] = nx;
  emxEnsureCapacity_real_T(b_b, ii);
  for (i2 = 0; i2 < nx; i2++) {
    for (jj = 0; jj < idx; jj++) {
      b_b->data[jj + b_b->size[0] * i2] = -Su->data[((i1 + jj) + Su->size[0] *
        ((c_i + i2) - 1)) - 1];
    }
  }

  if ((loop_ub == 1) || (b_b->size[0] == 1)) {
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = Q->size[0];
    d_y->size[1] = b_b->size[1];
    emxEnsureCapacity_real_T(d_y, i0);
    idx = Q->size[0];
    for (i0 = 0; i0 < idx; i0++) {
      loop_ub = b_b->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        d_y->data[i0 + d_y->size[0] * i1] = 0.0;
        nx = Q->size[1];
        for (i2 = 0; i2 < nx; i2++) {
          d_y->data[i0 + d_y->size[0] * i1] += Q->data[i0 + Q->size[0] * i2] *
            b_b->data[i2 + b_b->size[0] * i1];
        }
      }
    }
  } else {
    i1 = U->size[0];
    nx = loop_ub - 1;
    c_n = b_b->size[1];
    i2 = U->size[0];
    c_i = d_y->size[0] * d_y->size[1];
    d_y->size[0] = i2;
    d_y->size[1] = b_b->size[1];
    emxEnsureCapacity_real_T(d_y, c_i);
    for (b_j = 0; b_j < c_n; b_j++) {
      idx = b_j * i1;
      boffset = b_j * (nx + 1);
      for (b_i = 0; b_i < i1; b_i++) {
        d_y->data[idx + b_i] = 0.0;
      }

      for (k = 0; k <= nx; k++) {
        aoffset = k * i1;
        numElmAtot = b_b->data[boffset + k];
        for (b_i = 0; b_i < i1; b_i++) {
          i2 = U->size[0];
          c_i = aoffset + b_i;
          jj = idx + b_i;
          d_y->data[jj] += numElmAtot * U->data[c_i % i2 + U->size[0] * (i0 +
            div_nzp_s32_floor(c_i, i2))];
        }
      }
    }
  }

  emxFree_real_T(&b_b);
  emxFree_real_T(&U);
  emxFree_real_T(&Q);
  if ((d_y->size[1] == 1) || (Qt->size[0] == 1)) {
    i0 = Su->size[0] * Su->size[1];
    Su->size[0] = d_y->size[0];
    Su->size[1] = Qt->size[1];
    emxEnsureCapacity_real_T(Su, i0);
    idx = d_y->size[0];
    for (i0 = 0; i0 < idx; i0++) {
      loop_ub = Qt->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        Su->data[i0 + Su->size[0] * i1] = 0.0;
        nx = d_y->size[1];
        for (i2 = 0; i2 < nx; i2++) {
          Su->data[i0 + Su->size[0] * i1] += d_y->data[i0 + d_y->size[0] * i2] *
            Qt->data[i2 + Qt->size[0] * i1];
        }
      }
    }
  } else {
    m = d_y->size[0];
    nx = d_y->size[1];
    c_n = Qt->size[1];
    i0 = Su->size[0] * Su->size[1];
    Su->size[0] = d_y->size[0];
    Su->size[1] = Qt->size[1];
    emxEnsureCapacity_real_T(Su, i0);
    for (b_j = 0; b_j < c_n; b_j++) {
      idx = b_j * m;
      boffset = b_j * nx;
      for (b_i = 0; b_i < m; b_i++) {
        Su->data[idx + b_i] = 0.0;
      }

      for (k = 0; k < nx; k++) {
        aoffset = k * m;
        numElmAtot = Qt->data[boffset + k];
        for (b_i = 0; b_i < m; b_i++) {
          i0 = idx + b_i;
          Su->data[i0] += numElmAtot * d_y->data[aoffset + b_i];
        }
      }
    }
  }

  emxFree_real_T(&d_y);
  emxFree_real_T(&Qt);

  //  The formation gain matrix
  //  i
  //  eig(X2)
  //  trace(X2)
  //  3D formation gain matrix
  i0 = Aopt->size[0] * Aopt->size[1];
  Aopt->size[0] = 3 * adj->size[0];
  Aopt->size[1] = 3 * adj->size[0];
  emxEnsureCapacity_real_T(Aopt, i0);
  idx = 3 * adj->size[0] * (3 * adj->size[0]);
  for (i0 = 0; i0 < idx; i0++) {
    Aopt->data[i0] = 0.0;
  }

  i0 = adj->size[0];
  for (b_i = 0; b_i < i0; b_i++) {
    for (b_j = 0; b_j < n; b_j++) {
      //  Component corresponding to 2D formation
      numElmAtot = 2.0 * (1.0 + (double)b_i);
      itrr = 2.0 * (1.0 + (double)b_j);
      cdiff = 3.0 * (1.0 + (double)b_i);
      temp_im = 3.0 * (1.0 + (double)b_j);
      i1 = (int)itrr;
      i2 = (int)numElmAtot;
      c_i = (int)temp_im;
      jj = (int)cdiff;
      Aopt->data[(jj + Aopt->size[0] * (c_i - 3)) - 3] = Axy->data[(i2 +
        Axy->size[0] * (i1 - 2)) - 2];
      Aopt->data[((int)cdiff + Aopt->size[0] * ((int)temp_im - 3)) - 2] =
        Axy->data[((int)numElmAtot + Axy->size[0] * ((int)itrr - 2)) - 1];
      Aopt->data[(jj + Aopt->size[0] * (c_i - 2)) - 3] = Axy->data[(i2 +
        Axy->size[0] * (i1 - 1)) - 2];
      Aopt->data[((int)cdiff + Aopt->size[0] * ((int)temp_im - 2)) - 2] =
        Axy->data[((int)numElmAtot + Axy->size[0] * ((int)itrr - 1)) - 1];

      //  Component corresponding to altitude
      Aopt->data[(3 * (1 + b_i) + Aopt->size[0] * (3 * (1 + b_j) - 1)) - 1] =
        Su->data[b_i + Su->size[0] * b_j];
    }
  }

  emxFree_real_T(&Su);
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
