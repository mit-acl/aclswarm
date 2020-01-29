//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CXSparseAPI.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "CXSparseAPI.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "sparse.h"
#include "solve_from_qr.h"
#include "makeCXSparseMatrix.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A_d
//                const emxArray_int32_T *A_colidx
//                const emxArray_int32_T *A_rowidx
//                int A_m
//                int A_n
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                int n
//                coder_internal_sparse_1 *out
// Return Type  : void
//
void CXSparseAPI_iteratedQR(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n, const
  emxArray_real_T *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T
  *b_rowidx, int b_m, int n, coder_internal_sparse_1 *out)
{
  emxArray_real_T *outBuff;
  emxArray_int32_T *in_colidx;
  emxArray_int32_T *in_rowidx;
  int in_m;
  int in_n;
  cs_di* cxA;
  cs_dis * S;
  cs_din * N;
  double tol;
  emxArray_real_T *r7;
  emxArray_int32_T *r8;
  emxArray_real_T *t1_d;
  int t1_m;
  int t1_maxnz;
  emxArray_real_T *b_outBuff;
  emxInit_real_T(&outBuff, 1);
  emxInit_int32_T(&in_colidx, 1);
  emxInit_int32_T(&in_rowidx, 1);
  if (A_m < A_n) {
    sparse_transpose(A_d, A_colidx, A_rowidx, A_m, A_n, outBuff, in_colidx,
                     in_rowidx, &in_m, &in_n);
    cxA = makeCXSparseMatrix(in_colidx->data[in_colidx->size[0] - 1] - 1, in_n,
      in_m, &in_colidx->data[0], &in_rowidx->data[0], &outBuff->data[0]);
  } else {
    cxA = makeCXSparseMatrix(A_colidx->data[A_colidx->size[0] - 1] - 1, A_n, A_m,
      &A_colidx->data[0], &A_rowidx->data[0], &A_d->data[0]);
  }

  S = cs_di_sqr(2, cxA, 1);
  N = cs_di_qr(cxA, S);
  cs_di_spfree(cxA);
  qr_rank_di(N, &tol);
  if (b_m < n) {
    in_m = outBuff->size[0];
    outBuff->size[0] = n;
    emxEnsureCapacity_real_T(outBuff, in_m);
  } else {
    in_m = outBuff->size[0];
    outBuff->size[0] = b_m;
    emxEnsureCapacity_real_T(outBuff, in_m);
  }

  if (1 > b_m) {
    in_n = 0;
  } else {
    in_n = b_m;
  }

  emxInit_real_T(&r7, 2);
  emxInit_int32_T(&r8, 2);
  emxInit_real_T(&t1_d, 1);
  sparse_parenReference(b_d, b_colidx, b_rowidx, b_m, t1_d, in_colidx, in_rowidx,
                        &t1_m, &t1_maxnz);
  sparse_full(t1_d, in_colidx, in_rowidx, t1_m, r7);
  in_m = r8->size[0] * r8->size[1];
  r8->size[0] = 1;
  r8->size[1] = in_n;
  emxEnsureCapacity_int32_T(r8, in_m);
  for (in_m = 0; in_m < in_n; in_m++) {
    r8->data[in_m] = in_m;
  }

  in_n = r8->size[0] * r8->size[1];
  for (in_m = 0; in_m < in_n; in_m++) {
    outBuff->data[r8->data[in_m]] = r7->data[in_m];
  }

  emxFree_int32_T(&r8);
  emxFree_real_T(&r7);
  solve_from_qr_di(N, S, (double *)&outBuff->data[0], b_m, n);
  if (1 > n) {
    in_n = 0;
  } else {
    in_n = n;
  }

  emxInit_real_T(&b_outBuff, 1);
  sparse_spallocLike(n, t1_d, in_colidx, in_rowidx, &t1_m, &t1_maxnz);
  in_m = b_outBuff->size[0];
  b_outBuff->size[0] = in_n;
  emxEnsureCapacity_real_T(b_outBuff, in_m);
  for (in_m = 0; in_m < in_n; in_m++) {
    b_outBuff->data[in_m] = outBuff->data[in_m];
  }

  emxFree_real_T(&outBuff);
  sparse_parenAssign(t1_d, in_colidx, in_rowidx, t1_m, t1_maxnz, b_outBuff, out);
  cs_di_sfree(S);
  cs_di_nfree(N);
  emxFree_real_T(&b_outBuff);
  emxFree_real_T(&t1_d);
  emxFree_int32_T(&in_rowidx);
  emxFree_int32_T(&in_colidx);
}

//
// File trailer for CXSparseAPI.cpp
//
// [EOF]
//
