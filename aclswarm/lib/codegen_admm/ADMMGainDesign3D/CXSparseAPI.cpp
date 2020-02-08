/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CXSparseAPI.cpp
 *
 * Code generation for function 'CXSparseAPI'
 *
 */

/* Include files */
#include "CXSparseAPI.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "makeCXSparseMatrix.h"
#include "parenAssign2D.h"
#include "rt_nonfinite.h"
#include "solve_from_qr.h"
#include "sparse.h"

/* Function Definitions */
void CXSparseAPI_iteratedQR(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n, const
  emxArray_real_T *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T
  *b_rowidx, int b_m, int n, coder_internal_sparse_1 *out)
{
  emxArray_real_T *in_d;
  emxArray_int32_T *in_colidx;
  emxArray_int32_T *in_rowidx;
  coder_internal_sparse expl_temp;
  cs_di* cxA;
  int i;
  cs_dis * S;
  cs_din * N;
  int cend;
  double tol;
  emxArray_real_T *outBuff;
  emxArray_real_T *r;
  int idx;
  int nzRhs;
  int k;
  emxInit_real_T(&in_d, 1);
  emxInit_int32_T(&in_colidx, 1);
  emxInit_int32_T(&in_rowidx, 1);
  if (A_m < A_n) {
    c_emxInitStruct_coder_internal_(&expl_temp);
    sparse_transpose(A_d, A_colidx, A_rowidx, A_m, A_n, &expl_temp);
    i = in_d->size[0];
    in_d->size[0] = expl_temp.d->size[0];
    emxEnsureCapacity_real_T(in_d, i);
    cend = expl_temp.d->size[0];
    for (i = 0; i < cend; i++) {
      in_d->data[i] = expl_temp.d->data[i];
    }

    i = in_colidx->size[0];
    in_colidx->size[0] = expl_temp.colidx->size[0];
    emxEnsureCapacity_int32_T(in_colidx, i);
    cend = expl_temp.colidx->size[0];
    for (i = 0; i < cend; i++) {
      in_colidx->data[i] = expl_temp.colidx->data[i];
    }

    i = in_rowidx->size[0];
    in_rowidx->size[0] = expl_temp.rowidx->size[0];
    emxEnsureCapacity_int32_T(in_rowidx, i);
    cend = expl_temp.rowidx->size[0];
    for (i = 0; i < cend; i++) {
      in_rowidx->data[i] = expl_temp.rowidx->data[i];
    }

    cxA = makeCXSparseMatrix(in_colidx->data[in_colidx->size[0] - 1] - 1,
      expl_temp.n, expl_temp.m, &in_colidx->data[0], &in_rowidx->data[0],
      &in_d->data[0]);
    c_emxFreeStruct_coder_internal_(&expl_temp);
  } else {
    cxA = makeCXSparseMatrix(A_colidx->data[A_colidx->size[0] - 1] - 1, A_n, A_m,
      &A_colidx->data[0], &A_rowidx->data[0], &A_d->data[0]);
  }

  S = cs_di_sqr(2, cxA, 1);
  N = cs_di_qr(cxA, S);
  cs_di_spfree(cxA);
  qr_rank_di(N, &tol);
  emxInit_real_T(&outBuff, 1);
  if (b_m < n) {
    i = outBuff->size[0];
    outBuff->size[0] = n;
    emxEnsureCapacity_real_T(outBuff, i);
  } else {
    i = outBuff->size[0];
    outBuff->size[0] = b_m;
    emxEnsureCapacity_real_T(outBuff, i);
  }

  emxInit_real_T(&r, 2);
  sparse_parenReference(b_d, b_colidx, b_rowidx, b_m, in_d, in_colidx, in_rowidx,
                        &cend);
  i = r->size[0] * r->size[1];
  r->size[0] = cend;
  r->size[1] = 1;
  emxEnsureCapacity_real_T(r, i);
  for (i = 0; i < cend; i++) {
    r->data[i] = 0.0;
  }

  cend = in_colidx->data[1] - 1;
  i = in_colidx->data[0];
  emxFree_int32_T(&in_colidx);
  for (idx = i; idx <= cend; idx++) {
    r->data[in_rowidx->data[idx - 1] - 1] = in_d->data[idx - 1];
  }

  emxFree_int32_T(&in_rowidx);
  emxFree_real_T(&in_d);
  if (1 > b_m) {
    cend = 0;
  } else {
    cend = b_m;
  }

  for (i = 0; i < cend; i++) {
    outBuff->data[i] = r->data[i];
  }

  emxFree_real_T(&r);
  solve_from_qr_di(N, S, (double *)&outBuff->data[0], b_m, n);
  out->m = n;
  i = out->d->size[0];
  out->d->size[0] = 1;
  emxEnsureCapacity_real_T(out->d, i);
  out->d->data[0] = 0.0;
  out->maxnz = 1;
  i = out->colidx->size[0];
  out->colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(out->colidx, i);
  i = out->rowidx->size[0];
  out->rowidx->size[0] = 1;
  emxEnsureCapacity_int32_T(out->rowidx, i);
  out->rowidx->data[0] = 0;
  out->colidx->data[0] = 1;
  out->colidx->data[1] = 1;
  if (1 > n) {
    i = 0;
  } else {
    i = n;
  }

  if (i == 1) {
    idx = 0;
    nzRhs = 0;
    for (k = 0; k < n; k++) {
      tol = outBuff->data[idx];
      idx++;
      if (!(tol == 0.0)) {
        nzRhs++;
      }
    }

    if (0 < nzRhs) {
      if (1 < nzRhs) {
        b_realloc(out, nzRhs, 0, 1, 0, nzRhs);
      }

      cend = 0;
      idx = 0;
      i = out->m;
      for (k = 0; k < i; k++) {
        tol = outBuff->data[idx];
        idx++;
        if (tol != 0.0) {
          out->rowidx->data[cend] = k + 1;
          out->d->data[cend] = tol;
          cend++;
        }
      }

      out->colidx->data[1] += nzRhs;
    } else {
      cend = 0;
      idx = 0;
      for (k = 0; k < n; k++) {
        tol = outBuff->data[idx];
        idx++;
        if (tol != 0.0) {
          out->rowidx->data[cend] = k + 1;
          out->d->data[cend] = tol;
          cend++;
        }
      }

      if (-nzRhs > 0) {
        out->colidx->data[1] += nzRhs;
      }
    }
  } else {
    idx = 0;
    nzRhs = 0;
    for (k = 0; k < n; k++) {
      tol = outBuff->data[idx];
      idx++;
      if (!(tol == 0.0)) {
        nzRhs++;
      }
    }

    if (0 < nzRhs) {
      if (1 < nzRhs) {
        b_realloc(out, nzRhs, 0, 1, 0, nzRhs);
      }

      cend = 0;
      idx = 0;
      i = out->m;
      for (k = 0; k < i; k++) {
        tol = outBuff->data[idx];
        idx++;
        if (tol != 0.0) {
          out->rowidx->data[cend] = k + 1;
          out->d->data[cend] = tol;
          cend++;
        }
      }

      out->colidx->data[1] += nzRhs;
    } else {
      cend = 0;
      idx = 0;
      for (k = 0; k < n; k++) {
        tol = outBuff->data[idx];
        idx++;
        if (tol != 0.0) {
          out->rowidx->data[cend] = k + 1;
          out->d->data[cend] = tol;
          cend++;
        }
      }

      if (-nzRhs > 0) {
        out->colidx->data[1] += nzRhs;
      }
    }
  }

  emxFree_real_T(&outBuff);
  cs_di_sfree(S);
  cs_di_nfree(N);
}

/* End of code generation (CXSparseAPI.cpp) */
