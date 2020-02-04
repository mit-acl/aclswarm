//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sparse.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "sparse.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "CXSparseAPI.h"
#include "binOp.h"
#include "fillIn.h"
#include "locBsearch.h"
#include "makeCXSparseMatrix.h"
#include "parenAssign2D.h"
#include "rt_nonfinite.h"
#include "solve_from_lu.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const emxArray_boolean_T *this_d
//                const emxArray_int32_T *this_colidx
// Return Type  : bool
//
bool b_sparse_full(const emxArray_boolean_T *this_d, const emxArray_int32_T
                   *this_colidx)
{
  bool y;
  int cend;
  int i;
  int idx;
  y = false;
  cend = this_colidx->data[1] - 1;
  i = this_colidx->data[0];
  for (idx = i; idx <= cend; idx++) {
    y = this_d->data[0];
  }

  return y;
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
// Return Type  : void
//
void b_sparse_minus(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                    const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d,
                    const emxArray_int32_T *b_colidx, const emxArray_int32_T
                    *b_rowidx, int b_m, emxArray_real_T *s_d, emxArray_int32_T
                    *s_colidx, emxArray_int32_T *s_rowidx, int *s_m)
{
  int numalloc;
  int aidx;
  int bidx;
  bool moreAToDo;
  bool moreBToDo;
  double val;
  numalloc = (a_colidx->data[a_colidx->size[0] - 1] + b_colidx->data
              [b_colidx->size[0] - 1]) - 2;
  if (numalloc >= b_m) {
    numalloc = b_m;
  }

  if (numalloc < 1) {
    numalloc = 1;
  }

  sparse_spallocLike(b_m, numalloc, s_d, s_colidx, s_rowidx, s_m, &aidx);
  numalloc = 1;
  s_colidx->data[0] = 1;
  aidx = a_colidx->data[0] - 1;
  bidx = b_colidx->data[0] - 1;
  moreAToDo = (a_colidx->data[0] < a_colidx->data[1]);
  moreBToDo = (b_colidx->data[0] < b_colidx->data[1]);
  while (moreAToDo || moreBToDo) {
    while ((aidx + 1 < a_colidx->data[1]) && ((!moreBToDo) || (a_rowidx->
             data[aidx] < b_rowidx->data[bidx]))) {
      if (a_d->data[aidx] != 0.0) {
        s_d->data[numalloc - 1] = a_d->data[aidx];
        s_rowidx->data[numalloc - 1] = a_rowidx->data[aidx];
        numalloc++;
      }

      aidx++;
    }

    moreAToDo = (aidx + 1 < a_colidx->data[1]);
    while ((bidx + 1 < b_colidx->data[1]) && ((!moreAToDo) || (b_rowidx->
             data[bidx] < a_rowidx->data[aidx]))) {
      if (0.0 - b_d->data[bidx] != 0.0) {
        s_d->data[numalloc - 1] = 0.0 - b_d->data[bidx];
        s_rowidx->data[numalloc - 1] = b_rowidx->data[bidx];
        numalloc++;
      }

      bidx++;
    }

    while ((aidx + 1 < a_colidx->data[1]) && (bidx + 1 < b_colidx->data[1]) &&
           (a_rowidx->data[aidx] == b_rowidx->data[bidx])) {
      val = a_d->data[aidx] - b_d->data[bidx];
      if (val != 0.0) {
        s_d->data[numalloc - 1] = val;
        s_rowidx->data[numalloc - 1] = b_rowidx->data[bidx];
        numalloc++;
      }

      bidx++;
      aidx++;
    }

    moreAToDo = (aidx + 1 < a_colidx->data[1]);
    moreBToDo = (bidx + 1 < b_colidx->data[1]);
  }

  s_colidx->data[1] = numalloc;
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                int this_n
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
// Return Type  : void
//
void b_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_n, emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T
  *s_rowidx, int *s_m)
{
  int lowOrderA;
  int ridx;
  int partialResults_idx_0;
  int partialResults_idx_1;
  int partialResults_idx_2;
  lowOrderA = this_m & 65535;
  ridx = this_n & 65535;
  partialResults_idx_0 = lowOrderA * ridx;
  partialResults_idx_1 = (lowOrderA * (this_n >> 16)) << 16;
  partialResults_idx_2 = ((this_m >> 16) * ridx) << 16;
  if (lowOrderA * ridx > MAX_int32_T - partialResults_idx_1) {
    ridx = (partialResults_idx_0 + partialResults_idx_1) - MAX_int32_T;
  } else {
    ridx = partialResults_idx_0 + partialResults_idx_1;
  }

  if (ridx > MAX_int32_T - partialResults_idx_2) {
    ridx = (ridx + partialResults_idx_2) - MAX_int32_T;
  } else {
    ridx += partialResults_idx_2;
  }

  partialResults_idx_0 = this_colidx->data[this_colidx->size[0] - 1];
  sparse_spallocLike(ridx, this_colidx->data[this_colidx->size[0] - 1] - 1, s_d,
                     s_colidx, s_rowidx, s_m, &lowOrderA);
  s_colidx->data[0] = 1;
  s_colidx->data[s_colidx->size[0] - 1] = this_colidx->data[this_colidx->size[0]
    - 1];
  for (lowOrderA = 0; lowOrderA <= partialResults_idx_0 - 2; lowOrderA++) {
    s_d->data[lowOrderA] = this_d->data[lowOrderA];
  }

  for (lowOrderA = 0; lowOrderA < this_n; lowOrderA++) {
    ridx = this_colidx->data[lowOrderA];
    partialResults_idx_0 = lowOrderA * this_m;
    while (ridx < this_colidx->data[lowOrderA + 1]) {
      s_rowidx->data[ridx - 1] = partialResults_idx_0 + this_rowidx->data[ridx -
        1];
      ridx++;
    }
  }
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                int b_n
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
//                int *s_n
// Return Type  : void
//
void b_sparse_plus(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                   const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d,
                   const emxArray_int32_T *b_colidx, const emxArray_int32_T
                   *b_rowidx, int b_m, int b_n, emxArray_real_T *s_d,
                   emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx, int
                   *s_m, int *s_n)
{
  coder_internal_sparse expl_temp;
  int c;
  int didx;
  int b_c;
  int aidx_tmp;
  int aidx;
  int bidx_tmp;
  int bidx;
  bool moreAToDo;
  bool moreBToDo;
  double val;
  c_emxInitStruct_coder_internal_(&expl_temp);
  allocEqsizeBinop(a_colidx, b_colidx, b_n, b_m, &expl_temp);
  c = s_d->size[0];
  s_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(s_d, c);
  didx = expl_temp.d->size[0];
  for (c = 0; c < didx; c++) {
    s_d->data[c] = expl_temp.d->data[c];
  }

  c = s_colidx->size[0];
  s_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(s_colidx, c);
  didx = expl_temp.colidx->size[0];
  for (c = 0; c < didx; c++) {
    s_colidx->data[c] = expl_temp.colidx->data[c];
  }

  c = s_rowidx->size[0];
  s_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(s_rowidx, c);
  didx = expl_temp.rowidx->size[0];
  for (c = 0; c < didx; c++) {
    s_rowidx->data[c] = expl_temp.rowidx->data[c];
  }

  *s_m = expl_temp.m;
  *s_n = expl_temp.n;
  didx = 1;
  s_colidx->data[0] = 1;
  c_emxFreeStruct_coder_internal_(&expl_temp);
  for (c = 0; c < *s_n; c++) {
    b_c = c + 1;
    aidx_tmp = a_colidx->data[b_c - 1];
    aidx = aidx_tmp - 1;
    bidx_tmp = b_colidx->data[b_c - 1];
    bidx = bidx_tmp - 1;
    moreAToDo = (aidx_tmp < a_colidx->data[b_c]);
    moreBToDo = (bidx_tmp < b_colidx->data[b_c]);
    while (moreAToDo || moreBToDo) {
      while ((aidx + 1 < a_colidx->data[b_c]) && ((!moreBToDo) ||
              (a_rowidx->data[aidx] < b_rowidx->data[bidx]))) {
        if (a_d->data[aidx] != 0.0) {
          s_d->data[didx - 1] = a_d->data[aidx];
          s_rowidx->data[didx - 1] = a_rowidx->data[aidx];
          didx++;
        }

        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[b_c]);
      while ((bidx + 1 < b_colidx->data[b_c]) && ((!moreAToDo) ||
              (b_rowidx->data[bidx] < a_rowidx->data[aidx]))) {
        if (b_d->data[bidx] != 0.0) {
          s_d->data[didx - 1] = b_d->data[bidx];
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
      }

      while ((aidx + 1 < a_colidx->data[b_c]) && (bidx + 1 < b_colidx->data[b_c])
             && (a_rowidx->data[aidx] == b_rowidx->data[bidx])) {
        val = a_d->data[aidx] + b_d->data[bidx];
        if (val != 0.0) {
          s_d->data[didx - 1] = val;
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[b_c]);
      moreBToDo = (bidx + 1 < b_colidx->data[b_c]);
    }

    s_colidx->data[b_c] = didx;
  }
}

//
// Arguments    : const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                int b_n
//                coder_internal_sparse *s
// Return Type  : void
//
void b_sparse_times(const emxArray_real_T *b_d, const emxArray_int32_T *b_colidx,
                    const emxArray_int32_T *b_rowidx, int b_m, int b_n,
                    coder_internal_sparse *s)
{
  int nzs_tmp;
  int numalloc;
  emxArray_real_T *tmpd;
  int i;
  nzs_tmp = b_colidx->data[b_colidx->size[0] - 1];
  if (1 > b_colidx->data[b_colidx->size[0] - 1] - 1) {
    numalloc = 0;
  } else {
    numalloc = b_colidx->data[b_colidx->size[0] - 1] - 1;
  }

  emxInit_real_T(&tmpd, 1);
  i = tmpd->size[0];
  tmpd->size[0] = numalloc;
  emxEnsureCapacity_real_T(tmpd, i);
  for (i = 0; i < numalloc; i++) {
    tmpd->data[i] = 0.0 * b_d->data[i];
  }

  s->m = b_m;
  s->n = b_n;
  if (b_colidx->data[b_colidx->size[0] - 1] - 1 >= 1) {
    numalloc = b_colidx->data[b_colidx->size[0] - 1] - 2;
  } else {
    numalloc = 0;
  }

  i = s->d->size[0];
  s->d->size[0] = numalloc + 1;
  emxEnsureCapacity_real_T(s->d, i);
  for (i = 0; i <= numalloc; i++) {
    s->d->data[i] = 0.0;
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = b_n + 1;
  emxEnsureCapacity_int32_T(s->colidx, i);
  s->colidx->data[0] = 1;
  i = s->rowidx->size[0];
  s->rowidx->size[0] = numalloc + 1;
  emxEnsureCapacity_int32_T(s->rowidx, i);
  for (i = 0; i <= numalloc; i++) {
    s->rowidx->data[i] = 0;
  }

  for (numalloc = 0; numalloc < b_n; numalloc++) {
    s->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(s);
  if (1 > b_colidx->data[b_colidx->size[0] - 1] - 1) {
    numalloc = 1;
  } else {
    numalloc = b_colidx->data[b_colidx->size[0] - 1];
  }

  for (i = 0; i <= numalloc - 2; i++) {
    s->rowidx->data[i] = b_rowidx->data[i];
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = b_colidx->size[0];
  emxEnsureCapacity_int32_T(s->colidx, i);
  numalloc = b_colidx->size[0];
  for (i = 0; i < numalloc; i++) {
    s->colidx->data[i] = b_colidx->data[i];
  }

  for (numalloc = 0; numalloc <= nzs_tmp - 2; numalloc++) {
    s->d->data[numalloc] = tmpd->data[numalloc];
  }

  emxFree_real_T(&tmpd);
  b_sparse_fillIn(s);
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
// Return Type  : double
//
double c_sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
                     *this_colidx)
{
  double y;
  int cend;
  int i;
  int idx;
  y = 0.0;
  cend = this_colidx->data[1] - 1;
  i = this_colidx->data[0];
  for (idx = i; idx <= cend; idx++) {
    y = this_d->data[idx - 1];
  }

  return y;
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                const emxArray_real_T *varargin_1
//                const emxArray_real_T *varargin_2
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
//                int *s_n
// Return Type  : void
//
void c_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_n)
{
  int sm;
  int sn;
  int i;
  int colNnz;
  int k;
  int cidx;
  double nt;
  int ridx;
  int idx;
  bool found;
  int i1;
  double s_d_tmp;
  sm = varargin_1->size[1];
  sn = varargin_2->size[1];
  s_d->size[0] = 0;
  s_rowidx->size[0] = 0;
  i = s_colidx->size[0];
  s_colidx->size[0] = varargin_2->size[1] + 1;
  emxEnsureCapacity_int32_T(s_colidx, i);
  colNnz = varargin_2->size[1];
  for (i = 0; i <= colNnz; i++) {
    s_colidx->data[i] = 0;
  }

  s_colidx->data[0] = 1;
  colNnz = 1;
  k = 0;
  for (cidx = 0; cidx < sn; cidx++) {
    nt = varargin_2->data[cidx];
    for (ridx = 0; ridx < sm; ridx++) {
      sparse_locBsearch(this_rowidx, static_cast<int>(varargin_1->data[ridx]),
                        this_colidx->data[static_cast<int>(nt) - 1],
                        this_colidx->data[static_cast<int>(nt)], &idx, &found);
      if (found) {
        i = s_d->size[0];
        i1 = s_d->size[0];
        s_d->size[0]++;
        emxEnsureCapacity_real_T(s_d, i1);
        s_d_tmp = this_d->data[idx - 1];
        s_d->data[i] = s_d_tmp;
        i = s_rowidx->size[0];
        i1 = s_rowidx->size[0];
        s_rowidx->size[0]++;
        emxEnsureCapacity_int32_T(s_rowidx, i1);
        s_rowidx->data[i] = ridx + 1;
        s_d->data[k] = s_d_tmp;
        s_rowidx->data[k] = ridx + 1;
        k++;
        colNnz++;
      }
    }

    s_colidx->data[cidx + 1] = colNnz;
  }

  if (s_colidx->data[s_colidx->size[0] - 1] - 1 == 0) {
    i = s_rowidx->size[0];
    s_rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(s_rowidx, i);
    s_rowidx->data[0] = 1;
    i = s_d->size[0];
    s_d->size[0] = 1;
    emxEnsureCapacity_real_T(s_d, i);
    s_d->data[0] = 0.0;
  }

  *s_m = varargin_1->size[1];
  *s_n = varargin_2->size[1];
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                int a_m
//                coder_internal_sparse_1 *s
// Return Type  : void
//
void sparse_abs(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                const emxArray_int32_T *a_rowidx, int a_m,
                coder_internal_sparse_1 *s)
{
  int nzs_tmp;
  int i;
  emxArray_real_T *tmpd;
  int k;
  nzs_tmp = a_colidx->data[a_colidx->size[0] - 1];
  if (1 > a_colidx->data[a_colidx->size[0] - 1] - 1) {
    i = -1;
  } else {
    i = a_colidx->data[a_colidx->size[0] - 1] - 2;
  }

  emxInit_real_T(&tmpd, 1);
  k = tmpd->size[0];
  tmpd->size[0] = i + 1;
  emxEnsureCapacity_real_T(tmpd, k);
  for (k = 0; k <= i; k++) {
    tmpd->data[k] = std::abs(a_d->data[k]);
  }

  sparse_spallocLike(a_m, a_colidx->data[a_colidx->size[0] - 1] - 1, s->d,
                     s->colidx, s->rowidx, &s->m, &s->maxnz);
  if (1 > a_colidx->data[a_colidx->size[0] - 1] - 1) {
    k = 1;
  } else {
    k = a_colidx->data[a_colidx->size[0] - 1];
  }

  for (i = 0; i <= k - 2; i++) {
    s->rowidx->data[i] = a_rowidx->data[i];
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = a_colidx->size[0];
  emxEnsureCapacity_int32_T(s->colidx, i);
  k = a_colidx->size[0];
  for (i = 0; i < k; i++) {
    s->colidx->data[i] = a_colidx->data[i];
  }

  for (k = 0; k <= nzs_tmp - 2; k++) {
    s->d->data[k] = tmpd->data[k];
  }

  emxFree_real_T(&tmpd);
  c_sparse_fillIn(s);
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                int this_n
//                emxArray_real_T *y
// Return Type  : void
//
void sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
                 *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
                 int this_n, emxArray_real_T *y)
{
  int i;
  int loop_ub;
  int cend;
  int idx;
  i = y->size[0] * y->size[1];
  y->size[0] = this_m;
  y->size[1] = this_n;
  emxEnsureCapacity_real_T(y, i);
  loop_ub = this_m * this_n;
  for (i = 0; i < loop_ub; i++) {
    y->data[i] = 0.0;
  }

  for (loop_ub = 0; loop_ub < this_n; loop_ub++) {
    cend = this_colidx->data[loop_ub + 1] - 1;
    i = this_colidx->data[loop_ub];
    for (idx = i; idx <= cend; idx++) {
      y->data[(this_rowidx->data[idx - 1] + y->size[0] * loop_ub) - 1] =
        this_d->data[idx - 1];
    }
  }
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                emxArray_boolean_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
// Return Type  : void
//
void sparse_lt(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
               emxArray_boolean_T *s_d, emxArray_int32_T *s_colidx,
               emxArray_int32_T *s_rowidx)
{
  double uniOp_tunableEnvironment_idx_0;
  int i;
  if (a_colidx->data[a_colidx->size[0] - 1] - 1 > 0) {
    uniOp_tunableEnvironment_idx_0 = a_d->data[0];
  } else {
    uniOp_tunableEnvironment_idx_0 = 0.0;
  }

  i = s_d->size[0];
  s_d->size[0] = 1;
  emxEnsureCapacity_boolean_T(s_d, i);
  s_d->data[0] = false;
  i = s_colidx->size[0];
  s_colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(s_colidx, i);
  s_colidx->data[0] = 1;
  i = s_rowidx->size[0];
  s_rowidx->size[0] = 1;
  emxEnsureCapacity_int32_T(s_rowidx, i);
  s_rowidx->data[0] = 1;
  s_colidx->data[1] = 1;
  if (uniOp_tunableEnvironment_idx_0 < 0.0001) {
    s_rowidx->data[0] = 1;
    s_d->data[0] = true;
    s_colidx->data[1] = 2;
  }
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                int b_n
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
//                int *s_n
// Return Type  : void
//
void sparse_minus(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                  const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d,
                  const emxArray_int32_T *b_colidx, const emxArray_int32_T
                  *b_rowidx, int b_m, int b_n, emxArray_real_T *s_d,
                  emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx, int
                  *s_m, int *s_n)
{
  coder_internal_sparse expl_temp;
  int c;
  int didx;
  int b_c;
  int aidx_tmp;
  int aidx;
  int bidx_tmp;
  int bidx;
  bool moreAToDo;
  bool moreBToDo;
  double val;
  c_emxInitStruct_coder_internal_(&expl_temp);
  allocEqsizeBinop(a_colidx, b_colidx, b_n, b_m, &expl_temp);
  c = s_d->size[0];
  s_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(s_d, c);
  didx = expl_temp.d->size[0];
  for (c = 0; c < didx; c++) {
    s_d->data[c] = expl_temp.d->data[c];
  }

  c = s_colidx->size[0];
  s_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(s_colidx, c);
  didx = expl_temp.colidx->size[0];
  for (c = 0; c < didx; c++) {
    s_colidx->data[c] = expl_temp.colidx->data[c];
  }

  c = s_rowidx->size[0];
  s_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(s_rowidx, c);
  didx = expl_temp.rowidx->size[0];
  for (c = 0; c < didx; c++) {
    s_rowidx->data[c] = expl_temp.rowidx->data[c];
  }

  *s_m = expl_temp.m;
  *s_n = expl_temp.n;
  didx = 1;
  s_colidx->data[0] = 1;
  c_emxFreeStruct_coder_internal_(&expl_temp);
  for (c = 0; c < *s_n; c++) {
    b_c = c + 1;
    aidx_tmp = a_colidx->data[b_c - 1];
    aidx = aidx_tmp - 1;
    bidx_tmp = b_colidx->data[b_c - 1];
    bidx = bidx_tmp - 1;
    moreAToDo = (aidx_tmp < a_colidx->data[b_c]);
    moreBToDo = (bidx_tmp < b_colidx->data[b_c]);
    while (moreAToDo || moreBToDo) {
      while ((aidx + 1 < a_colidx->data[b_c]) && ((!moreBToDo) ||
              (a_rowidx->data[aidx] < b_rowidx->data[bidx]))) {
        if (a_d->data[aidx] != 0.0) {
          s_d->data[didx - 1] = a_d->data[aidx];
          s_rowidx->data[didx - 1] = a_rowidx->data[aidx];
          didx++;
        }

        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[b_c]);
      while ((bidx + 1 < b_colidx->data[b_c]) && ((!moreAToDo) ||
              (b_rowidx->data[bidx] < a_rowidx->data[aidx]))) {
        if (0.0 - b_d->data[bidx] != 0.0) {
          s_d->data[didx - 1] = 0.0 - b_d->data[bidx];
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
      }

      while ((aidx + 1 < a_colidx->data[b_c]) && (bidx + 1 < b_colidx->data[b_c])
             && (a_rowidx->data[aidx] == b_rowidx->data[bidx])) {
        val = a_d->data[aidx] - b_d->data[bidx];
        if (val != 0.0) {
          s_d->data[didx - 1] = val;
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[b_c]);
      moreBToDo = (bidx + 1 < b_colidx->data[b_c]);
    }

    s_colidx->data[b_c] = didx;
  }
}

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
//                coder_internal_sparse_1 *y
// Return Type  : void
//
void sparse_mldivide(const emxArray_real_T *A_d, const emxArray_int32_T
                     *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int
                     A_n, const emxArray_real_T *b_d, const emxArray_int32_T
                     *b_colidx, const emxArray_int32_T *b_rowidx, int b_m,
                     coder_internal_sparse_1 *y)
{
  bool p;
  emxArray_real_T *tmp;
  emxArray_real_T *in_d;
  emxArray_int32_T *in_colidx;
  emxArray_int32_T *in_rowidx;
  coder_internal_sparse expl_temp;
  int i;
  cs_di* cxA;
  cs_dis * S;
  cs_din * N;
  int cend;
  int idx;
  int nzRhs;
  int k;
  double rhsv;
  if ((A_m == 0) || (A_n == 0)) {
    p = true;
  } else {
    p = false;
  }

  emxInit_real_T(&tmp, 2);
  emxInit_real_T(&in_d, 1);
  emxInit_int32_T(&in_colidx, 1);
  emxInit_int32_T(&in_rowidx, 1);
  c_emxInitStruct_coder_internal_(&expl_temp);
  if (p || (b_m == 0)) {
    y->m = A_n;
    i = y->d->size[0];
    y->d->size[0] = 1;
    emxEnsureCapacity_real_T(y->d, i);
    y->d->data[0] = 0.0;
    y->maxnz = 1;
    i = y->colidx->size[0];
    y->colidx->size[0] = 2;
    emxEnsureCapacity_int32_T(y->colidx, i);
    i = y->rowidx->size[0];
    y->rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(y->rowidx, i);
    y->rowidx->data[0] = 0;
    y->colidx->data[0] = 1;
    y->colidx->data[1] = 1;
  } else if (b_m == A_n) {
    if (A_m < A_n) {
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
    } else {
      cxA = makeCXSparseMatrix(A_colidx->data[A_colidx->size[0] - 1] - 1, A_n,
        A_m, &A_colidx->data[0], &A_rowidx->data[0], &A_d->data[0]);
    }

    S = cs_di_sqr(2, cxA, 0);
    N = cs_di_lu(cxA, S, 1);
    cs_di_spfree(cxA);
    if (N == NULL) {
      cs_di_sfree(S);
      cs_di_nfree(N);
      CXSparseAPI_iteratedQR(A_d, A_colidx, A_rowidx, A_m, A_n, b_d, b_colidx,
        b_rowidx, b_m, A_n, y);
    } else {
      y->m = A_n;
      i = y->d->size[0];
      y->d->size[0] = 1;
      emxEnsureCapacity_real_T(y->d, i);
      y->d->data[0] = 0.0;
      y->maxnz = 1;
      i = y->colidx->size[0];
      y->colidx->size[0] = 2;
      emxEnsureCapacity_int32_T(y->colidx, i);
      i = y->rowidx->size[0];
      y->rowidx->size[0] = 1;
      emxEnsureCapacity_int32_T(y->rowidx, i);
      y->rowidx->data[0] = 0;
      y->colidx->data[0] = 1;
      y->colidx->data[1] = 1;
      sparse_parenReference(b_d, b_colidx, b_rowidx, b_m, in_d, in_colidx,
                            in_rowidx, &cend);
      i = tmp->size[0] * tmp->size[1];
      tmp->size[0] = cend;
      tmp->size[1] = 1;
      emxEnsureCapacity_real_T(tmp, i);
      for (i = 0; i < cend; i++) {
        tmp->data[i] = 0.0;
      }

      cend = in_colidx->data[1] - 1;
      i = in_colidx->data[0];
      for (idx = i; idx <= cend; idx++) {
        tmp->data[in_rowidx->data[idx - 1] - 1] = in_d->data[idx - 1];
      }

      solve_from_lu_di(N, S, (double *)&tmp->data[0], b_m);
      idx = 0;
      nzRhs = 0;
      for (k = 0; k < A_n; k++) {
        rhsv = tmp->data[idx];
        idx++;
        if (!(rhsv == 0.0)) {
          nzRhs++;
        }
      }

      if (0 < nzRhs) {
        if (1 < nzRhs) {
          b_realloc(y, nzRhs, 0, 1, 0, nzRhs);
        }

        cend = 0;
        idx = 0;
        i = y->m;
        for (k = 0; k < i; k++) {
          rhsv = tmp->data[idx];
          idx++;
          if (rhsv != 0.0) {
            y->rowidx->data[cend] = k + 1;
            y->d->data[cend] = rhsv;
            cend++;
          }
        }

        y->colidx->data[1] += nzRhs;
      } else {
        cend = 0;
        idx = 0;
        for (k = 0; k < A_n; k++) {
          rhsv = tmp->data[idx];
          idx++;
          if (rhsv != 0.0) {
            y->rowidx->data[cend] = k + 1;
            y->d->data[cend] = rhsv;
            cend++;
          }
        }

        if (-nzRhs > 0) {
          y->colidx->data[1] += nzRhs;
        }
      }

      cs_di_sfree(S);
      cs_di_nfree(N);
    }
  } else {
    CXSparseAPI_iteratedQR(A_d, A_colidx, A_rowidx, A_m, A_n, b_d, b_colidx,
      b_rowidx, b_m, A_n, y);
  }

  c_emxFreeStruct_coder_internal_(&expl_temp);
  emxFree_int32_T(&in_rowidx);
  emxFree_int32_T(&in_colidx);
  emxFree_real_T(&in_d);
  emxFree_real_T(&tmp);
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
// Return Type  : void
//
void sparse_parenReference(const emxArray_real_T *this_d, const emxArray_int32_T
  *this_colidx, const emxArray_int32_T *this_rowidx, int this_m, emxArray_real_T
  *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx, int *s_m)
{
  int nd_tmp;
  int outIdx;
  int colstart;
  int k;
  int s_d_tmp;
  nd_tmp = this_colidx->data[1] - this_colidx->data[0];
  sparse_spallocLike(this_m, nd_tmp, s_d, s_colidx, s_rowidx, s_m, &outIdx);
  if (nd_tmp != 0) {
    outIdx = 0;
    colstart = this_colidx->data[0] - 2;
    for (k = 0; k < nd_tmp; k++) {
      s_d_tmp = (colstart + k) + 1;
      s_d->data[outIdx] = this_d->data[s_d_tmp];
      s_rowidx->data[outIdx] = this_rowidx->data[s_d_tmp];
      outIdx++;
    }

    s_colidx->data[1] = s_colidx->data[0] + nd_tmp;
  }
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
// Return Type  : void
//
void sparse_plus(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                 const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d,
                 const emxArray_int32_T *b_colidx, const emxArray_int32_T
                 *b_rowidx, int b_m, emxArray_real_T *s_d, emxArray_int32_T
                 *s_colidx, emxArray_int32_T *s_rowidx, int *s_m)
{
  int numalloc;
  int aidx;
  int bidx;
  bool moreAToDo;
  bool moreBToDo;
  double val;
  numalloc = (a_colidx->data[a_colidx->size[0] - 1] + b_colidx->data
              [b_colidx->size[0] - 1]) - 2;
  if (numalloc >= b_m) {
    numalloc = b_m;
  }

  if (numalloc < 1) {
    numalloc = 1;
  }

  sparse_spallocLike(b_m, numalloc, s_d, s_colidx, s_rowidx, s_m, &aidx);
  numalloc = 1;
  s_colidx->data[0] = 1;
  aidx = a_colidx->data[0] - 1;
  bidx = b_colidx->data[0] - 1;
  moreAToDo = (a_colidx->data[0] < a_colidx->data[1]);
  moreBToDo = (b_colidx->data[0] < b_colidx->data[1]);
  while (moreAToDo || moreBToDo) {
    while ((aidx + 1 < a_colidx->data[1]) && ((!moreBToDo) || (a_rowidx->
             data[aidx] < b_rowidx->data[bidx]))) {
      if (a_d->data[aidx] != 0.0) {
        s_d->data[numalloc - 1] = a_d->data[aidx];
        s_rowidx->data[numalloc - 1] = a_rowidx->data[aidx];
        numalloc++;
      }

      aidx++;
    }

    moreAToDo = (aidx + 1 < a_colidx->data[1]);
    while ((bidx + 1 < b_colidx->data[1]) && ((!moreAToDo) || (b_rowidx->
             data[bidx] < a_rowidx->data[aidx]))) {
      if (b_d->data[bidx] != 0.0) {
        s_d->data[numalloc - 1] = b_d->data[bidx];
        s_rowidx->data[numalloc - 1] = b_rowidx->data[bidx];
        numalloc++;
      }

      bidx++;
    }

    while ((aidx + 1 < a_colidx->data[1]) && (bidx + 1 < b_colidx->data[1]) &&
           (a_rowidx->data[aidx] == b_rowidx->data[bidx])) {
      val = a_d->data[aidx] + b_d->data[bidx];
      if (val != 0.0) {
        s_d->data[numalloc - 1] = val;
        s_rowidx->data[numalloc - 1] = b_rowidx->data[bidx];
        numalloc++;
      }

      bidx++;
      aidx++;
    }

    moreAToDo = (aidx + 1 < a_colidx->data[1]);
    moreBToDo = (bidx + 1 < b_colidx->data[1]);
  }

  s_colidx->data[1] = numalloc;
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                int a_m
//                int a_n
//                coder_internal_sparse *s
// Return Type  : void
//
void sparse_rdivide(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                    const emxArray_int32_T *a_rowidx, int a_m, int a_n,
                    coder_internal_sparse *s)
{
  int nzs_tmp;
  int numalloc;
  emxArray_real_T *tmpd;
  int i;
  nzs_tmp = a_colidx->data[a_colidx->size[0] - 1];
  if (1 > a_colidx->data[a_colidx->size[0] - 1] - 1) {
    numalloc = 0;
  } else {
    numalloc = a_colidx->data[a_colidx->size[0] - 1] - 1;
  }

  emxInit_real_T(&tmpd, 1);
  i = tmpd->size[0];
  tmpd->size[0] = numalloc;
  emxEnsureCapacity_real_T(tmpd, i);
  for (i = 0; i < numalloc; i++) {
    tmpd->data[i] = a_d->data[i] / 2.0;
  }

  s->m = a_m;
  s->n = a_n;
  if (a_colidx->data[a_colidx->size[0] - 1] - 1 >= 1) {
    numalloc = a_colidx->data[a_colidx->size[0] - 1] - 2;
  } else {
    numalloc = 0;
  }

  i = s->d->size[0];
  s->d->size[0] = numalloc + 1;
  emxEnsureCapacity_real_T(s->d, i);
  for (i = 0; i <= numalloc; i++) {
    s->d->data[i] = 0.0;
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = a_n + 1;
  emxEnsureCapacity_int32_T(s->colidx, i);
  s->colidx->data[0] = 1;
  i = s->rowidx->size[0];
  s->rowidx->size[0] = numalloc + 1;
  emxEnsureCapacity_int32_T(s->rowidx, i);
  for (i = 0; i <= numalloc; i++) {
    s->rowidx->data[i] = 0;
  }

  for (numalloc = 0; numalloc < a_n; numalloc++) {
    s->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(s);
  if (1 > a_colidx->data[a_colidx->size[0] - 1] - 1) {
    numalloc = 1;
  } else {
    numalloc = a_colidx->data[a_colidx->size[0] - 1];
  }

  for (i = 0; i <= numalloc - 2; i++) {
    s->rowidx->data[i] = a_rowidx->data[i];
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = a_colidx->size[0];
  emxEnsureCapacity_int32_T(s->colidx, i);
  numalloc = a_colidx->size[0];
  for (i = 0; i < numalloc; i++) {
    s->colidx->data[i] = a_colidx->data[i];
  }

  for (numalloc = 0; numalloc <= nzs_tmp - 2; numalloc++) {
    s->d->data[numalloc] = tmpd->data[numalloc];
  }

  emxFree_real_T(&tmpd);
  b_sparse_fillIn(s);
}

//
// Arguments    : int m
//                int nzmax
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
//                int *s_maxnz
// Return Type  : void
//
void sparse_spallocLike(int m, int nzmax, emxArray_real_T *s_d, emxArray_int32_T
  *s_colidx, emxArray_int32_T *s_rowidx, int *s_m, int *s_maxnz)
{
  int numalloc;
  int i;
  if (nzmax >= 1) {
    numalloc = nzmax;
  } else {
    numalloc = 1;
  }

  i = s_d->size[0];
  s_d->size[0] = numalloc;
  emxEnsureCapacity_real_T(s_d, i);
  for (i = 0; i < numalloc; i++) {
    s_d->data[i] = 0.0;
  }

  i = s_colidx->size[0];
  s_colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(s_colidx, i);
  i = s_rowidx->size[0];
  s_rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(s_rowidx, i);
  for (i = 0; i < numalloc; i++) {
    s_rowidx->data[i] = 0;
  }

  s_colidx->data[0] = 1;
  s_colidx->data[1] = 1;
  *s_m = m;
  *s_maxnz = numalloc;
}

//
// Arguments    : const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_m
//                int b_n
//                coder_internal_sparse *s
// Return Type  : void
//
void sparse_times(const emxArray_real_T *b_d, const emxArray_int32_T *b_colidx,
                  const emxArray_int32_T *b_rowidx, int b_m, int b_n,
                  coder_internal_sparse *s)
{
  int nzs_tmp;
  int numalloc;
  int i;
  nzs_tmp = b_colidx->data[b_colidx->size[0] - 1];
  s->m = b_m;
  s->n = b_n;
  if (b_colidx->data[b_colidx->size[0] - 1] - 1 >= 1) {
    numalloc = b_colidx->data[b_colidx->size[0] - 1] - 2;
  } else {
    numalloc = 0;
  }

  i = s->d->size[0];
  s->d->size[0] = numalloc + 1;
  emxEnsureCapacity_real_T(s->d, i);
  for (i = 0; i <= numalloc; i++) {
    s->d->data[i] = 0.0;
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = b_n + 1;
  emxEnsureCapacity_int32_T(s->colidx, i);
  s->colidx->data[0] = 1;
  i = s->rowidx->size[0];
  s->rowidx->size[0] = numalloc + 1;
  emxEnsureCapacity_int32_T(s->rowidx, i);
  for (i = 0; i <= numalloc; i++) {
    s->rowidx->data[i] = 0;
  }

  for (numalloc = 0; numalloc < b_n; numalloc++) {
    s->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(s);
  if (1 > b_colidx->data[b_colidx->size[0] - 1] - 1) {
    numalloc = 1;
  } else {
    numalloc = b_colidx->data[b_colidx->size[0] - 1];
  }

  for (i = 0; i <= numalloc - 2; i++) {
    s->rowidx->data[i] = b_rowidx->data[i];
  }

  i = s->colidx->size[0];
  s->colidx->size[0] = b_colidx->size[0];
  emxEnsureCapacity_int32_T(s->colidx, i);
  numalloc = b_colidx->size[0];
  for (i = 0; i < numalloc; i++) {
    s->colidx->data[i] = b_colidx->data[i];
  }

  for (numalloc = 0; numalloc <= nzs_tmp - 2; numalloc++) {
    s->d->data[numalloc] = b_d->data[numalloc];
  }

  b_sparse_fillIn(s);
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                int this_n
//                coder_internal_sparse *y
// Return Type  : void
//
void sparse_transpose(const emxArray_real_T *this_d, const emxArray_int32_T
                      *this_colidx, const emxArray_int32_T *this_rowidx, int
                      this_m, int this_n, coder_internal_sparse *y)
{
  int numalloc;
  int idx;
  emxArray_int32_T *counts;
  int outridx_tmp;
  int outridx;
  y->m = this_n;
  y->n = this_m;
  if (this_colidx->data[this_colidx->size[0] - 1] - 1 >= 1) {
    numalloc = this_colidx->data[this_colidx->size[0] - 1] - 2;
  } else {
    numalloc = 0;
  }

  idx = y->d->size[0];
  y->d->size[0] = numalloc + 1;
  emxEnsureCapacity_real_T(y->d, idx);
  for (idx = 0; idx <= numalloc; idx++) {
    y->d->data[idx] = 0.0;
  }

  idx = y->colidx->size[0];
  y->colidx->size[0] = this_m + 1;
  emxEnsureCapacity_int32_T(y->colidx, idx);
  y->colidx->data[0] = 1;
  idx = y->rowidx->size[0];
  y->rowidx->size[0] = numalloc + 1;
  emxEnsureCapacity_int32_T(y->rowidx, idx);
  for (idx = 0; idx <= numalloc; idx++) {
    y->rowidx->data[idx] = 0;
  }

  for (numalloc = 0; numalloc < this_m; numalloc++) {
    y->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(y);
  if ((this_m != 0) && (this_n != 0)) {
    numalloc = y->colidx->size[0];
    for (idx = 0; idx < numalloc; idx++) {
      y->colidx->data[idx] = 0;
    }

    idx = this_colidx->data[this_colidx->size[0] - 1];
    for (numalloc = 0; numalloc <= idx - 2; numalloc++) {
      y->colidx->data[this_rowidx->data[numalloc]]++;
    }

    y->colidx->data[0] = 1;
    idx = this_m + 1;
    for (numalloc = 2; numalloc <= idx; numalloc++) {
      y->colidx->data[numalloc - 1] += y->colidx->data[numalloc - 2];
    }

    emxInit_int32_T(&counts, 1);
    idx = counts->size[0];
    counts->size[0] = this_m;
    emxEnsureCapacity_int32_T(counts, idx);
    for (idx = 0; idx < this_m; idx++) {
      counts->data[idx] = 0;
    }

    for (numalloc = 0; numalloc < this_n; numalloc++) {
      for (idx = this_colidx->data[numalloc] - 1; idx + 1 < this_colidx->
           data[numalloc + 1]; idx++) {
        outridx_tmp = counts->data[this_rowidx->data[idx] - 1];
        outridx = (outridx_tmp + y->colidx->data[this_rowidx->data[idx] - 1]) -
          1;
        y->d->data[outridx] = this_d->data[idx];
        y->rowidx->data[outridx] = numalloc + 1;
        counts->data[this_rowidx->data[idx] - 1] = outridx_tmp + 1;
      }
    }

    emxFree_int32_T(&counts);
  }
}

//
// File trailer for sparse.cpp
//
// [EOF]
//
