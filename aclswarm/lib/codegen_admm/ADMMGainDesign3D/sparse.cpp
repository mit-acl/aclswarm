//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sparse.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "sparse.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "binOp.h"
#include "parenAssign2D.h"
#include "parenReference2D.h"
#include "CXSparseAPI.h"
#include "locBsearch1.h"
#include "bigProduct.h"
#include "solve_from_lu.h"
#include "makeCXSparseMatrix.h"

// Function Declarations
static void c_sparse_sparse(coder_internal_sparse *b_this);

// Function Definitions

//
// Arguments    : coder_internal_sparse *b_this
// Return Type  : void
//
static void c_sparse_sparse(coder_internal_sparse *b_this)
{
  b_this->d->size[0] = 0;
  b_this->colidx->size[0] = 0;
  b_this->rowidx->size[0] = 0;
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
void b_sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
                   *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
                   int this_n, emxArray_real_T *y)
{
  int i24;
  int loop_ub;
  int cend;
  int idx;
  i24 = y->size[0] * y->size[1];
  y->size[0] = this_m;
  y->size[1] = this_n;
  emxEnsureCapacity_real_T(y, i24);
  loop_ub = this_m * this_n;
  for (i24 = 0; i24 < loop_ub; i24++) {
    y->data[i24] = 0.0;
  }

  for (loop_ub = 0; loop_ub < this_n; loop_ub++) {
    cend = this_colidx->data[loop_ub + 1] - 1;
    i24 = this_colidx->data[loop_ub];
    for (idx = i24; idx <= cend; idx++) {
      y->data[(this_rowidx->data[idx - 1] + y->size[0] * loop_ub) - 1] =
        this_d->data[idx - 1];
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

  b_sparse_sparse(b_m, numalloc, s_d, s_colidx, s_rowidx, s_m, &aidx);
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
  int ridx;
  int overflow;
  int nz;
  b_bigProduct(this_m, this_n, &ridx, &overflow);
  nz = this_colidx->data[this_colidx->size[0] - 1];
  b_sparse_sparse(ridx, this_colidx->data[this_colidx->size[0] - 1] - 1, s_d,
                  s_colidx, s_rowidx, s_m, &overflow);
  s_colidx->data[0] = 1;
  s_colidx->data[s_colidx->size[0] - 1] = this_colidx->data[this_colidx->size[0]
    - 1];
  for (overflow = 0; overflow <= nz - 2; overflow++) {
    s_d->data[overflow] = this_d->data[overflow];
  }

  for (overflow = 0; overflow < this_n; overflow++) {
    ridx = this_colidx->data[overflow];
    nz = overflow * this_m;
    while (ridx < this_colidx->data[overflow + 1]) {
      s_rowidx->data[ridx - 1] = nz + this_rowidx->data[ridx - 1];
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
  int didx;
  int sn;
  int c;
  int aidx;
  int bidx;
  bool moreAToDo;
  bool moreBToDo;
  double val;
  getBinOpSize(b_m, b_n, &didx, &sn);
  allocEqsizeBinop(a_colidx, b_colidx, sn, didx, s_d, s_colidx, s_rowidx, s_m,
                   s_n);
  didx = 1;
  s_colidx->data[0] = 1;
  for (sn = 0; sn < *s_n; sn++) {
    c = 1 + sn;
    aidx = a_colidx->data[c - 1] - 1;
    bidx = b_colidx->data[c - 1] - 1;
    moreAToDo = (a_colidx->data[c - 1] < a_colidx->data[c]);
    moreBToDo = (b_colidx->data[c - 1] < b_colidx->data[c]);
    while (moreAToDo || moreBToDo) {
      while ((aidx + 1 < a_colidx->data[c]) && ((!moreBToDo) || (a_rowidx->
               data[aidx] < b_rowidx->data[bidx]))) {
        if (a_d->data[aidx] != 0.0) {
          s_d->data[didx - 1] = a_d->data[aidx];
          s_rowidx->data[didx - 1] = a_rowidx->data[aidx];
          didx++;
        }

        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[c]);
      while ((bidx + 1 < b_colidx->data[c]) && ((!moreAToDo) || (b_rowidx->
               data[bidx] < a_rowidx->data[aidx]))) {
        if (b_d->data[bidx] != 0.0) {
          s_d->data[didx - 1] = b_d->data[bidx];
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
      }

      while ((aidx + 1 < a_colidx->data[c]) && (bidx + 1 < b_colidx->data[c]) &&
             (a_rowidx->data[aidx] == b_rowidx->data[bidx])) {
        val = a_d->data[aidx] + b_d->data[bidx];
        if (val != 0.0) {
          s_d->data[didx - 1] = val;
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[c]);
      moreBToDo = (bidx + 1 < b_colidx->data[c]);
    }

    s_colidx->data[c] = didx;
  }
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
void b_sparse_rdivide(const emxArray_real_T *a_d, const emxArray_int32_T
                      *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, int
                      a_n, coder_internal_sparse *s)
{
  int nzs;
  int y;
  int i30;
  sparse_copy(a_colidx, a_rowidx, a_m, a_n, s);
  nzs = a_colidx->data[a_colidx->size[0] - 1];
  if (a_colidx->data[a_colidx->size[0] - 1] - 1 >= 1) {
    y = a_colidx->data[a_colidx->size[0] - 1] - 2;
  } else {
    y = 0;
  }

  i30 = s->d->size[0];
  s->d->size[0] = y + 1;
  emxEnsureCapacity_real_T(s->d, i30);
  for (i30 = 0; i30 <= y; i30++) {
    s->d->data[i30] = 0.0;
  }

  for (y = 0; y <= nzs - 2; y++) {
    s->d->data[y] = a_d->data[y];
  }

  b_sparse_fillIn(s);
}

//
// Arguments    : int m
//                int nzmaxval
//                emxArray_real_T *this_d
//                emxArray_int32_T *this_colidx
//                emxArray_int32_T *this_rowidx
//                int *this_m
//                int *this_maxnz
// Return Type  : void
//
void b_sparse_sparse(int m, int nzmaxval, emxArray_real_T *this_d,
                     emxArray_int32_T *this_colidx, emxArray_int32_T
                     *this_rowidx, int *this_m, int *this_maxnz)
{
  int numalloc;
  int i16;
  if (nzmaxval >= 1) {
    numalloc = nzmaxval;
  } else {
    numalloc = 1;
  }

  i16 = this_d->size[0];
  this_d->size[0] = numalloc;
  emxEnsureCapacity_real_T(this_d, i16);
  for (i16 = 0; i16 < numalloc; i16++) {
    this_d->data[i16] = 0.0;
  }

  i16 = this_colidx->size[0];
  this_colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(this_colidx, i16);
  this_colidx->data[0] = 1;
  i16 = this_rowidx->size[0];
  this_rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(this_rowidx, i16);
  for (i16 = 0; i16 < numalloc; i16++) {
    this_rowidx->data[i16] = 0;
  }

  this_colidx->data[0] = 1;
  this_colidx->data[1] = 1;
  *this_m = m;
  *this_maxnz = numalloc;
}

//
// Arguments    : const emxArray_boolean_T *this_d
//                const emxArray_int32_T *this_colidx
// Return Type  : bool
//
bool c_sparse_full(const emxArray_boolean_T *this_d, const emxArray_int32_T
                   *this_colidx)
{
  bool y;
  int cend;
  int i33;
  int idx;
  y = false;
  cend = this_colidx->data[1] - 1;
  i33 = this_colidx->data[0];
  for (idx = i33; idx <= cend; idx++) {
    y = this_d->data[0];
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
  int i34;
  int colNnz;
  int k;
  int cidx;
  double nt;
  int ridx;
  int idx;
  bool found;
  int i35;
  sm = varargin_1->size[1];
  sn = varargin_2->size[1];
  s_d->size[0] = 0;
  s_rowidx->size[0] = 0;
  i34 = s_colidx->size[0];
  s_colidx->size[0] = varargin_2->size[1] + 1;
  emxEnsureCapacity_int32_T(s_colidx, i34);
  colNnz = varargin_2->size[1];
  for (i34 = 0; i34 <= colNnz; i34++) {
    s_colidx->data[i34] = 0;
  }

  s_colidx->data[0] = 1;
  colNnz = 1;
  k = 0;
  for (cidx = 0; cidx < sn; cidx++) {
    nt = varargin_2->data[cidx];
    for (ridx = 0; ridx < sm; ridx++) {
      locBsearch(this_rowidx, (int)varargin_1->data[ridx], this_colidx->data
                 [(int)nt - 1], this_colidx->data[(int)nt], &idx, &found);
      if (found) {
        i34 = s_d->size[0];
        i35 = s_d->size[0];
        s_d->size[0] = i34 + 1;
        emxEnsureCapacity_real_T(s_d, i35);
        s_d->data[i34] = this_d->data[idx - 1];
        i34 = s_rowidx->size[0];
        i35 = s_rowidx->size[0];
        s_rowidx->size[0] = i34 + 1;
        emxEnsureCapacity_int32_T(s_rowidx, i35);
        s_rowidx->data[i34] = ridx + 1;
        s_d->data[k] = this_d->data[idx - 1];
        s_rowidx->data[k] = ridx + 1;
        k++;
        colNnz++;
      }
    }

    s_colidx->data[cidx + 1] = colNnz;
  }

  if (s_colidx->data[s_colidx->size[0] - 1] - 1 == 0) {
    i34 = s_rowidx->size[0];
    s_rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(s_rowidx, i34);
    s_rowidx->data[0] = 1;
    i34 = s_d->size[0];
    s_d->size[0] = 1;
    emxEnsureCapacity_real_T(s_d, i34);
    s_d->data[0] = 0.0;
  }

  *s_m = varargin_1->size[1];
  *s_n = varargin_2->size[1];
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
// Return Type  : double
//
double d_sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
                     *this_colidx)
{
  double y;
  int cend;
  int i36;
  int idx;
  y = 0.0;
  cend = this_colidx->data[1] - 1;
  i36 = this_colidx->data[0];
  for (idx = i36; idx <= cend; idx++) {
    y = this_d->data[idx - 1];
  }

  return y;
}

//
// Arguments    : const emxArray_int32_T *idx
//                emxArray_int32_T *y
// Return Type  : void
//
void permuteVector(const emxArray_int32_T *idx, emxArray_int32_T *y)
{
  emxArray_int32_T *t;
  int ny;
  int k;
  int loop_ub;
  emxInit_int32_T(&t, 1);
  ny = y->size[0];
  k = t->size[0];
  t->size[0] = y->size[0];
  emxEnsureCapacity_int32_T(t, k);
  loop_ub = y->size[0];
  for (k = 0; k < loop_ub; k++) {
    t->data[k] = y->data[k];
  }

  for (k = 0; k < ny; k++) {
    y->data[k] = t->data[idx->data[k] - 1];
  }

  emxFree_int32_T(&t);
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
  int i31;
  int loop_ub;
  int nalloc;
  emxArray_real_T *tmpd;
  i31 = s->colidx->size[0];
  s->colidx->size[0] = a_colidx->size[0];
  emxEnsureCapacity_int32_T(s->colidx, i31);
  loop_ub = a_colidx->size[0];
  for (i31 = 0; i31 < loop_ub; i31++) {
    s->colidx->data[i31] = a_colidx->data[i31];
  }

  s->m = a_m;
  if (a_colidx->data[a_colidx->size[0] - 1] - 1 >= 1) {
    nalloc = a_colidx->data[a_colidx->size[0] - 1] - 1;
  } else {
    nalloc = 1;
  }

  if (1 > nalloc) {
    loop_ub = 0;
  } else {
    loop_ub = nalloc;
  }

  i31 = s->rowidx->size[0];
  s->rowidx->size[0] = loop_ub;
  emxEnsureCapacity_int32_T(s->rowidx, i31);
  for (i31 = 0; i31 < loop_ub; i31++) {
    s->rowidx->data[i31] = a_rowidx->data[i31];
  }

  s->maxnz = nalloc;
  nalloc = a_colidx->data[a_colidx->size[0] - 1];
  if (1 > a_colidx->data[a_colidx->size[0] - 1] - 1) {
    i31 = -1;
  } else {
    i31 = a_colidx->data[a_colidx->size[0] - 1] - 2;
  }

  emxInit_real_T(&tmpd, 1);
  loop_ub = tmpd->size[0];
  tmpd->size[0] = i31 + 1;
  emxEnsureCapacity_real_T(tmpd, loop_ub);
  for (loop_ub = 0; loop_ub <= i31; loop_ub++) {
    tmpd->data[loop_ub] = std::abs(a_d->data[loop_ub]);
  }

  if (a_colidx->data[a_colidx->size[0] - 1] - 1 >= 1) {
    loop_ub = a_colidx->data[a_colidx->size[0] - 1] - 2;
  } else {
    loop_ub = 0;
  }

  i31 = s->d->size[0];
  s->d->size[0] = loop_ub + 1;
  emxEnsureCapacity_real_T(s->d, i31);
  for (i31 = 0; i31 <= loop_ub; i31++) {
    s->d->data[i31] = 0.0;
  }

  for (loop_ub = 0; loop_ub <= nalloc - 2; loop_ub++) {
    s->d->data[loop_ub] = tmpd->data[loop_ub];
  }

  emxFree_real_T(&tmpd);
  c_sparse_fillIn(s);
}

//
// Arguments    : const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                int this_n
//                coder_internal_sparse *t
// Return Type  : void
//
void sparse_copy(const emxArray_int32_T *this_colidx, const emxArray_int32_T
                 *this_rowidx, int this_m, int this_n, coder_internal_sparse *t)
{
  int nalloc;
  int loop_ub;
  c_sparse_sparse(t);
  nalloc = t->colidx->size[0];
  t->colidx->size[0] = this_colidx->size[0];
  emxEnsureCapacity_int32_T(t->colidx, nalloc);
  loop_ub = this_colidx->size[0];
  for (nalloc = 0; nalloc < loop_ub; nalloc++) {
    t->colidx->data[nalloc] = this_colidx->data[nalloc];
  }

  t->n = this_n;
  t->m = this_m;
  if (this_colidx->data[this_colidx->size[0] - 1] - 1 >= 1) {
    nalloc = this_colidx->data[this_colidx->size[0] - 1] - 1;
  } else {
    nalloc = 1;
  }

  if (1 > nalloc) {
    loop_ub = 0;
  } else {
    loop_ub = nalloc;
  }

  nalloc = t->rowidx->size[0];
  t->rowidx->size[0] = loop_ub;
  emxEnsureCapacity_int32_T(t->rowidx, nalloc);
  for (nalloc = 0; nalloc < loop_ub; nalloc++) {
    t->rowidx->data[nalloc] = this_rowidx->data[nalloc];
  }
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                emxArray_real_T *y
// Return Type  : void
//
void sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
                 *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
                 emxArray_real_T *y)
{
  int i20;
  int cend;
  int idx;
  i20 = y->size[0] * y->size[1];
  y->size[0] = this_m;
  y->size[1] = 1;
  emxEnsureCapacity_real_T(y, i20);
  for (i20 = 0; i20 < this_m; i20++) {
    y->data[i20] = 0.0;
  }

  cend = this_colidx->data[1] - 1;
  i20 = this_colidx->data[0];
  for (idx = i20; idx <= cend; idx++) {
    y->data[this_rowidx->data[idx - 1] - 1] = this_d->data[idx - 1];
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
  double tunableEnvironment_idx_0;
  int i32;
  if (a_colidx->data[a_colidx->size[0] - 1] - 1 > 0) {
    tunableEnvironment_idx_0 = a_d->data[0];
  } else {
    tunableEnvironment_idx_0 = 0.0;
  }

  i32 = s_d->size[0];
  s_d->size[0] = 1;
  emxEnsureCapacity_boolean_T(s_d, i32);
  s_d->data[0] = false;
  i32 = s_colidx->size[0];
  s_colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(s_colidx, i32);
  s_colidx->data[0] = 0;
  s_colidx->data[1] = 0;
  s_colidx->data[0] = 1;
  i32 = s_rowidx->size[0];
  s_rowidx->size[0] = 1;
  emxEnsureCapacity_int32_T(s_rowidx, i32);
  s_rowidx->data[0] = 1;
  s_colidx->data[1] = 1;
  if (tunableEnvironment_idx_0 < 0.0001) {
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
  int didx;
  int sn;
  int c;
  int aidx;
  int bidx;
  bool moreAToDo;
  bool moreBToDo;
  double val;
  getBinOpSize(b_m, b_n, &didx, &sn);
  allocEqsizeBinop(a_colidx, b_colidx, sn, didx, s_d, s_colidx, s_rowidx, s_m,
                   s_n);
  didx = 1;
  s_colidx->data[0] = 1;
  for (sn = 0; sn < *s_n; sn++) {
    c = 1 + sn;
    aidx = a_colidx->data[c - 1] - 1;
    bidx = b_colidx->data[c - 1] - 1;
    moreAToDo = (a_colidx->data[c - 1] < a_colidx->data[c]);
    moreBToDo = (b_colidx->data[c - 1] < b_colidx->data[c]);
    while (moreAToDo || moreBToDo) {
      while ((aidx + 1 < a_colidx->data[c]) && ((!moreBToDo) || (a_rowidx->
               data[aidx] < b_rowidx->data[bidx]))) {
        if (a_d->data[aidx] != 0.0) {
          s_d->data[didx - 1] = a_d->data[aidx];
          s_rowidx->data[didx - 1] = a_rowidx->data[aidx];
          didx++;
        }

        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[c]);
      while ((bidx + 1 < b_colidx->data[c]) && ((!moreAToDo) || (b_rowidx->
               data[bidx] < a_rowidx->data[aidx]))) {
        if (0.0 - b_d->data[bidx] != 0.0) {
          s_d->data[didx - 1] = 0.0 - b_d->data[bidx];
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
      }

      while ((aidx + 1 < a_colidx->data[c]) && (bidx + 1 < b_colidx->data[c]) &&
             (a_rowidx->data[aidx] == b_rowidx->data[bidx])) {
        val = a_d->data[aidx] - b_d->data[bidx];
        if (val != 0.0) {
          s_d->data[didx - 1] = val;
          s_rowidx->data[didx - 1] = b_rowidx->data[bidx];
          didx++;
        }

        bidx++;
        aidx++;
      }

      moreAToDo = (aidx + 1 < a_colidx->data[c]);
      moreBToDo = (bidx + 1 < b_colidx->data[c]);
    }

    s_colidx->data[c] = didx;
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
  coder_internal_sparse_1 b_this;
  int nzColAlloc;
  int extraSpace;
  int extraCol;
  cs_di* cxA;
  cs_dis * S;
  cs_din * N;
  int n;
  int idx;
  b_struct_T rhsIter;
  int nzRhs;
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
  d_emxInitStruct_coder_internal_(&b_this);
  if (p || (b_m == 0)) {
    y->m = A_n;
    nzColAlloc = y->d->size[0];
    y->d->size[0] = 1;
    emxEnsureCapacity_real_T(y->d, nzColAlloc);
    y->d->data[0] = 0.0;
    y->maxnz = 1;
    nzColAlloc = y->colidx->size[0];
    y->colidx->size[0] = 2;
    emxEnsureCapacity_int32_T(y->colidx, nzColAlloc);
    y->colidx->data[0] = 1;
    nzColAlloc = y->rowidx->size[0];
    y->rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(y->rowidx, nzColAlloc);
    y->rowidx->data[0] = 0;
    y->colidx->data[0] = 1;
    y->colidx->data[1] = 1;
  } else if (b_m == A_n) {
    if (A_m < A_n) {
      sparse_transpose(A_d, A_colidx, A_rowidx, A_m, A_n, in_d, in_colidx,
                       in_rowidx, &extraSpace, &extraCol);
      cxA = makeCXSparseMatrix(in_colidx->data[in_colidx->size[0] - 1] - 1,
        extraCol, extraSpace, &in_colidx->data[0], &in_rowidx->data[0],
        &in_d->data[0]);
    } else {
      cxA = makeCXSparseMatrix(A_colidx->data[A_colidx->size[0] - 1] - 1, A_n,
        A_m, &A_colidx->data[0], &A_rowidx->data[0], &A_d->data[0]);
    }

    S = cs_di_sqr(2, cxA, 0);
    N = cs_di_lu(cxA, S, 1);
    cs_di_spfree(cxA);
    if (N == NULL) {
      CXSparseAPI_iteratedQR(A_d, A_colidx, A_rowidx, A_m, A_n, b_d, b_colidx,
        b_rowidx, b_m, A_n, y);
    } else {
      sparse_spallocLike(A_n, y->d, y->colidx, y->rowidx, &y->m, &y->maxnz);
      sparse_parenReference(b_d, b_colidx, b_rowidx, b_m, b_this.d,
                            b_this.colidx, b_this.rowidx, &b_this.m,
                            &b_this.maxnz);
      sparse_full(b_this.d, b_this.colidx, b_this.rowidx, b_this.m, tmp);
      solve_from_lu_di(N, S, (double *)&tmp->data[0], b_m);
      extraCol = y->m;
      n = y->colidx->data[y->colidx->size[0] - 1];
      nzColAlloc = y->colidx->data[1] - y->colidx->data[0];
      idx = y->colidx->data[0];
      rhsIter.idx = 1;
      nzRhs = 0;
      for (extraSpace = 0; extraSpace < extraCol; extraSpace++) {
        rhsv = tmp->data[rhsIter.idx - 1];
        rhsIter.idx++;
        if (!(rhsv == 0.0)) {
          nzRhs++;
        }
      }

      if (nzColAlloc < nzRhs) {
        extraCol = nzRhs - nzColAlloc;
        extraSpace = (y->maxnz - y->colidx->data[y->colidx->size[0] - 1]) + 1;
        if (extraSpace < extraCol) {
          nzColAlloc = y->colidx->data[0] - 1;
          nzRhs = y->colidx->data[1];
          n = y->colidx->data[y->colidx->size[0] - 1] - 1;
          b_realloc(y, (y->maxnz + extraCol) - extraSpace, nzColAlloc, nzRhs, n,
                    extraCol);
        } else {
          nzColAlloc = y->colidx->data[1] + extraCol;
          nzRhs = y->colidx->data[1];
          n = y->colidx->data[y->colidx->size[0] - 1] - y->colidx->data[1];
          shiftRowidxAndData(y, nzColAlloc, nzRhs, n);
        }

        rhsIter.idx = 1;
        rhsIter.col = 1;
        rhsIter.row = 1;
        copyNonzeroValues(y, &rhsIter, idx, tmp);
        y->colidx->data[1] += extraCol;
      } else {
        d_emxCopyStruct_coder_internal_(&b_this, y);
        rhsIter.idx = 1;
        rhsIter.col = 1;
        rhsIter.row = 1;
        extraCol = copyNonzeroValues(&b_this, &rhsIter, y->colidx->data[0], tmp);
        d_emxCopyStruct_coder_internal_(y, &b_this);
        extraSpace = nzColAlloc - nzRhs;
        if (extraSpace > 0) {
          shiftRowidxAndData(y, extraCol, b_this.colidx->data[1], n -
                             b_this.colidx->data[1]);
          y->colidx->data[1] -= extraSpace;
        }
      }

      cs_di_sfree(S);
      cs_di_nfree(N);
    }
  } else {
    CXSparseAPI_iteratedQR(A_d, A_colidx, A_rowidx, A_m, A_n, b_d, b_colidx,
      b_rowidx, b_m, A_n, y);
  }

  d_emxFreeStruct_coder_internal_(&b_this);
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
//                int this_maxnz
//                const emxArray_real_T *rhs
//                coder_internal_sparse_1 *s
// Return Type  : void
//
void sparse_parenAssign(const emxArray_real_T *this_d, const emxArray_int32_T
  *this_colidx, const emxArray_int32_T *this_rowidx, int this_m, int this_maxnz,
  const emxArray_real_T *rhs, coder_internal_sparse_1 *s)
{
  bool p;
  coder_internal_sparse_1 b_this;
  int nzColAlloc;
  emxArray_real_T *b_rhs;
  b_struct_T rhsIter;
  int i21;
  int nzRhs;
  int extraSpace;
  double rhsv;
  int loop_ub;
  int extraAlloc;
  p = (rhs->size[0] == 1);
  d_emxInitStruct_coder_internal_(&b_this);
  if (p) {
    emxInit_real_T(&b_rhs, 2);
    i21 = b_rhs->size[0] * b_rhs->size[1];
    b_rhs->size[0] = 1;
    b_rhs->size[1] = rhs->size[0];
    emxEnsureCapacity_real_T(b_rhs, i21);
    loop_ub = rhs->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      b_rhs->data[i21] = rhs->data[i21];
    }

    nzColAlloc = this_colidx->data[1] - this_colidx->data[0];
    rhsIter.idx = 1;
    nzRhs = 0;
    for (extraSpace = 0; extraSpace < this_m; extraSpace++) {
      rhsv = b_rhs->data[rhsIter.idx - 1];
      rhsIter.idx++;
      if (!(rhsv == 0.0)) {
        nzRhs++;
      }
    }

    if (nzColAlloc < nzRhs) {
      extraSpace = nzRhs - nzColAlloc;
      extraAlloc = (this_maxnz - this_colidx->data[this_colidx->size[0] - 1]) +
        1;
      if (extraAlloc < extraSpace) {
        i21 = s->d->size[0];
        s->d->size[0] = this_d->size[0];
        emxEnsureCapacity_real_T(s->d, i21);
        loop_ub = this_d->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->d->data[i21] = this_d->data[i21];
        }

        i21 = s->colidx->size[0];
        s->colidx->size[0] = this_colidx->size[0];
        emxEnsureCapacity_int32_T(s->colidx, i21);
        loop_ub = this_colidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->colidx->data[i21] = this_colidx->data[i21];
        }

        i21 = s->rowidx->size[0];
        s->rowidx->size[0] = this_rowidx->size[0];
        emxEnsureCapacity_int32_T(s->rowidx, i21);
        loop_ub = this_rowidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->rowidx->data[i21] = this_rowidx->data[i21];
        }

        s->m = this_m;
        s->maxnz = this_maxnz;
        b_realloc(s, (this_maxnz + extraSpace) - extraAlloc, this_colidx->data[0]
                  - 1, this_colidx->data[1], this_colidx->data[this_colidx->
                  size[0] - 1] - 1, extraSpace);
      } else {
        i21 = s->d->size[0];
        s->d->size[0] = this_d->size[0];
        emxEnsureCapacity_real_T(s->d, i21);
        loop_ub = this_d->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->d->data[i21] = this_d->data[i21];
        }

        i21 = s->colidx->size[0];
        s->colidx->size[0] = this_colidx->size[0];
        emxEnsureCapacity_int32_T(s->colidx, i21);
        loop_ub = this_colidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->colidx->data[i21] = this_colidx->data[i21];
        }

        i21 = s->rowidx->size[0];
        s->rowidx->size[0] = this_rowidx->size[0];
        emxEnsureCapacity_int32_T(s->rowidx, i21);
        loop_ub = this_rowidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->rowidx->data[i21] = this_rowidx->data[i21];
        }

        s->m = this_m;
        s->maxnz = this_maxnz;
        shiftRowidxAndData(s, this_colidx->data[1] + extraSpace,
                           this_colidx->data[1], this_colidx->data
                           [this_colidx->size[0] - 1] - this_colidx->data[1]);
      }

      rhsIter.idx = 1;
      rhsIter.col = 1;
      rhsIter.row = 1;
      copyNonzeroValues(s, &rhsIter, this_colidx->data[0], b_rhs);
      s->colidx->data[1] += extraSpace;
    } else {
      i21 = b_this.d->size[0];
      b_this.d->size[0] = this_d->size[0];
      emxEnsureCapacity_real_T(b_this.d, i21);
      loop_ub = this_d->size[0];
      for (i21 = 0; i21 < loop_ub; i21++) {
        b_this.d->data[i21] = this_d->data[i21];
      }

      i21 = b_this.colidx->size[0];
      b_this.colidx->size[0] = this_colidx->size[0];
      emxEnsureCapacity_int32_T(b_this.colidx, i21);
      loop_ub = this_colidx->size[0];
      for (i21 = 0; i21 < loop_ub; i21++) {
        b_this.colidx->data[i21] = this_colidx->data[i21];
      }

      i21 = b_this.rowidx->size[0];
      b_this.rowidx->size[0] = this_rowidx->size[0];
      emxEnsureCapacity_int32_T(b_this.rowidx, i21);
      loop_ub = this_rowidx->size[0];
      for (i21 = 0; i21 < loop_ub; i21++) {
        b_this.rowidx->data[i21] = this_rowidx->data[i21];
      }

      b_this.m = this_m;
      b_this.maxnz = this_maxnz;
      rhsIter.idx = 1;
      rhsIter.col = 1;
      rhsIter.row = 1;
      extraAlloc = copyNonzeroValues(&b_this, &rhsIter, this_colidx->data[0],
        b_rhs);
      d_emxCopyStruct_coder_internal_(s, &b_this);
      extraSpace = nzColAlloc - nzRhs;
      if (extraSpace > 0) {
        shiftRowidxAndData(s, extraAlloc, b_this.colidx->data[1],
                           this_colidx->data[this_colidx->size[0] - 1] -
                           b_this.colidx->data[1]);
        s->colidx->data[1] -= extraSpace;
      }
    }

    emxFree_real_T(&b_rhs);
  } else {
    nzColAlloc = this_colidx->data[1] - this_colidx->data[0];
    rhsIter.idx = 1;
    nzRhs = 0;
    for (extraSpace = 0; extraSpace < this_m; extraSpace++) {
      rhsv = rhs->data[rhsIter.idx - 1];
      rhsIter.idx++;
      if (!(rhsv == 0.0)) {
        nzRhs++;
      }
    }

    if (nzColAlloc < nzRhs) {
      extraSpace = nzRhs - nzColAlloc;
      extraAlloc = (this_maxnz - this_colidx->data[this_colidx->size[0] - 1]) +
        1;
      if (extraAlloc < extraSpace) {
        i21 = s->d->size[0];
        s->d->size[0] = this_d->size[0];
        emxEnsureCapacity_real_T(s->d, i21);
        loop_ub = this_d->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->d->data[i21] = this_d->data[i21];
        }

        i21 = s->colidx->size[0];
        s->colidx->size[0] = this_colidx->size[0];
        emxEnsureCapacity_int32_T(s->colidx, i21);
        loop_ub = this_colidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->colidx->data[i21] = this_colidx->data[i21];
        }

        i21 = s->rowidx->size[0];
        s->rowidx->size[0] = this_rowidx->size[0];
        emxEnsureCapacity_int32_T(s->rowidx, i21);
        loop_ub = this_rowidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->rowidx->data[i21] = this_rowidx->data[i21];
        }

        s->m = this_m;
        s->maxnz = this_maxnz;
        b_realloc(s, (this_maxnz + extraSpace) - extraAlloc, this_colidx->data[0]
                  - 1, this_colidx->data[1], this_colidx->data[this_colidx->
                  size[0] - 1] - 1, extraSpace);
      } else {
        i21 = s->d->size[0];
        s->d->size[0] = this_d->size[0];
        emxEnsureCapacity_real_T(s->d, i21);
        loop_ub = this_d->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->d->data[i21] = this_d->data[i21];
        }

        i21 = s->colidx->size[0];
        s->colidx->size[0] = this_colidx->size[0];
        emxEnsureCapacity_int32_T(s->colidx, i21);
        loop_ub = this_colidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->colidx->data[i21] = this_colidx->data[i21];
        }

        i21 = s->rowidx->size[0];
        s->rowidx->size[0] = this_rowidx->size[0];
        emxEnsureCapacity_int32_T(s->rowidx, i21);
        loop_ub = this_rowidx->size[0];
        for (i21 = 0; i21 < loop_ub; i21++) {
          s->rowidx->data[i21] = this_rowidx->data[i21];
        }

        s->m = this_m;
        s->maxnz = this_maxnz;
        shiftRowidxAndData(s, this_colidx->data[1] + extraSpace,
                           this_colidx->data[1], this_colidx->data
                           [this_colidx->size[0] - 1] - this_colidx->data[1]);
      }

      rhsIter.idx = 1;
      rhsIter.col = 1;
      rhsIter.row = 1;
      copyNonzeroValues(s, &rhsIter, this_colidx->data[0], rhs);
      s->colidx->data[1] += extraSpace;
    } else {
      i21 = b_this.d->size[0];
      b_this.d->size[0] = this_d->size[0];
      emxEnsureCapacity_real_T(b_this.d, i21);
      loop_ub = this_d->size[0];
      for (i21 = 0; i21 < loop_ub; i21++) {
        b_this.d->data[i21] = this_d->data[i21];
      }

      i21 = b_this.colidx->size[0];
      b_this.colidx->size[0] = this_colidx->size[0];
      emxEnsureCapacity_int32_T(b_this.colidx, i21);
      loop_ub = this_colidx->size[0];
      for (i21 = 0; i21 < loop_ub; i21++) {
        b_this.colidx->data[i21] = this_colidx->data[i21];
      }

      i21 = b_this.rowidx->size[0];
      b_this.rowidx->size[0] = this_rowidx->size[0];
      emxEnsureCapacity_int32_T(b_this.rowidx, i21);
      loop_ub = this_rowidx->size[0];
      for (i21 = 0; i21 < loop_ub; i21++) {
        b_this.rowidx->data[i21] = this_rowidx->data[i21];
      }

      b_this.m = this_m;
      b_this.maxnz = this_maxnz;
      rhsIter.idx = 1;
      rhsIter.col = 1;
      rhsIter.row = 1;
      extraAlloc = copyNonzeroValues(&b_this, &rhsIter, this_colidx->data[0],
        rhs);
      d_emxCopyStruct_coder_internal_(s, &b_this);
      extraSpace = nzColAlloc - nzRhs;
      if (extraSpace > 0) {
        shiftRowidxAndData(s, extraAlloc, b_this.colidx->data[1],
                           this_colidx->data[this_colidx->size[0] - 1] -
                           b_this.colidx->data[1]);
        s->colidx->data[1] -= extraSpace;
      }
    }
  }

  d_emxFreeStruct_coder_internal_(&b_this);
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
//                int *s_maxnz
// Return Type  : void
//
void sparse_parenReference(const emxArray_real_T *this_d, const emxArray_int32_T
  *this_colidx, const emxArray_int32_T *this_rowidx, int this_m, emxArray_real_T
  *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx, int *s_m, int
  *s_maxnz)
{
  parenReference2DColumns(this_d, this_colidx, this_rowidx, this_m, s_d,
    s_colidx, s_rowidx, s_m, s_maxnz);
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

  b_sparse_sparse(b_m, numalloc, s_d, s_colidx, s_rowidx, s_m, &aidx);
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
  int nzs;
  int loop_ub;
  emxArray_real_T *tmpd;
  int i23;
  sparse_copy(a_colidx, a_rowidx, a_m, a_n, s);
  nzs = a_colidx->data[a_colidx->size[0] - 1];
  if (1 > a_colidx->data[a_colidx->size[0] - 1] - 1) {
    loop_ub = 0;
  } else {
    loop_ub = a_colidx->data[a_colidx->size[0] - 1] - 1;
  }

  emxInit_real_T(&tmpd, 1);
  i23 = tmpd->size[0];
  tmpd->size[0] = loop_ub;
  emxEnsureCapacity_real_T(tmpd, i23);
  for (i23 = 0; i23 < loop_ub; i23++) {
    tmpd->data[i23] = a_d->data[i23] / 2.0;
  }

  if (a_colidx->data[a_colidx->size[0] - 1] - 1 >= 1) {
    loop_ub = a_colidx->data[a_colidx->size[0] - 1] - 2;
  } else {
    loop_ub = 0;
  }

  i23 = s->d->size[0];
  s->d->size[0] = loop_ub + 1;
  emxEnsureCapacity_real_T(s->d, i23);
  for (i23 = 0; i23 <= loop_ub; i23++) {
    s->d->data[i23] = 0.0;
  }

  for (loop_ub = 0; loop_ub <= nzs - 2; loop_ub++) {
    s->d->data[loop_ub] = tmpd->data[loop_ub];
  }

  emxFree_real_T(&tmpd);
  b_sparse_fillIn(s);
}

//
// Arguments    : int m
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
//                int *s_maxnz
// Return Type  : void
//
void sparse_spallocLike(int m, emxArray_real_T *s_d, emxArray_int32_T *s_colidx,
  emxArray_int32_T *s_rowidx, int *s_m, int *s_maxnz)
{
  int i18;
  i18 = s_d->size[0];
  s_d->size[0] = 1;
  emxEnsureCapacity_real_T(s_d, i18);
  s_d->data[0] = 0.0;
  i18 = s_colidx->size[0];
  s_colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(s_colidx, i18);
  s_colidx->data[0] = 1;
  i18 = s_rowidx->size[0];
  s_rowidx->size[0] = 1;
  emxEnsureCapacity_int32_T(s_rowidx, i18);
  s_rowidx->data[0] = 0;
  s_colidx->data[0] = 1;
  s_colidx->data[1] = 1;
  *s_m = m;
  *s_maxnz = 1;
}

//
// Arguments    : int m
//                int n
//                int nzmaxval
//                coder_internal_sparse *b_this
// Return Type  : void
//
void sparse_sparse(int m, int n, int nzmaxval, coder_internal_sparse *b_this)
{
  int numalloc;
  int i10;
  b_this->m = m;
  b_this->n = n;
  if (nzmaxval >= 1) {
    numalloc = nzmaxval;
  } else {
    numalloc = 1;
  }

  i10 = b_this->d->size[0];
  b_this->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(b_this->d, i10);
  for (i10 = 0; i10 < numalloc; i10++) {
    b_this->d->data[i10] = 0.0;
  }

  i10 = b_this->colidx->size[0];
  b_this->colidx->size[0] = n + 1;
  emxEnsureCapacity_int32_T(b_this->colidx, i10);
  b_this->colidx->data[0] = 1;
  i10 = b_this->rowidx->size[0];
  b_this->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(b_this->rowidx, i10);
  for (i10 = 0; i10 < numalloc; i10++) {
    b_this->rowidx->data[i10] = 0;
  }

  for (numalloc = 0; numalloc < n; numalloc++) {
    b_this->colidx->data[1 + numalloc] = 1;
  }

  sparse_fillIn(b_this);
}

//
// Arguments    : const emxArray_real_T *this_d
//                const emxArray_int32_T *this_colidx
//                const emxArray_int32_T *this_rowidx
//                int this_m
//                int this_n
//                emxArray_real_T *y_d
//                emxArray_int32_T *y_colidx
//                emxArray_int32_T *y_rowidx
//                int *y_m
//                int *y_n
// Return Type  : void
//
void sparse_transpose(const emxArray_real_T *this_d, const emxArray_int32_T
                      *this_colidx, const emxArray_int32_T *this_rowidx, int
                      this_m, int this_n, emxArray_real_T *y_d, emxArray_int32_T
                      *y_colidx, emxArray_int32_T *y_rowidx, int *y_m, int *y_n)
{
  coder_internal_sparse expl_temp;
  int idx;
  int loop_ub;
  emxArray_int32_T *counts;
  int outridx;
  c_emxInitStruct_coder_internal_(&expl_temp);
  sparse_sparse(this_n, this_m, this_colidx->data[this_colidx->size[0] - 1] - 1,
                &expl_temp);
  idx = y_d->size[0];
  y_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(y_d, idx);
  loop_ub = expl_temp.d->size[0];
  for (idx = 0; idx < loop_ub; idx++) {
    y_d->data[idx] = expl_temp.d->data[idx];
  }

  idx = y_colidx->size[0];
  y_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(y_colidx, idx);
  loop_ub = expl_temp.colidx->size[0];
  for (idx = 0; idx < loop_ub; idx++) {
    y_colidx->data[idx] = expl_temp.colidx->data[idx];
  }

  idx = y_rowidx->size[0];
  y_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(y_rowidx, idx);
  loop_ub = expl_temp.rowidx->size[0];
  for (idx = 0; idx < loop_ub; idx++) {
    y_rowidx->data[idx] = expl_temp.rowidx->data[idx];
  }

  *y_m = expl_temp.m;
  *y_n = expl_temp.n;
  c_emxFreeStruct_coder_internal_(&expl_temp);
  if ((this_m == 0) || (this_n == 0)) {
  } else {
    loop_ub = y_colidx->size[0];
    idx = y_colidx->size[0];
    y_colidx->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(y_colidx, idx);
    for (idx = 0; idx < loop_ub; idx++) {
      y_colidx->data[idx] = 0;
    }

    idx = this_colidx->data[this_colidx->size[0] - 1];
    for (loop_ub = 0; loop_ub <= idx - 2; loop_ub++) {
      y_colidx->data[this_rowidx->data[loop_ub]]++;
    }

    y_colidx->data[0] = 1;
    idx = this_m + 1;
    for (loop_ub = 2; loop_ub <= idx; loop_ub++) {
      y_colidx->data[loop_ub - 1] += y_colidx->data[loop_ub - 2];
    }

    emxInit_int32_T(&counts, 1);
    idx = counts->size[0];
    counts->size[0] = this_m;
    emxEnsureCapacity_int32_T(counts, idx);
    for (idx = 0; idx < this_m; idx++) {
      counts->data[idx] = 0;
    }

    for (loop_ub = 0; loop_ub < this_n; loop_ub++) {
      for (idx = this_colidx->data[loop_ub] - 1; idx + 1 < this_colidx->
           data[loop_ub + 1]; idx++) {
        outridx = (counts->data[this_rowidx->data[idx] - 1] + y_colidx->
                   data[this_rowidx->data[idx] - 1]) - 1;
        y_d->data[outridx] = this_d->data[idx];
        y_rowidx->data[outridx] = loop_ub + 1;
        counts->data[this_rowidx->data[idx] - 1]++;
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
