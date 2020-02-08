/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * horzcat.cpp
 *
 * Code generation for function 'horzcat'
 *
 */

/* Include files */
#include "horzcat.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void sparse_horzcat(const emxArray_real_T *varargin_1_d, const emxArray_int32_T *
                    varargin_1_colidx, const emxArray_int32_T *varargin_1_rowidx,
                    int varargin_1_m, int varargin_1_n, const emxArray_real_T
                    *varargin_2_d, const emxArray_int32_T *varargin_2_colidx,
                    const emxArray_int32_T *varargin_2_rowidx, int varargin_2_m,
                    int varargin_2_n, coder_internal_sparse *c)
{
  bool isAcceptableEmpty_tmp;
  bool b_isAcceptableEmpty_tmp;
  bool allEmpty;
  int numalloc;
  int cnvardim;
  int nzCount;
  int ccolidx;
  int idx;
  c->m = varargin_1_m;
  if ((varargin_1_m == 0) || (varargin_1_n == 0)) {
    isAcceptableEmpty_tmp = true;
  } else {
    isAcceptableEmpty_tmp = false;
  }

  if ((varargin_2_m == 0) || (varargin_2_n == 0)) {
    b_isAcceptableEmpty_tmp = true;
  } else {
    b_isAcceptableEmpty_tmp = false;
  }

  allEmpty = (isAcceptableEmpty_tmp && b_isAcceptableEmpty_tmp);
  if ((!b_isAcceptableEmpty_tmp) && isAcceptableEmpty_tmp) {
    c->m = varargin_2_m;
  }

  numalloc = 0;
  cnvardim = 0;
  if (allEmpty || (!isAcceptableEmpty_tmp)) {
    numalloc = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1] - 1;
    cnvardim = varargin_1_n;
  }

  if (allEmpty || (!b_isAcceptableEmpty_tmp)) {
    numalloc = (numalloc + varargin_2_colidx->data[varargin_2_colidx->size[0] -
                1]) - 1;
    cnvardim += varargin_2_n;
  }

  c->n = cnvardim;
  if (numalloc < 1) {
    numalloc = 1;
  }

  nzCount = c->d->size[0];
  c->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(c->d, nzCount);
  for (nzCount = 0; nzCount < numalloc; nzCount++) {
    c->d->data[nzCount] = 0.0;
  }

  nzCount = c->colidx->size[0];
  c->colidx->size[0] = cnvardim + 1;
  emxEnsureCapacity_int32_T(c->colidx, nzCount);
  c->colidx->data[0] = 1;
  nzCount = c->rowidx->size[0];
  c->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(c->rowidx, nzCount);
  for (nzCount = 0; nzCount < numalloc; nzCount++) {
    c->rowidx->data[nzCount] = 0;
  }

  for (numalloc = 0; numalloc < cnvardim; numalloc++) {
    c->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(c);
  nzCount = -1;
  ccolidx = 0;
  if ((varargin_1_m != 0) && (varargin_1_n != 0)) {
    cnvardim = -1;
    numalloc = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1];
    for (idx = 0; idx <= numalloc - 2; idx++) {
      cnvardim++;
      c->rowidx->data[cnvardim] = varargin_1_rowidx->data[idx];
      c->d->data[cnvardim] = varargin_1_d->data[idx];
    }

    for (numalloc = 0; numalloc < varargin_1_n; numalloc++) {
      ccolidx++;
      c->colidx->data[ccolidx] = varargin_1_colidx->data[numalloc + 1];
    }

    nzCount = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1] - 2;
  }

  if ((varargin_2_m != 0) && (varargin_2_n != 0)) {
    cnvardim = nzCount;
    numalloc = varargin_2_colidx->data[varargin_2_colidx->size[0] - 1];
    for (idx = 0; idx <= numalloc - 2; idx++) {
      cnvardim++;
      c->rowidx->data[cnvardim] = varargin_2_rowidx->data[idx];
      c->d->data[cnvardim] = varargin_2_d->data[idx];
    }

    for (numalloc = 0; numalloc < varargin_2_n; numalloc++) {
      ccolidx++;
      c->colidx->data[ccolidx] = (varargin_2_colidx->data[numalloc + 1] +
        nzCount) + 1;
    }
  }
}

/* End of code generation (horzcat.cpp) */
