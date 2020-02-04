//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vertcat.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "vertcat.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *varargin_1_d
//                const emxArray_int32_T *varargin_1_colidx
//                const emxArray_int32_T *varargin_1_rowidx
//                int varargin_1_m
//                int varargin_1_n
//                const emxArray_real_T *varargin_2_d
//                const emxArray_int32_T *varargin_2_colidx
//                const emxArray_int32_T *varargin_2_rowidx
//                int varargin_2_m
//                int varargin_2_n
//                coder_internal_sparse *c
// Return Type  : void
//
void sparse_vertcat(const emxArray_real_T *varargin_1_d, const emxArray_int32_T *
                    varargin_1_colidx, const emxArray_int32_T *varargin_1_rowidx,
                    int varargin_1_m, int varargin_1_n, const emxArray_real_T
                    *varargin_2_d, const emxArray_int32_T *varargin_2_colidx,
                    const emxArray_int32_T *varargin_2_rowidx, int varargin_2_m,
                    int varargin_2_n, coder_internal_sparse *c)
{
  int cnfixeddim;
  bool emptyflag_idx_0;
  bool emptyflag_idx_1;
  bool allEmpty;
  int numalloc;
  int cnvardim;
  int i;
  int cidx;
  int kpstart;
  int kpend_tmp;
  int kpend;
  int kp;
  cnfixeddim = varargin_1_n;
  if ((varargin_1_m == 0) || (varargin_1_n == 0)) {
    emptyflag_idx_0 = true;
  } else {
    emptyflag_idx_0 = false;
  }

  if ((varargin_2_m == 0) || (varargin_2_n == 0)) {
    emptyflag_idx_1 = true;
  } else {
    emptyflag_idx_1 = false;
  }

  allEmpty = (emptyflag_idx_0 && emptyflag_idx_1);
  if ((!emptyflag_idx_1) && emptyflag_idx_0) {
    cnfixeddim = varargin_2_n;
  }

  numalloc = 0;
  cnvardim = 0;
  if (allEmpty || (!emptyflag_idx_0)) {
    numalloc = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1] - 1;
    cnvardim = varargin_1_m;
  }

  if (allEmpty || (!emptyflag_idx_1)) {
    numalloc = (numalloc + varargin_2_colidx->data[varargin_2_colidx->size[0] -
                1]) - 1;
    cnvardim += varargin_2_m;
  }

  c->m = cnvardim;
  c->n = cnfixeddim;
  if (numalloc < 1) {
    numalloc = 1;
  }

  i = c->d->size[0];
  c->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(c->d, i);
  for (i = 0; i < numalloc; i++) {
    c->d->data[i] = 0.0;
  }

  i = c->colidx->size[0];
  c->colidx->size[0] = cnfixeddim + 1;
  emxEnsureCapacity_int32_T(c->colidx, i);
  c->colidx->data[0] = 1;
  i = c->rowidx->size[0];
  c->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(c->rowidx, i);
  for (i = 0; i < numalloc; i++) {
    c->rowidx->data[i] = 0;
  }

  for (numalloc = 0; numalloc < cnfixeddim; numalloc++) {
    c->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(c);
  cnvardim = -1;
  if ((varargin_1_m == 0) || (varargin_1_n == 0)) {
    emptyflag_idx_0 = true;
  } else {
    emptyflag_idx_0 = false;
  }

  if ((varargin_2_m == 0) || (varargin_2_n == 0)) {
    emptyflag_idx_1 = true;
  } else {
    emptyflag_idx_1 = false;
  }

  i = c->n;
  for (numalloc = 0; numalloc < i; numalloc++) {
    cnfixeddim = 0;
    if (!emptyflag_idx_0) {
      cidx = cnvardim;
      kpstart = varargin_1_colidx->data[numalloc];
      kpend_tmp = varargin_1_colidx->data[numalloc + 1];
      kpend = kpend_tmp - 1;
      for (kp = kpstart; kp <= kpend; kp++) {
        cidx++;
        c->rowidx->data[cidx] = varargin_1_rowidx->data[kp - 1];
        c->d->data[cidx] = varargin_1_d->data[kp - 1];
      }

      cnvardim = (cnvardim + kpend_tmp) - varargin_1_colidx->data[numalloc];
      cnfixeddim = varargin_1_m;
    }

    if (!emptyflag_idx_1) {
      cidx = cnvardim;
      kpstart = varargin_2_colidx->data[numalloc];
      kpend_tmp = varargin_2_colidx->data[numalloc + 1];
      kpend = kpend_tmp - 1;
      for (kp = kpstart; kp <= kpend; kp++) {
        cidx++;
        c->rowidx->data[cidx] = varargin_2_rowidx->data[kp - 1] + cnfixeddim;
        c->d->data[cidx] = varargin_2_d->data[kp - 1];
      }

      cnvardim = (cnvardim + kpend_tmp) - varargin_2_colidx->data[numalloc];
    }

    c->colidx->data[numalloc + 1] = cnvardim + 2;
  }
}

//
// File trailer for vertcat.cpp
//
// [EOF]
//
