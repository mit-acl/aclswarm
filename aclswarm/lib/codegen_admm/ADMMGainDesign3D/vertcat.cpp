//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vertcat.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "vertcat.h"
#include "sparse.h"

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
  bool foundSize;
  bool isAcceptableEmpty;
  bool allEmpty;
  int cnnz;
  int cnvardim;
  int crowoffs;
  int cidx;
  int kpstart;
  int kpend;
  int kp;
  cnfixeddim = varargin_1_n;
  foundSize = false;
  if ((varargin_1_m == 0) || (varargin_1_n == 0)) {
    isAcceptableEmpty = true;
  } else {
    isAcceptableEmpty = false;
  }

  allEmpty = isAcceptableEmpty;
  if (!isAcceptableEmpty) {
    foundSize = true;
  }

  if ((varargin_2_m == 0) || (varargin_2_n == 0)) {
    isAcceptableEmpty = true;
  } else {
    isAcceptableEmpty = false;
  }

  allEmpty = (allEmpty && isAcceptableEmpty);
  if ((!isAcceptableEmpty) && (!foundSize)) {
    cnfixeddim = varargin_2_n;
  }

  cnnz = 0;
  cnvardim = 0;
  if (allEmpty || ((varargin_1_m != 0) && (varargin_1_n != 0))) {
    cnnz = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1] - 1;
    cnvardim = varargin_1_m;
  }

  if (allEmpty || ((varargin_2_m != 0) && (varargin_2_n != 0))) {
    cnnz = (cnnz + varargin_2_colidx->data[varargin_2_colidx->size[0] - 1]) - 1;
    cnvardim += varargin_2_m;
  }

  sparse_sparse(cnvardim, cnfixeddim, cnnz, c);
  cnfixeddim = -1;
  if ((varargin_1_m == 0) || (varargin_1_n == 0)) {
    foundSize = true;
  } else {
    foundSize = false;
  }

  if ((varargin_2_m == 0) || (varargin_2_n == 0)) {
    isAcceptableEmpty = true;
  } else {
    isAcceptableEmpty = false;
  }

  cnnz = c->n;
  for (cnvardim = 0; cnvardim < cnnz; cnvardim++) {
    crowoffs = 0;
    if (!foundSize) {
      cidx = cnfixeddim;
      kpstart = varargin_1_colidx->data[cnvardim];
      kpend = varargin_1_colidx->data[cnvardim + 1] - 1;
      for (kp = kpstart; kp <= kpend; kp++) {
        cidx++;
        c->rowidx->data[cidx] = varargin_1_rowidx->data[kp - 1];
        c->d->data[cidx] = varargin_1_d->data[kp - 1];
      }

      cnfixeddim = (cnfixeddim + varargin_1_colidx->data[cnvardim + 1]) -
        varargin_1_colidx->data[cnvardim];
      crowoffs = varargin_1_m;
    }

    if (!isAcceptableEmpty) {
      cidx = cnfixeddim;
      kpstart = varargin_2_colidx->data[cnvardim];
      kpend = varargin_2_colidx->data[cnvardim + 1] - 1;
      for (kp = kpstart; kp <= kpend; kp++) {
        cidx++;
        c->rowidx->data[cidx] = varargin_2_rowidx->data[kp - 1] + crowoffs;
        c->d->data[cidx] = varargin_2_d->data[kp - 1];
      }

      cnfixeddim = (cnfixeddim + varargin_2_colidx->data[cnvardim + 1]) -
        varargin_2_colidx->data[cnvardim];
    }

    c->colidx->data[cnvardim + 1] = cnfixeddim + 2;
  }
}

//
// File trailer for vertcat.cpp
//
// [EOF]
//
