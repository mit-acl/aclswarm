//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: horzcat.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "horzcat.h"
#include "ADMMGainDesign3D_emxutil.h"
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
//                emxArray_real_T *c_d
//                emxArray_int32_T *c_colidx
//                emxArray_int32_T *c_rowidx
//                int *c_m
//                int *c_n
// Return Type  : void
//
void sparse_horzcat(const emxArray_real_T *varargin_1_d, const emxArray_int32_T *
                    varargin_1_colidx, const emxArray_int32_T *varargin_1_rowidx,
                    int varargin_1_m, int varargin_1_n, const emxArray_real_T
                    *varargin_2_d, const emxArray_int32_T *varargin_2_colidx,
                    const emxArray_int32_T *varargin_2_rowidx, int varargin_2_m,
                    int varargin_2_n, emxArray_real_T *c_d, emxArray_int32_T
                    *c_colidx, emxArray_int32_T *c_rowidx, int *c_m, int *c_n)
{
  int cnfixeddim;
  bool foundSize;
  bool isAcceptableEmpty;
  bool allEmpty;
  int nnzk;
  int cnvardim;
  coder_internal_sparse expl_temp;
  int nzCount;
  int ccolidx;
  cnfixeddim = varargin_1_m;
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
    cnfixeddim = varargin_2_m;
  }

  nnzk = 0;
  cnvardim = 0;
  if (allEmpty || ((varargin_1_m != 0) && (varargin_1_n != 0))) {
    nnzk = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1] - 1;
    cnvardim = varargin_1_n;
  }

  if (allEmpty || ((varargin_2_m != 0) && (varargin_2_n != 0))) {
    nnzk = (nnzk + varargin_2_colidx->data[varargin_2_colidx->size[0] - 1]) - 1;
    cnvardim += varargin_2_n;
  }

  c_emxInitStruct_coder_internal_(&expl_temp);
  sparse_sparse(cnfixeddim, cnvardim, nnzk, &expl_temp);
  nnzk = c_d->size[0];
  c_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(c_d, nnzk);
  cnfixeddim = expl_temp.d->size[0];
  for (nnzk = 0; nnzk < cnfixeddim; nnzk++) {
    c_d->data[nnzk] = expl_temp.d->data[nnzk];
  }

  nnzk = c_colidx->size[0];
  c_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(c_colidx, nnzk);
  cnfixeddim = expl_temp.colidx->size[0];
  for (nnzk = 0; nnzk < cnfixeddim; nnzk++) {
    c_colidx->data[nnzk] = expl_temp.colidx->data[nnzk];
  }

  nnzk = c_rowidx->size[0];
  c_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(c_rowidx, nnzk);
  cnfixeddim = expl_temp.rowidx->size[0];
  for (nnzk = 0; nnzk < cnfixeddim; nnzk++) {
    c_rowidx->data[nnzk] = expl_temp.rowidx->data[nnzk];
  }

  *c_m = expl_temp.m;
  *c_n = expl_temp.n;
  nzCount = -1;
  ccolidx = 0;
  c_emxFreeStruct_coder_internal_(&expl_temp);
  if ((varargin_1_m == 0) || (varargin_1_n == 0)) {
  } else {
    cnfixeddim = -1;
    nnzk = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1];
    for (cnvardim = 0; cnvardim <= nnzk - 2; cnvardim++) {
      cnfixeddim++;
      c_rowidx->data[cnfixeddim] = varargin_1_rowidx->data[cnvardim];
      c_d->data[cnfixeddim] = varargin_1_d->data[cnvardim];
    }

    for (cnfixeddim = 0; cnfixeddim < varargin_1_n; cnfixeddim++) {
      ccolidx++;
      c_colidx->data[ccolidx] = varargin_1_colidx->data[1 + cnfixeddim];
    }

    nzCount = varargin_1_colidx->data[varargin_1_colidx->size[0] - 1] - 2;
  }

  if ((varargin_2_m == 0) || (varargin_2_n == 0)) {
  } else {
    cnfixeddim = nzCount;
    nnzk = varargin_2_colidx->data[varargin_2_colidx->size[0] - 1];
    for (cnvardim = 0; cnvardim <= nnzk - 2; cnvardim++) {
      cnfixeddim++;
      c_rowidx->data[cnfixeddim] = varargin_2_rowidx->data[cnvardim];
      c_d->data[cnfixeddim] = varargin_2_d->data[cnvardim];
    }

    for (cnfixeddim = 0; cnfixeddim < varargin_2_n; cnfixeddim++) {
      ccolidx++;
      c_colidx->data[ccolidx] = (varargin_2_colidx->data[1 + cnfixeddim] +
        nzCount) + 1;
    }
  }
}

//
// File trailer for horzcat.cpp
//
// [EOF]
//
