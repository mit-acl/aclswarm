//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: reshape.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "reshape.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "sparse.h"
#include "ADMMGainDesign3D_rtwutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A_d
//                const emxArray_int32_T *A_colidx
//                const emxArray_int32_T *A_rowidx
//                const double varargin_1[2]
//                emxArray_real_T *B_d
//                emxArray_int32_T *B_colidx
//                emxArray_int32_T *B_rowidx
//                int *B_m
//                int *B_n
// Return Type  : void
//
void sparse_reshape(const emxArray_real_T *A_d, const emxArray_int32_T *A_colidx,
                    const emxArray_int32_T *A_rowidx, const double varargin_1[2],
                    emxArray_real_T *B_d, emxArray_int32_T *B_colidx,
                    emxArray_int32_T *B_rowidx, int *B_m, int *B_n)
{
  coder_internal_sparse expl_temp;
  int i22;
  int loop_ub;
  unsigned int colWriteHead;
  int startRow;
  int siz_idx_0;
  int offset;
  int b_varargin_1;
  int vk;
  emxArray_int32_T *r9;
  c_emxInitStruct_coder_internal_(&expl_temp);
  sparse_sparse((int)varargin_1[0], (int)varargin_1[1], A_colidx->data
                [A_colidx->size[0] - 1] - 1, &expl_temp);
  i22 = B_d->size[0];
  B_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(B_d, i22);
  loop_ub = expl_temp.d->size[0];
  for (i22 = 0; i22 < loop_ub; i22++) {
    B_d->data[i22] = expl_temp.d->data[i22];
  }

  i22 = B_colidx->size[0];
  B_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(B_colidx, i22);
  loop_ub = expl_temp.colidx->size[0];
  for (i22 = 0; i22 < loop_ub; i22++) {
    B_colidx->data[i22] = expl_temp.colidx->data[i22];
  }

  i22 = B_rowidx->size[0];
  B_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(B_rowidx, i22);
  loop_ub = expl_temp.rowidx->size[0];
  for (i22 = 0; i22 < loop_ub; i22++) {
    B_rowidx->data[i22] = expl_temp.rowidx->data[i22];
  }

  *B_m = expl_temp.m;
  *B_n = expl_temp.n;
  c_emxFreeStruct_coder_internal_(&expl_temp);
  if ((*B_m == 0) || (*B_n == 0)) {
  } else {
    colWriteHead = 1U;
    startRow = A_colidx->data[0] - 1;
    i22 = (A_colidx->data[1] - A_colidx->data[0]) - 1;
    if (0 <= i22) {
      siz_idx_0 = (int)varargin_1[0];
    }

    for (offset = 0; offset <= i22; offset++) {
      loop_ub = startRow + offset;
      b_varargin_1 = A_rowidx->data[loop_ub];
      vk = div_s32(b_varargin_1 - 1, siz_idx_0);
      B_rowidx->data[loop_ub] = b_varargin_1 - vk * siz_idx_0;
      B_d->data[loop_ub] = A_d->data[loop_ub];
      while ((double)(vk + 1) >= colWriteHead) {
        B_colidx->data[(int)colWriteHead - 1] = loop_ub + 1;
        colWriteHead++;
      }
    }

    if (colWriteHead > (unsigned int)B_colidx->size[0]) {
      i22 = 0;
      startRow = 0;
    } else {
      i22 = (int)colWriteHead - 1;
      startRow = B_colidx->size[0];
    }

    emxInit_int32_T(&r9, 2);
    offset = r9->size[0] * r9->size[1];
    r9->size[0] = 1;
    loop_ub = startRow - i22;
    r9->size[1] = loop_ub;
    emxEnsureCapacity_int32_T(r9, offset);
    for (startRow = 0; startRow < loop_ub; startRow++) {
      r9->data[startRow] = i22 + startRow;
    }

    loop_ub = r9->size[0] * r9->size[1];
    for (i22 = 0; i22 < loop_ub; i22++) {
      B_colidx->data[r9->data[i22]] = A_colidx->data[A_colidx->size[0] - 1];
    }

    emxFree_int32_T(&r9);
  }
}

//
// File trailer for reshape.cpp
//
// [EOF]
//
