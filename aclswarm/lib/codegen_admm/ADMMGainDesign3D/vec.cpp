//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vec.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "vec.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "reshape.h"
#include "sparse.h"
#include "ADMMGainDesign3D_rtwutil.h"

// Function Definitions

//
// VEC   Vectorize.
//     VEC(X), where X is a vector, matrix, or N-D array, returns a column vector
//     containing all of the elements of X; i.e., VEC(X)=X(:).
// Arguments    : const emxArray_real_T *x_d
//                const emxArray_int32_T *x_colidx
//                const emxArray_int32_T *x_rowidx
//                int x_m
//                int x_n
//                emxArray_real_T *v_d
//                emxArray_int32_T *v_colidx
//                emxArray_int32_T *v_rowidx
//                int *v_m
// Return Type  : void
//
void vec(const emxArray_real_T *x_d, const emxArray_int32_T *x_colidx, const
         emxArray_int32_T *x_rowidx, int x_m, int x_n, emxArray_real_T *v_d,
         emxArray_int32_T *v_colidx, emxArray_int32_T *v_rowidx, int *v_m)
{
  int varargin_1;
  int expl_temp;
  unsigned int colWriteHead;
  int startRow;
  int i15;
  int siz_idx_0;
  emxArray_int32_T *r6;
  int offset;
  int idx_tmp;
  int idx;
  int vk;
  varargin_1 = x_m * x_n;
  b_sparse_sparse(varargin_1, x_colidx->data[x_colidx->size[0] - 1] - 1, v_d,
                  v_colidx, v_rowidx, v_m, &expl_temp);
  if (*v_m != 0) {
    colWriteHead = 1U;
    for (expl_temp = 0; expl_temp < x_n; expl_temp++) {
      startRow = x_colidx->data[expl_temp] - 1;
      i15 = (x_colidx->data[expl_temp + 1] - x_colidx->data[expl_temp]) - 1;
      if (0 <= i15) {
        siz_idx_0 = varargin_1;
      }

      for (offset = 0; offset <= i15; offset++) {
        idx_tmp = startRow + offset;
        idx = x_rowidx->data[idx_tmp] + x_m * expl_temp;
        vk = div_s32(idx - 1, siz_idx_0);
        v_rowidx->data[idx_tmp] = idx - vk * siz_idx_0;
        v_d->data[idx_tmp] = x_d->data[idx_tmp];
        while ((double)(vk + 1) >= colWriteHead) {
          v_colidx->data[(int)colWriteHead - 1] = idx_tmp + 1;
          colWriteHead++;
        }
      }
    }

    if (colWriteHead > (unsigned int)v_colidx->size[0]) {
      i15 = 0;
      expl_temp = 0;
    } else {
      i15 = (int)colWriteHead - 1;
      expl_temp = v_colidx->size[0];
    }

    emxInit_int32_T(&r6, 2);
    startRow = r6->size[0] * r6->size[1];
    r6->size[0] = 1;
    offset = expl_temp - i15;
    r6->size[1] = offset;
    emxEnsureCapacity_int32_T(r6, startRow);
    for (expl_temp = 0; expl_temp < offset; expl_temp++) {
      r6->data[expl_temp] = i15 + expl_temp;
    }

    offset = r6->size[0] * r6->size[1];
    for (i15 = 0; i15 < offset; i15++) {
      v_colidx->data[r6->data[i15]] = x_colidx->data[x_colidx->size[0] - 1];
    }

    emxFree_int32_T(&r6);
  }

  //  Copyright 2005-2016 CVX Research, Inc.
  //  See the file LICENSE.txt for full copyright information.
  //  The command 'cvx_where' will show where this file is located.
}

//
// File trailer for vec.cpp
//
// [EOF]
//
