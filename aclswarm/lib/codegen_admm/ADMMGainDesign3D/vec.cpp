//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vec.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "vec.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_rtwutil.h"
#include "reshape.h"
#include "rt_nonfinite.h"
#include "sparse.h"

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
  int i;
  int siz_idx_0;
  int offset;
  int idx_tmp_tmp;
  int idx_tmp;
  int idx;
  int vk;
  varargin_1 = x_m * x_n;
  sparse_spallocLike(varargin_1, x_colidx->data[x_colidx->size[0] - 1] - 1, v_d,
                     v_colidx, v_rowidx, v_m, &expl_temp);
  if (*v_m != 0) {
    colWriteHead = 1U;
    for (expl_temp = 0; expl_temp < x_n; expl_temp++) {
      i = (x_colidx->data[expl_temp + 1] - x_colidx->data[expl_temp]) - 1;
      if (0 <= i) {
        siz_idx_0 = varargin_1;
      }

      for (offset = 0; offset <= i; offset++) {
        idx_tmp_tmp = x_colidx->data[expl_temp] + offset;
        idx_tmp = idx_tmp_tmp - 1;
        idx = x_rowidx->data[idx_tmp] + x_m * expl_temp;
        vk = div_s32(idx - 1, siz_idx_0);
        v_rowidx->data[idx_tmp] = idx - vk * siz_idx_0;
        v_d->data[idx_tmp] = x_d->data[idx_tmp];
        while (static_cast<double>((vk + 1)) >= colWriteHead) {
          v_colidx->data[static_cast<int>(colWriteHead) - 1] = idx_tmp_tmp;
          colWriteHead++;
        }
      }
    }

    if (colWriteHead > static_cast<unsigned int>(v_colidx->size[0])) {
      i = -1;
      expl_temp = 0;
    } else {
      i = static_cast<int>(colWriteHead) - 2;
      expl_temp = v_colidx->size[0];
    }

    offset = (expl_temp - i) - 1;
    for (expl_temp = 0; expl_temp < offset; expl_temp++) {
      v_colidx->data[(i + expl_temp) + 1] = x_colidx->data[x_colidx->size[0] - 1];
    }
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
