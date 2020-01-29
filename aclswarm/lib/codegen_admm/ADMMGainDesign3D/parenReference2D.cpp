//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: parenReference2D.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "parenReference2D.h"
#include "sparse.h"

// Function Definitions

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
void parenReference2DColumns(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_maxnz)
{
  int nd;
  int colstart;
  int colNnz;
  int k;
  int i19;
  nd = this_colidx->data[1] - this_colidx->data[0];
  b_sparse_sparse(this_m, nd, s_d, s_colidx, s_rowidx, s_m, s_maxnz);
  if (nd != 0) {
    nd = 0;
    colstart = this_colidx->data[0] - 2;
    colNnz = this_colidx->data[1] - this_colidx->data[0];
    for (k = 0; k < colNnz; k++) {
      i19 = (colstart + k) + 1;
      s_d->data[nd] = this_d->data[i19];
      s_rowidx->data[nd] = this_rowidx->data[i19];
      nd++;
    }

    s_colidx->data[1] = s_colidx->data[0] + colNnz;
  }
}

//
// File trailer for parenReference2D.cpp
//
// [EOF]
//
