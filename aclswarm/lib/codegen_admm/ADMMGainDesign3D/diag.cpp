//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "diag.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "locBsearch1.h"
#include "sparse.h"

// Function Definitions

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
// Return Type  : void
//
void sparse_diag(const emxArray_real_T *this_d, const emxArray_int32_T
                 *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
                 int this_n, emxArray_real_T *y_d, emxArray_int32_T *y_colidx,
                 emxArray_int32_T *y_rowidx, int *y_m)
{
  int M;
  int u1;
  int expl_temp;
  int toFill;
  bool found;
  if (this_m == this_n) {
    M = this_n;
  } else if (this_n > this_m) {
    if (0 > this_n - this_m) {
      M = this_n;
    } else {
      M = this_m;
    }
  } else if (0 < this_n - this_m) {
    M = this_m;
  } else {
    M = this_n;
  }

  u1 = this_colidx->data[this_colidx->size[0] - 1] - 1;
  if (M < u1) {
    u1 = M;
  }

  b_sparse_sparse(M, u1, y_d, y_colidx, y_rowidx, y_m, &expl_temp);
  toFill = 0;
  for (M = 0; M < this_n; M++) {
    u1 = 1 + M;
    locBsearch(this_rowidx, u1, this_colidx->data[u1 - 1], this_colidx->data[u1],
               &expl_temp, &found);
    if (found) {
      y_rowidx->data[toFill] = u1;
      y_d->data[toFill] = this_d->data[expl_temp - 1];
      toFill++;
    }
  }

  M = y_colidx->size[0];
  y_colidx->size[0] = 2;
  emxEnsureCapacity_int32_T(y_colidx, M);
  y_colidx->data[0] = 1;
  y_colidx->data[1] = toFill + 1;
}

//
// File trailer for diag.cpp
//
// [EOF]
//
