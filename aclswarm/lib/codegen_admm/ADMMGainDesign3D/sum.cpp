//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sum.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "sum.h"
#include "ADMMGainDesign3D_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x_d
//                const emxArray_int32_T *x_colidx
//                int x_m
//                emxArray_real_T *y_d
//                emxArray_int32_T *y_colidx
//                emxArray_int32_T *y_rowidx
// Return Type  : void
//
void sum(const emxArray_real_T *x_d, const emxArray_int32_T *x_colidx, int x_m,
         emxArray_real_T *y_d, emxArray_int32_T *y_colidx, emxArray_int32_T
         *y_rowidx)
{
  int xstart;
  int xend;
  double r;
  int xp;
  if (x_m == 0) {
    xstart = y_colidx->size[0];
    y_colidx->size[0] = 2;
    emxEnsureCapacity_int32_T(y_colidx, xstart);
    y_colidx->data[0] = 1;
    y_colidx->data[1] = 1;
    xstart = y_d->size[0];
    y_d->size[0] = 1;
    emxEnsureCapacity_real_T(y_d, xstart);
    y_d->data[0] = 0.0;
    xstart = y_rowidx->size[0];
    y_rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(y_rowidx, xstart);
    y_rowidx->data[0] = 1;
  } else {
    xstart = y_d->size[0];
    y_d->size[0] = 1;
    emxEnsureCapacity_real_T(y_d, xstart);
    y_d->data[0] = 0.0;
    xstart = y_colidx->size[0];
    y_colidx->size[0] = 2;
    emxEnsureCapacity_int32_T(y_colidx, xstart);
    xstart = y_rowidx->size[0];
    y_rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(y_rowidx, xstart);
    y_rowidx->data[0] = 0;
    y_colidx->data[0] = 1;
    y_colidx->data[1] = 1;
    xstart = x_colidx->data[0];
    xend = x_colidx->data[1] - 1;
    r = 0.0;
    for (xp = xstart; xp <= xend; xp++) {
      r += x_d->data[xp - 1];
    }

    if (r != 0.0) {
      y_d->data[0] = r;
      y_colidx->data[1] = 2;
    }

    xstart = y_colidx->data[1];
    for (xend = 0; xend <= xstart - 2; xend++) {
      y_rowidx->data[0] = 1;
    }
  }
}

//
// File trailer for sum.cpp
//
// [EOF]
//
