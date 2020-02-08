/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.cpp
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_sum(const emxArray_real_T *x, emxArray_real_T *y)
{
  int vlen;
  int vstride;
  unsigned int sz_idx_0;
  int k;
  int j;
  int xoffset;
  vlen = x->size[1];
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    sz_idx_0 = static_cast<unsigned int>(x->size[0]);
    k = y->size[0];
    y->size[0] = static_cast<int>(sz_idx_0);
    emxEnsureCapacity_real_T(y, k);
    j = static_cast<int>(sz_idx_0);
    for (k = 0; k < j; k++) {
      y->data[k] = 0.0;
    }
  } else {
    vstride = x->size[0];
    k = y->size[0];
    y->size[0] = x->size[0];
    emxEnsureCapacity_real_T(y, k);
    for (j = 0; j < vstride; j++) {
      y->data[j] = x->data[j];
    }

    for (k = 2; k <= vlen; k++) {
      xoffset = (k - 1) * vstride;
      for (j = 0; j < vstride; j++) {
        y->data[j] += x->data[xoffset + j];
      }
    }
  }
}

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

    if (0 <= y_colidx->data[1] - 2) {
      y_rowidx->data[0] = 1;
    }
  }
}

/* End of code generation (sum.cpp) */
