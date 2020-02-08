/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diag.cpp
 *
 * Code generation for function 'diag'
 *
 */

/* Include files */
#include "diag.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "locBsearch.h"
#include "rt_nonfinite.h"
#include "sparse.h"

/* Function Definitions */
void sparse_diag(const emxArray_real_T *this_d, const emxArray_int32_T
                 *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
                 int this_n, emxArray_real_T *y_d, emxArray_int32_T *y_colidx,
                 emxArray_int32_T *y_rowidx, int *y_m)
{
  int M;
  int minval;
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

  if (M < this_colidx->data[this_colidx->size[0] - 1] - 1) {
    minval = M;
  } else {
    minval = this_colidx->data[this_colidx->size[0] - 1] - 1;
  }

  sparse_spallocLike(M, minval, y_d, y_colidx, y_rowidx, y_m, &expl_temp);
  toFill = 0;
  for (M = 0; M < this_n; M++) {
    minval = M + 1;
    sparse_locBsearch(this_rowidx, minval, this_colidx->data[minval - 1],
                      this_colidx->data[minval], &expl_temp, &found);
    if (found) {
      y_rowidx->data[toFill] = minval;
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

/* End of code generation (diag.cpp) */
