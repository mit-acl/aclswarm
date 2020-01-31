//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: speye.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "speye.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "sparse.h"

// Function Definitions

//
// Arguments    : double m
//                coder_internal_sparse *y
// Return Type  : void
//
void speye(double m, coder_internal_sparse *y)
{
  double t;
  int ndiag;
  int loop_ub;
  int i9;
  int c;
  if (m < 0.0) {
    t = 0.0;
  } else {
    t = m;
  }

  ndiag = (int)t;
  sparse_sparse((int)t, (int)t, (int)t, y);
  y->colidx->data[0] = 1;
  loop_ub = y->d->size[0];
  i9 = y->d->size[0];
  y->d->size[0] = loop_ub;
  emxEnsureCapacity_real_T(y->d, i9);
  for (i9 = 0; i9 < loop_ub; i9++) {
    y->d->data[i9] = 1.0;
  }

  for (c = 2; c <= ndiag; c++) {
    y->colidx->data[c - 1] = c;
  }

  i9 = (int)t + 1;
  loop_ub = (int)t + 1;
  for (c = i9; c <= loop_ub; c++) {
    y->colidx->data[c - 1] = (int)t + 1;
  }

  i9 = y->colidx->data[y->colidx->size[0] - 1];
  for (loop_ub = 0; loop_ub <= i9 - 2; loop_ub++) {
    y->rowidx->data[loop_ub] = loop_ub + 1;
  }
}

//
// File trailer for speye.cpp
//
// [EOF]
//
