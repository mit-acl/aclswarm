//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: speye.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "speye.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "rt_nonfinite.h"

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
  int n;
  int numalloc;
  int i;
  int c;
  if (m < 0.0) {
    t = 0.0;
  } else {
    t = m;
  }

  ndiag = static_cast<int>(t);
  n = static_cast<int>(t);
  y->m = static_cast<int>(t);
  y->n = static_cast<int>(t);
  if (static_cast<int>(t) >= 1) {
    numalloc = static_cast<int>(t);
  } else {
    numalloc = 1;
  }

  i = y->d->size[0];
  y->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(y->d, i);
  for (i = 0; i < numalloc; i++) {
    y->d->data[i] = 0.0;
  }

  i = y->colidx->size[0];
  y->colidx->size[0] = static_cast<int>(t) + 1;
  emxEnsureCapacity_int32_T(y->colidx, i);
  y->colidx->data[0] = 1;
  i = y->rowidx->size[0];
  y->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(y->rowidx, i);
  for (i = 0; i < numalloc; i++) {
    y->rowidx->data[i] = 0;
  }

  for (c = 0; c < n; c++) {
    y->colidx->data[c + 1] = 1;
  }

  sparse_fillIn(y);
  y->colidx->data[0] = 1;
  numalloc = y->d->size[0];
  for (i = 0; i < numalloc; i++) {
    y->d->data[i] = 1.0;
  }

  for (c = 2; c <= ndiag; c++) {
    y->colidx->data[c - 1] = c;
  }

  i = static_cast<int>(t) + 1;
  numalloc = static_cast<int>(t) + 1;
  for (c = i; c <= numalloc; c++) {
    y->colidx->data[c - 1] = static_cast<int>(t) + 1;
  }

  i = y->colidx->data[y->colidx->size[0] - 1];
  for (numalloc = 0; numalloc <= i - 2; numalloc++) {
    y->rowidx->data[numalloc] = numalloc + 1;
  }
}

//
// File trailer for speye.cpp
//
// [EOF]
//
