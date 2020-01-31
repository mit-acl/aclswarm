//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sparse1.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "sparse1.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "sparse.h"
#include "introsort.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *varargin_1
//                const emxArray_real_T *varargin_2
//                const emxArray_real_T *varargin_3
//                coder_internal_sparse *y
// Return Type  : void
//
void b_sparse(const emxArray_real_T *varargin_1, const emxArray_real_T
              *varargin_2, const emxArray_real_T *varargin_3,
              coder_internal_sparse *y)
{
  emxArray_int32_T *ridxInt;
  int nc;
  int ns;
  int i13;
  int k;
  emxArray_int32_T *cidxInt;
  emxArray_int32_T *sortedIndices;
  cell_wrap_3 this_tunableEnvironment[2];
  int thism;
  emxInit_int32_T(&ridxInt, 1);
  nc = varargin_2->size[0];
  ns = varargin_1->size[0];
  i13 = ridxInt->size[0];
  ridxInt->size[0] = varargin_1->size[0];
  emxEnsureCapacity_int32_T(ridxInt, i13);
  for (k = 0; k < ns; k++) {
    ridxInt->data[k] = (int)varargin_1->data[k];
  }

  emxInit_int32_T(&cidxInt, 1);
  ns = varargin_2->size[0];
  i13 = cidxInt->size[0];
  cidxInt->size[0] = varargin_2->size[0];
  emxEnsureCapacity_int32_T(cidxInt, i13);
  for (k = 0; k < ns; k++) {
    cidxInt->data[k] = (int)varargin_2->data[k];
  }

  emxInit_int32_T(&sortedIndices, 1);
  i13 = sortedIndices->size[0];
  sortedIndices->size[0] = varargin_2->size[0];
  emxEnsureCapacity_int32_T(sortedIndices, i13);
  for (k = 0; k < nc; k++) {
    sortedIndices->data[k] = k + 1;
  }

  emxInitMatrix_cell_wrap_3(this_tunableEnvironment);
  i13 = this_tunableEnvironment[0].f1->size[0];
  this_tunableEnvironment[0].f1->size[0] = cidxInt->size[0];
  emxEnsureCapacity_int32_T(this_tunableEnvironment[0].f1, i13);
  ns = cidxInt->size[0];
  for (i13 = 0; i13 < ns; i13++) {
    this_tunableEnvironment[0].f1->data[i13] = cidxInt->data[i13];
  }

  i13 = this_tunableEnvironment[1].f1->size[0];
  this_tunableEnvironment[1].f1->size[0] = ridxInt->size[0];
  emxEnsureCapacity_int32_T(this_tunableEnvironment[1].f1, i13);
  ns = ridxInt->size[0];
  for (i13 = 0; i13 < ns; i13++) {
    this_tunableEnvironment[1].f1->data[i13] = ridxInt->data[i13];
  }

  introsort(sortedIndices, cidxInt->size[0], this_tunableEnvironment);
  permuteVector(sortedIndices, cidxInt);
  permuteVector(sortedIndices, ridxInt);
  emxFreeMatrix_cell_wrap_3(this_tunableEnvironment);
  if ((ridxInt->size[0] == 0) || (cidxInt->size[0] == 0)) {
    thism = 0;
    y->n = 0;
  } else {
    ns = ridxInt->size[0];
    thism = ridxInt->data[0];
    for (k = 2; k <= ns; k++) {
      if (thism < ridxInt->data[k - 1]) {
        thism = ridxInt->data[k - 1];
      }
    }

    y->n = cidxInt->data[cidxInt->size[0] - 1];
  }

  y->m = thism;
  if (varargin_2->size[0] >= 1) {
    ns = varargin_2->size[0];
  } else {
    ns = 1;
  }

  i13 = y->d->size[0];
  y->d->size[0] = ns;
  emxEnsureCapacity_real_T(y->d, i13);
  for (i13 = 0; i13 < ns; i13++) {
    y->d->data[i13] = 0.0;
  }

  i13 = y->colidx->size[0];
  y->colidx->size[0] = y->n + 1;
  emxEnsureCapacity_int32_T(y->colidx, i13);
  y->colidx->data[0] = 1;
  i13 = y->rowidx->size[0];
  y->rowidx->size[0] = ns;
  emxEnsureCapacity_int32_T(y->rowidx, i13);
  for (i13 = 0; i13 < ns; i13++) {
    y->rowidx->data[i13] = 0;
  }

  ns = 0;
  i13 = y->n;
  for (thism = 0; thism < i13; thism++) {
    k = 1 + thism;
    while ((ns + 1 <= nc) && (cidxInt->data[ns] == k)) {
      y->rowidx->data[ns] = ridxInt->data[ns];
      ns++;
    }

    y->colidx->data[k] = ns + 1;
  }

  emxFree_int32_T(&cidxInt);
  emxFree_int32_T(&ridxInt);
  for (k = 0; k < nc; k++) {
    y->d->data[k] = varargin_3->data[sortedIndices->data[k] - 1];
  }

  emxFree_int32_T(&sortedIndices);
  sparse_fillIn(y);
}

//
// Arguments    : const emxArray_real_T *varargin_1
//                emxArray_real_T *y_d
//                emxArray_int32_T *y_colidx
//                emxArray_int32_T *y_rowidx
//                int *y_m
//                int *y_n
// Return Type  : void
//
void c_sparse(const emxArray_real_T *varargin_1, emxArray_real_T *y_d,
              emxArray_int32_T *y_colidx, emxArray_int32_T *y_rowidx, int *y_m,
              int *y_n)
{
  int mInt;
  int nInt;
  int numalloc;
  int row;
  int ctr;
  double xrc;
  mInt = varargin_1->size[0];
  nInt = varargin_1->size[1];
  numalloc = 0;
  row = varargin_1->size[0] * varargin_1->size[1];
  for (ctr = 0; ctr < row; ctr++) {
    if (varargin_1->data[ctr] != 0.0) {
      numalloc++;
    }
  }

  *y_m = varargin_1->size[0];
  *y_n = varargin_1->size[1];
  if (numalloc >= 1) {
  } else {
    numalloc = 1;
  }

  row = y_d->size[0];
  y_d->size[0] = numalloc;
  emxEnsureCapacity_real_T(y_d, row);
  for (row = 0; row < numalloc; row++) {
    y_d->data[row] = 0.0;
  }

  row = y_colidx->size[0];
  y_colidx->size[0] = varargin_1->size[1] + 1;
  emxEnsureCapacity_int32_T(y_colidx, row);
  ctr = varargin_1->size[1];
  for (row = 0; row <= ctr; row++) {
    y_colidx->data[row] = 0;
  }

  y_colidx->data[0] = 1;
  row = y_rowidx->size[0];
  y_rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(y_rowidx, row);
  for (row = 0; row < numalloc; row++) {
    y_rowidx->data[row] = 0;
  }

  y_rowidx->data[0] = 1;
  ctr = 0;
  for (numalloc = 0; numalloc < nInt; numalloc++) {
    for (row = 0; row < mInt; row++) {
      xrc = varargin_1->data[row + varargin_1->size[0] * numalloc];
      if (xrc != 0.0) {
        y_rowidx->data[ctr] = row + 1;
        y_d->data[ctr] = xrc;
        ctr++;
      }
    }

    y_colidx->data[numalloc + 1] = ctr + 1;
  }
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                emxArray_real_T *y_d
//                emxArray_int32_T *y_colidx
//                emxArray_int32_T *y_rowidx
//                int *y_m
//                int *y_n
// Return Type  : void
//
void sparse(double varargin_1, double varargin_2, emxArray_real_T *y_d,
            emxArray_int32_T *y_colidx, emxArray_int32_T *y_rowidx, int *y_m,
            int *y_n)
{
  int i11;
  int loop_ub;
  i11 = y_d->size[0];
  y_d->size[0] = 1;
  emxEnsureCapacity_real_T(y_d, i11);
  y_d->data[0] = 0.0;
  i11 = y_colidx->size[0];
  loop_ub = (int)varargin_2;
  y_colidx->size[0] = loop_ub + 1;
  emxEnsureCapacity_int32_T(y_colidx, i11);
  for (i11 = 0; i11 <= loop_ub; i11++) {
    y_colidx->data[i11] = 1;
  }

  i11 = y_rowidx->size[0];
  y_rowidx->size[0] = 1;
  emxEnsureCapacity_int32_T(y_rowidx, i11);
  y_rowidx->data[0] = 1;
  *y_m = (int)varargin_1;
  *y_n = loop_ub;
}

//
// File trailer for sparse1.cpp
//
// [EOF]
//
