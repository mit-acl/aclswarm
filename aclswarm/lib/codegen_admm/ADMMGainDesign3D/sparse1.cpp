/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sparse1.cpp
 *
 * Code generation for function 'sparse1'
 *
 */

/* Include files */
#include "sparse1.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "introsort.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_sparse(const emxArray_real_T *varargin_1, const emxArray_real_T
              *varargin_2, const emxArray_real_T *varargin_3,
              coder_internal_sparse *y)
{
  emxArray_int32_T *ridxInt;
  int nc;
  int ns;
  int i;
  int k;
  emxArray_int32_T *cidxInt;
  emxArray_int32_T *sortedIndices;
  cell_wrap_3 this_tunableEnvironment[2];
  emxArray_int32_T *t;
  int ny;
  emxInit_int32_T(&ridxInt, 1);
  nc = varargin_2->size[0];
  ns = varargin_1->size[0];
  i = ridxInt->size[0];
  ridxInt->size[0] = varargin_1->size[0];
  emxEnsureCapacity_int32_T(ridxInt, i);
  for (k = 0; k < ns; k++) {
    ridxInt->data[k] = static_cast<int>(varargin_1->data[k]);
  }

  emxInit_int32_T(&cidxInt, 1);
  ns = varargin_2->size[0];
  i = cidxInt->size[0];
  cidxInt->size[0] = varargin_2->size[0];
  emxEnsureCapacity_int32_T(cidxInt, i);
  for (k = 0; k < ns; k++) {
    cidxInt->data[k] = static_cast<int>(varargin_2->data[k]);
  }

  emxInit_int32_T(&sortedIndices, 1);
  i = sortedIndices->size[0];
  sortedIndices->size[0] = varargin_2->size[0];
  emxEnsureCapacity_int32_T(sortedIndices, i);
  for (k = 0; k < nc; k++) {
    sortedIndices->data[k] = k + 1;
  }

  emxInitMatrix_cell_wrap_3(this_tunableEnvironment);
  i = this_tunableEnvironment[0].f1->size[0];
  this_tunableEnvironment[0].f1->size[0] = cidxInt->size[0];
  emxEnsureCapacity_int32_T(this_tunableEnvironment[0].f1, i);
  ns = cidxInt->size[0];
  for (i = 0; i < ns; i++) {
    this_tunableEnvironment[0].f1->data[i] = cidxInt->data[i];
  }

  i = this_tunableEnvironment[1].f1->size[0];
  this_tunableEnvironment[1].f1->size[0] = ridxInt->size[0];
  emxEnsureCapacity_int32_T(this_tunableEnvironment[1].f1, i);
  ns = ridxInt->size[0];
  for (i = 0; i < ns; i++) {
    this_tunableEnvironment[1].f1->data[i] = ridxInt->data[i];
  }

  emxInit_int32_T(&t, 1);
  introsort(sortedIndices, cidxInt->size[0], this_tunableEnvironment);
  ny = cidxInt->size[0];
  i = t->size[0];
  t->size[0] = cidxInt->size[0];
  emxEnsureCapacity_int32_T(t, i);
  ns = cidxInt->size[0];
  emxFreeMatrix_cell_wrap_3(this_tunableEnvironment);
  for (i = 0; i < ns; i++) {
    t->data[i] = cidxInt->data[i];
  }

  for (k = 0; k < ny; k++) {
    cidxInt->data[k] = t->data[sortedIndices->data[k] - 1];
  }

  ny = ridxInt->size[0];
  i = t->size[0];
  t->size[0] = ridxInt->size[0];
  emxEnsureCapacity_int32_T(t, i);
  ns = ridxInt->size[0];
  for (i = 0; i < ns; i++) {
    t->data[i] = ridxInt->data[i];
  }

  for (k = 0; k < ny; k++) {
    ridxInt->data[k] = t->data[sortedIndices->data[k] - 1];
  }

  emxFree_int32_T(&t);
  if ((ridxInt->size[0] == 0) || (cidxInt->size[0] == 0)) {
    ny = 0;
    y->n = 0;
  } else {
    ns = ridxInt->size[0];
    ny = ridxInt->data[0];
    for (k = 2; k <= ns; k++) {
      i = ridxInt->data[k - 1];
      if (ny < i) {
        ny = i;
      }
    }

    y->n = cidxInt->data[cidxInt->size[0] - 1];
  }

  y->m = ny;
  if (varargin_2->size[0] >= 1) {
    ns = varargin_2->size[0];
  } else {
    ns = 1;
  }

  i = y->d->size[0];
  y->d->size[0] = ns;
  emxEnsureCapacity_real_T(y->d, i);
  for (i = 0; i < ns; i++) {
    y->d->data[i] = 0.0;
  }

  i = y->colidx->size[0];
  y->colidx->size[0] = y->n + 1;
  emxEnsureCapacity_int32_T(y->colidx, i);
  y->colidx->data[0] = 1;
  i = y->rowidx->size[0];
  y->rowidx->size[0] = ns;
  emxEnsureCapacity_int32_T(y->rowidx, i);
  for (i = 0; i < ns; i++) {
    y->rowidx->data[i] = 0;
  }

  ns = 0;
  i = y->n;
  for (ny = 0; ny < i; ny++) {
    k = ny + 1;
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
  if (numalloc < 1) {
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

void sparse(double varargin_1, double varargin_2, emxArray_real_T *y_d,
            emxArray_int32_T *y_colidx, emxArray_int32_T *y_rowidx, int *y_m,
            int *y_n)
{
  int y_n_tmp;
  int i;
  y_n_tmp = static_cast<int>(varargin_2);
  i = y_d->size[0];
  y_d->size[0] = 1;
  emxEnsureCapacity_real_T(y_d, i);
  y_d->data[0] = 0.0;
  i = y_colidx->size[0];
  y_colidx->size[0] = y_n_tmp + 1;
  emxEnsureCapacity_int32_T(y_colidx, i);
  for (i = 0; i <= y_n_tmp; i++) {
    y_colidx->data[i] = 1;
  }

  i = y_rowidx->size[0];
  y_rowidx->size[0] = 1;
  emxEnsureCapacity_int32_T(y_rowidx, i);
  y_rowidx->data[0] = 1;
  *y_m = static_cast<int>(varargin_1);
  *y_n = y_n_tmp;
}

/* End of code generation (sparse1.cpp) */
