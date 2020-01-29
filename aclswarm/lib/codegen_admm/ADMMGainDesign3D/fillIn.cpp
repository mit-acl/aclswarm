//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fillIn.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "fillIn.h"

// Function Definitions

//
// Arguments    : coder_internal_sparse *b_this
// Return Type  : void
//
void b_sparse_fillIn(coder_internal_sparse *b_this)
{
  int idx;
  int i49;
  int c;
  int ridx;
  int currRowIdx;
  double val;
  idx = 1;
  i49 = b_this->colidx->size[0];
  for (c = 0; c <= i49 - 2; c++) {
    ridx = b_this->colidx->data[c];
    b_this->colidx->data[c] = idx;
    while (ridx < b_this->colidx->data[c + 1]) {
      currRowIdx = b_this->rowidx->data[ridx - 1];
      val = b_this->d->data[ridx - 1];
      ridx++;
      if (val != 0.0) {
        b_this->d->data[idx - 1] = val;
        b_this->rowidx->data[idx - 1] = currRowIdx;
        idx++;
      }
    }
  }

  b_this->colidx->data[b_this->colidx->size[0] - 1] = idx;
}

//
// Arguments    : coder_internal_sparse_1 *b_this
// Return Type  : void
//
void c_sparse_fillIn(coder_internal_sparse_1 *b_this)
{
  int idx;
  int i50;
  int c;
  int ridx;
  int currRowIdx;
  double val;
  idx = 1;
  i50 = b_this->colidx->size[0];
  for (c = 0; c <= i50 - 2; c++) {
    ridx = b_this->colidx->data[c];
    b_this->colidx->data[c] = idx;
    while (ridx < b_this->colidx->data[c + 1]) {
      currRowIdx = b_this->rowidx->data[ridx - 1];
      val = b_this->d->data[ridx - 1];
      ridx++;
      if (val != 0.0) {
        b_this->d->data[idx - 1] = val;
        b_this->rowidx->data[idx - 1] = currRowIdx;
        idx++;
      }
    }
  }

  b_this->colidx->data[b_this->colidx->size[0] - 1] = idx;
}

//
// Arguments    : coder_internal_sparse *b_this
// Return Type  : void
//
void sparse_fillIn(coder_internal_sparse *b_this)
{
  int idx;
  int i45;
  int c;
  int ridx;
  double val;
  int currRowIdx;
  idx = 1;
  i45 = b_this->colidx->size[0];
  for (c = 0; c <= i45 - 2; c++) {
    ridx = b_this->colidx->data[c];
    b_this->colidx->data[c] = idx;
    while (ridx < b_this->colidx->data[c + 1]) {
      val = 0.0;
      currRowIdx = b_this->rowidx->data[ridx - 1];
      while ((ridx < b_this->colidx->data[c + 1]) && (b_this->rowidx->data[ridx
              - 1] == currRowIdx)) {
        val += b_this->d->data[ridx - 1];
        ridx++;
      }

      if (val != 0.0) {
        b_this->d->data[idx - 1] = val;
        b_this->rowidx->data[idx - 1] = currRowIdx;
        idx++;
      }
    }
  }

  b_this->colidx->data[b_this->colidx->size[0] - 1] = idx;
}

//
// File trailer for fillIn.cpp
//
// [EOF]
//
