/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fillIn.cpp
 *
 * Code generation for function 'fillIn'
 *
 */

/* Include files */
#include "fillIn.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_sparse_fillIn(coder_internal_sparse *b_this)
{
  int idx;
  int i;
  int c;
  int ridx;
  int currRowIdx;
  double val;
  idx = 1;
  i = b_this->colidx->size[0];
  for (c = 0; c <= i - 2; c++) {
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

void c_sparse_fillIn(coder_internal_sparse_1 *b_this)
{
  int idx;
  int i;
  int c;
  int ridx;
  int currRowIdx;
  double val;
  idx = 1;
  i = b_this->colidx->size[0];
  for (c = 0; c <= i - 2; c++) {
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

void sparse_fillIn(coder_internal_sparse *b_this)
{
  int idx;
  int i;
  int c;
  int ridx;
  int exitg1;
  int i1;
  double val;
  int currRowIdx_tmp;
  idx = 1;
  i = b_this->colidx->size[0];
  for (c = 0; c <= i - 2; c++) {
    ridx = b_this->colidx->data[c];
    b_this->colidx->data[c] = idx;
    do {
      exitg1 = 0;
      i1 = b_this->colidx->data[c + 1];
      if (ridx < i1) {
        val = 0.0;
        currRowIdx_tmp = b_this->rowidx->data[ridx - 1];
        while ((ridx < i1) && (b_this->rowidx->data[ridx - 1] == currRowIdx_tmp))
        {
          val += b_this->d->data[ridx - 1];
          ridx++;
        }

        if (val != 0.0) {
          b_this->d->data[idx - 1] = val;
          b_this->rowidx->data[idx - 1] = currRowIdx_tmp;
          idx++;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_this->colidx->data[b_this->colidx->size[0] - 1] = idx;
}

/* End of code generation (fillIn.cpp) */
