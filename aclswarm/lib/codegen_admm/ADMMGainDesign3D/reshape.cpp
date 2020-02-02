//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: reshape.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "reshape.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "ADMMGainDesign3D_rtwutil.h"
#include "fillIn.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A_d
//                const emxArray_int32_T *A_colidx
//                const emxArray_int32_T *A_rowidx
//                const double varargin_1[2]
//                coder_internal_sparse *B
// Return Type  : void
//
void sparse_reshape(const emxArray_real_T *A_d, const emxArray_int32_T *A_colidx,
                    const emxArray_int32_T *A_rowidx, const double varargin_1[2],
                    coder_internal_sparse *B)
{
  int vk;
  int B_tmp;
  int numalloc;
  int i;
  int c;
  unsigned int colWriteHead;
  int startRow;
  int siz_idx_0;
  vk = static_cast<int>(varargin_1[1]);
  B_tmp = static_cast<int>(varargin_1[0]);
  B->m = B_tmp;
  B->n = vk;
  if (A_colidx->data[A_colidx->size[0] - 1] - 1 >= 1) {
    numalloc = A_colidx->data[A_colidx->size[0] - 1] - 2;
  } else {
    numalloc = 0;
  }

  i = B->d->size[0];
  B->d->size[0] = numalloc + 1;
  emxEnsureCapacity_real_T(B->d, i);
  for (i = 0; i <= numalloc; i++) {
    B->d->data[i] = 0.0;
  }

  i = B->colidx->size[0];
  B->colidx->size[0] = vk + 1;
  emxEnsureCapacity_int32_T(B->colidx, i);
  B->colidx->data[0] = 1;
  i = B->rowidx->size[0];
  B->rowidx->size[0] = numalloc + 1;
  emxEnsureCapacity_int32_T(B->rowidx, i);
  for (i = 0; i <= numalloc; i++) {
    B->rowidx->data[i] = 0;
  }

  for (c = 0; c < vk; c++) {
    B->colidx->data[c + 1] = 1;
  }

  sparse_fillIn(B);
  if ((B->m != 0) && (B->n != 0)) {
    colWriteHead = 1U;
    startRow = A_colidx->data[0] - 1;
    i = (A_colidx->data[1] - A_colidx->data[0]) - 1;
    if (0 <= i) {
      siz_idx_0 = B_tmp;
    }

    for (c = 0; c <= i; c++) {
      numalloc = startRow + c;
      vk = div_s32(A_rowidx->data[numalloc] - 1, siz_idx_0);
      B->rowidx->data[numalloc] = A_rowidx->data[numalloc] - vk * siz_idx_0;
      B->d->data[numalloc] = A_d->data[numalloc];
      while (static_cast<double>((vk + 1)) >= colWriteHead) {
        B->colidx->data[static_cast<int>(colWriteHead) - 1] = numalloc + 1;
        colWriteHead++;
      }
    }

    if (static_cast<double>(colWriteHead) > B->colidx->size[0]) {
      i = -1;
      c = 0;
    } else {
      i = static_cast<int>(colWriteHead) - 2;
      c = B->colidx->size[0];
    }

    numalloc = (c - i) - 1;
    for (c = 0; c < numalloc; c++) {
      B->colidx->data[(i + c) + 1] = A_colidx->data[A_colidx->size[0] - 1];
    }
  }
}

//
// File trailer for reshape.cpp
//
// [EOF]
//
