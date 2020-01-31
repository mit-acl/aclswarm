//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: binOp.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "binOp.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "sparse.h"

// Function Definitions

//
// Arguments    : const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *b_colidx
//                int sn
//                int sm
//                emxArray_real_T *s_d
//                emxArray_int32_T *s_colidx
//                emxArray_int32_T *s_rowidx
//                int *s_m
//                int *s_n
// Return Type  : void
//
void allocEqsizeBinop(const emxArray_int32_T *a_colidx, const emxArray_int32_T
                      *b_colidx, int sn, int sm, emxArray_real_T *s_d,
                      emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
                      int *s_m, int *s_n)
{
  int u0;
  int numalloc;
  coder_internal_sparse expl_temp;
  u0 = (a_colidx->data[a_colidx->size[0] - 1] + b_colidx->data[b_colidx->size[0]
        - 1]) - 2;
  numalloc = sn * sm;
  if (u0 < numalloc) {
    numalloc = u0;
  }

  if (numalloc < 1) {
    numalloc = 1;
  }

  c_emxInitStruct_coder_internal_(&expl_temp);
  sparse_sparse(sm, sn, numalloc, &expl_temp);
  numalloc = s_d->size[0];
  s_d->size[0] = expl_temp.d->size[0];
  emxEnsureCapacity_real_T(s_d, numalloc);
  u0 = expl_temp.d->size[0];
  for (numalloc = 0; numalloc < u0; numalloc++) {
    s_d->data[numalloc] = expl_temp.d->data[numalloc];
  }

  numalloc = s_colidx->size[0];
  s_colidx->size[0] = expl_temp.colidx->size[0];
  emxEnsureCapacity_int32_T(s_colidx, numalloc);
  u0 = expl_temp.colidx->size[0];
  for (numalloc = 0; numalloc < u0; numalloc++) {
    s_colidx->data[numalloc] = expl_temp.colidx->data[numalloc];
  }

  numalloc = s_rowidx->size[0];
  s_rowidx->size[0] = expl_temp.rowidx->size[0];
  emxEnsureCapacity_int32_T(s_rowidx, numalloc);
  u0 = expl_temp.rowidx->size[0];
  for (numalloc = 0; numalloc < u0; numalloc++) {
    s_rowidx->data[numalloc] = expl_temp.rowidx->data[numalloc];
  }

  *s_m = expl_temp.m;
  *s_n = expl_temp.n;
  c_emxFreeStruct_coder_internal_(&expl_temp);
}

//
// Arguments    : int b_m
//                int b_n
//                int *m
//                int *n
// Return Type  : void
//
void getBinOpSize(int b_m, int b_n, int *m, int *n)
{
  *m = b_m;
  *n = b_n;
}

//
// File trailer for binOp.cpp
//
// [EOF]
//
