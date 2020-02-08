/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * binOp.cpp
 *
 * Code generation for function 'binOp'
 *
 */

/* Include files */
#include "binOp.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void allocEqsizeBinop(const emxArray_int32_T *a_colidx, const emxArray_int32_T
                      *b_colidx, int sn, int sm, coder_internal_sparse *s)
{
  int u0;
  int numalloc;
  u0 = (a_colidx->data[a_colidx->size[0] - 1] + b_colidx->data[b_colidx->size[0]
        - 1]) - 2;
  numalloc = sn * sm;
  if (u0 < numalloc) {
    numalloc = u0;
  }

  if (numalloc < 1) {
    numalloc = 1;
  }

  s->m = sm;
  s->n = sn;
  u0 = s->d->size[0];
  s->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(s->d, u0);
  for (u0 = 0; u0 < numalloc; u0++) {
    s->d->data[u0] = 0.0;
  }

  u0 = s->colidx->size[0];
  s->colidx->size[0] = sn + 1;
  emxEnsureCapacity_int32_T(s->colidx, u0);
  s->colidx->data[0] = 1;
  u0 = s->rowidx->size[0];
  s->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(s->rowidx, u0);
  for (u0 = 0; u0 < numalloc; u0++) {
    s->rowidx->data[u0] = 0;
  }

  for (u0 = 0; u0 < sn; u0++) {
    s->colidx->data[u0 + 1] = 1;
  }

  sparse_fillIn(s);
}

/* End of code generation (binOp.cpp) */
