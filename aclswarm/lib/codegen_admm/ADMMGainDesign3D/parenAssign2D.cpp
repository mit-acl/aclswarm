/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * parenAssign2D.cpp
 *
 * Code generation for function 'parenAssign2D'
 *
 */

/* Include files */
#include "parenAssign2D.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_realloc(coder_internal_sparse_1 *b_this, int numAllocRequested, int ub1,
               int lb2, int ub2, int offs)
{
  int numAlloc;
  int overflow;
  int i;
  numAlloc = b_this->m >> 16;
  overflow = numAlloc >> 16;
  if ((b_this->m & 65535) > MAX_int32_T - (numAlloc << 16)) {
    overflow++;
  }

  if (overflow == 0) {
    if (numAllocRequested <= b_this->m) {
      numAlloc = numAllocRequested;
    } else {
      numAlloc = b_this->m;
    }

    if (1 >= numAlloc) {
      numAlloc = 1;
    }
  } else if (1 >= numAllocRequested) {
    numAlloc = 1;
  } else {
    numAlloc = numAllocRequested;
  }

  i = b_this->rowidx->size[0];
  b_this->rowidx->size[0] = numAlloc;
  emxEnsureCapacity_int32_T(b_this->rowidx, i);
  for (i = 0; i < numAlloc; i++) {
    b_this->rowidx->data[i] = 0;
  }

  i = b_this->d->size[0];
  b_this->d->size[0] = numAlloc;
  emxEnsureCapacity_real_T(b_this->d, i);
  for (i = 0; i < numAlloc; i++) {
    b_this->d->data[i] = 0.0;
  }

  b_this->maxnz = numAlloc;
  for (overflow = 0; overflow < ub1; overflow++) {
    b_this->rowidx->data[overflow] = 0;
    b_this->d->data[overflow] = 0.0;
  }

  for (overflow = lb2; overflow <= ub2; overflow++) {
    i = (overflow + offs) - 1;
    b_this->rowidx->data[i] = 0;
    b_this->d->data[i] = 0.0;
  }
}

/* End of code generation (parenAssign2D.cpp) */
