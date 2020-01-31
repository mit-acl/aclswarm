//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: parenAssign2D.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "parenAssign2D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "bigProduct.h"
#include <string.h>

// Type Definitions
#include <stddef.h>

// Function Definitions

//
// Arguments    : coder_internal_sparse_1 *b_this
//                int numAllocRequested
//                int ub1
//                int lb2
//                int ub2
//                int offs
// Return Type  : void
//
void b_realloc(coder_internal_sparse_1 *b_this, int numAllocRequested, int ub1,
               int lb2, int ub2, int offs)
{
  int numAlloc;
  int overflow;
  int i51;
  bigProduct(b_this->m, &numAlloc, &overflow);
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

  i51 = b_this->rowidx->size[0];
  b_this->rowidx->size[0] = numAlloc;
  emxEnsureCapacity_int32_T(b_this->rowidx, i51);
  for (i51 = 0; i51 < numAlloc; i51++) {
    b_this->rowidx->data[i51] = 0;
  }

  i51 = b_this->d->size[0];
  b_this->d->size[0] = numAlloc;
  emxEnsureCapacity_real_T(b_this->d, i51);
  for (i51 = 0; i51 < numAlloc; i51++) {
    b_this->d->data[i51] = 0.0;
  }

  b_this->maxnz = numAlloc;
  for (overflow = 0; overflow < ub1; overflow++) {
    b_this->rowidx->data[overflow] = 0;
    b_this->d->data[overflow] = 0.0;
  }

  for (overflow = lb2; overflow <= ub2; overflow++) {
    i51 = (overflow + offs) - 1;
    b_this->rowidx->data[i51] = 0;
    b_this->d->data[i51] = 0.0;
  }
}

//
// Arguments    : coder_internal_sparse_1 *b_this
//                b_struct_T *rhsIter
//                int outStart
//                const emxArray_real_T *rhs
// Return Type  : int
//
int copyNonzeroValues(coder_internal_sparse_1 *b_this, b_struct_T *rhsIter, int
                      outStart, const emxArray_real_T *rhs)
{
  int outIdx;
  int i52;
  int k;
  double rhsv;
  outIdx = outStart;
  i52 = b_this->m;
  for (k = 0; k < i52; k++) {
    rhsv = rhs->data[rhsIter->idx - 1];
    rhsIter->idx++;
    rhsIter->row++;
    if (rhsv != 0.0) {
      b_this->rowidx->data[outIdx - 1] = 1 + k;
      b_this->d->data[outIdx - 1] = rhsv;
      outIdx++;
    }
  }

  return outIdx;
}

//
// Arguments    : coder_internal_sparse_1 *b_this
//                int outstart
//                int instart
//                int nelem
// Return Type  : void
//
void shiftRowidxAndData(coder_internal_sparse_1 *b_this, int outstart, int
  instart, int nelem)
{
  if (nelem > 0) {
    memmove((void *)&b_this->rowidx->data[outstart - 1], (void *)&b_this->
            rowidx->data[instart - 1], (size_t)nelem * sizeof(int));
    memmove((void *)&b_this->d->data[outstart - 1], (void *)&b_this->d->
            data[instart - 1], (size_t)nelem * sizeof(double));
  }
}

//
// File trailer for parenAssign2D.cpp
//
// [EOF]
//
