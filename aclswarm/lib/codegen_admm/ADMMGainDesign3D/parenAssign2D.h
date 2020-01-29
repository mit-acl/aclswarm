//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: parenAssign2D.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef PARENASSIGN2D_H
#define PARENASSIGN2D_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Type Definitions

// Function Declarations
extern void b_realloc(coder_internal_sparse_1 *b_this, int numAllocRequested,
                      int ub1, int lb2, int ub2, int offs);
extern int copyNonzeroValues(coder_internal_sparse_1 *b_this, b_struct_T
  *rhsIter, int outStart, const emxArray_real_T *rhs);
extern void shiftRowidxAndData(coder_internal_sparse_1 *b_this, int outstart,
  int instart, int nelem);

#endif

//
// File trailer for parenAssign2D.h
//
// [EOF]
//
