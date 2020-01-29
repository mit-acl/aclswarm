//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: binOp.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef BINOP_H
#define BINOP_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void allocEqsizeBinop(const emxArray_int32_T *a_colidx, const
  emxArray_int32_T *b_colidx, int sn, int sm, emxArray_real_T *s_d,
  emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx, int *s_m, int *s_n);
extern void getBinOpSize(int b_m, int b_n, int *m, int *n);

#endif

//
// File trailer for binOp.h
//
// [EOF]
//
