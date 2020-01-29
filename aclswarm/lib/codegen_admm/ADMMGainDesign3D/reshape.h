//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: reshape.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef RESHAPE_H
#define RESHAPE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void sparse_reshape(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, const double varargin_1[2],
  emxArray_real_T *B_d, emxArray_int32_T *B_colidx, emxArray_int32_T *B_rowidx,
  int *B_m, int *B_n);

#endif

//
// File trailer for reshape.h
//
// [EOF]
//
