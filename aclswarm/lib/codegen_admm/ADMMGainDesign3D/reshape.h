//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: reshape.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef RESHAPE_H
#define RESHAPE_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void sparse_reshape(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, const double varargin_1[2],
  coder_internal_sparse *B);

#endif

//
// File trailer for reshape.h
//
// [EOF]
//
