//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vertcat.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef VERTCAT_H
#define VERTCAT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void sparse_vertcat(const emxArray_real_T *varargin_1_d, const
  emxArray_int32_T *varargin_1_colidx, const emxArray_int32_T *varargin_1_rowidx,
  int varargin_1_m, int varargin_1_n, const emxArray_real_T *varargin_2_d, const
  emxArray_int32_T *varargin_2_colidx, const emxArray_int32_T *varargin_2_rowidx,
  int varargin_2_m, int varargin_2_n, coder_internal_sparse *c);

#endif

//
// File trailer for vertcat.h
//
// [EOF]
//
