//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: horzcat.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef HORZCAT_H
#define HORZCAT_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void sparse_horzcat(const emxArray_real_T *varargin_1_d, const
  emxArray_int32_T *varargin_1_colidx, const emxArray_int32_T *varargin_1_rowidx,
  int varargin_1_m, int varargin_1_n, const emxArray_real_T *varargin_2_d, const
  emxArray_int32_T *varargin_2_colidx, const emxArray_int32_T *varargin_2_rowidx,
  int varargin_2_m, int varargin_2_n, emxArray_real_T *c_d, emxArray_int32_T
  *c_colidx, emxArray_int32_T *c_rowidx, int *c_m, int *c_n);

#endif

//
// File trailer for horzcat.h
//
// [EOF]
//
