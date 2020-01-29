//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sparse1.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef SPARSE1_H
#define SPARSE1_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_sparse(const emxArray_real_T *varargin_1, const emxArray_real_T
                     *varargin_2, const emxArray_real_T *varargin_3,
                     coder_internal_sparse *y);
extern void c_sparse(const emxArray_real_T *varargin_1, emxArray_real_T *y_d,
                     emxArray_int32_T *y_colidx, emxArray_int32_T *y_rowidx, int
                     *y_m, int *y_n);
extern void sparse(double varargin_1, double varargin_2, emxArray_real_T *y_d,
                   emxArray_int32_T *y_colidx, emxArray_int32_T *y_rowidx, int
                   *y_m, int *y_n);

#endif

//
// File trailer for sparse1.h
//
// [EOF]
//
