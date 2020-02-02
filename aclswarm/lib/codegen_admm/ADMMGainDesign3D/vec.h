//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vec.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef VEC_H
#define VEC_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void vec(const emxArray_real_T *x_d, const emxArray_int32_T *x_colidx,
                const emxArray_int32_T *x_rowidx, int x_m, int x_n,
                emxArray_real_T *v_d, emxArray_int32_T *v_colidx,
                emxArray_int32_T *v_rowidx, int *v_m);

#endif

//
// File trailer for vec.h
//
// [EOF]
//
