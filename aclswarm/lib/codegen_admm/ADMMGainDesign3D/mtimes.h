//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef MTIMES_H
#define MTIMES_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, const emxArray_real_T
  *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx,
  coder_internal_sparse_1 *c);
extern void sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, const emxArray_real_T
  *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int
  b_n, coder_internal_sparse *c);

#endif

//
// File trailer for mtimes.h
//
// [EOF]
//
