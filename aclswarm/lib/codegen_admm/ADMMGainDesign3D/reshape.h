/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * reshape.h
 *
 * Code generation for function 'reshape'
 *
 */

#ifndef RESHAPE_H
#define RESHAPE_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void sparse_reshape(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, const double varargin_1[2],
  coder_internal_sparse *B);

#endif

/* End of code generation (reshape.h) */
