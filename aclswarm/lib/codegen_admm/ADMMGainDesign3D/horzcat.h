/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * horzcat.h
 *
 * Code generation for function 'horzcat'
 *
 */

#ifndef HORZCAT_H
#define HORZCAT_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void sparse_horzcat(const emxArray_real_T *varargin_1_d, const
  emxArray_int32_T *varargin_1_colidx, const emxArray_int32_T *varargin_1_rowidx,
  int varargin_1_m, int varargin_1_n, const emxArray_real_T *varargin_2_d, const
  emxArray_int32_T *varargin_2_colidx, const emxArray_int32_T *varargin_2_rowidx,
  int varargin_2_m, int varargin_2_n, coder_internal_sparse *c);

#endif

/* End of code generation (horzcat.h) */
