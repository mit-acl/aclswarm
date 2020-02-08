/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.h
 *
 * Code generation for function 'sum'
 *
 */

#ifndef SUM_H
#define SUM_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_sum(const emxArray_real_T *x, emxArray_real_T *y);
extern void sum(const emxArray_real_T *x_d, const emxArray_int32_T *x_colidx,
                int x_m, emxArray_real_T *y_d, emxArray_int32_T *y_colidx,
                emxArray_int32_T *y_rowidx);

#endif

/* End of code generation (sum.h) */
