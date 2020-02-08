/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd.h
 *
 * Code generation for function 'svd'
 *
 */

#ifndef SVD_H
#define SVD_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void c_svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *
                  S, double V_data[], int V_size[2]);
extern void svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *S,
                double V[16]);

#endif

/* End of code generation (svd.h) */
