/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd1.h
 *
 * Code generation for function 'svd1'
 *
 */

#ifndef SVD1_H
#define SVD1_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[],
                  int s_size[1], double V[16]);
extern void d_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[],
                  int s_size[1], double V_data[], int V_size[2]);

#endif

/* End of code generation (svd1.h) */
