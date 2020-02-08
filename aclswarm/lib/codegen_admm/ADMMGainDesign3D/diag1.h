/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diag1.h
 *
 * Code generation for function 'diag1'
 *
 */

#ifndef DIAG1_H
#define DIAG1_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_diag(const emxArray_boolean_T *v, emxArray_boolean_T *d);
extern void c_diag(const emxArray_creal_T *v, emxArray_creal_T *d);
extern void d_diag(const emxArray_creal_T *v, emxArray_creal_T *d);
extern void diag(const emxArray_boolean_T *v, emxArray_boolean_T *d);

#endif

/* End of code generation (diag1.h) */
