/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mtimes1.h
 *
 * Code generation for function 'mtimes1'
 *
 */

#ifndef MTIMES1_H
#define MTIMES1_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
extern void mtimes(const emxArray_creal_T *A, const emxArray_creal_T *B,
                   emxArray_creal_T *C);

#endif

/* End of code generation (mtimes1.h) */
