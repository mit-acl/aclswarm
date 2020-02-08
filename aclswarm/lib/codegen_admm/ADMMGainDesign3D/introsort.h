/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * introsort.h
 *
 * Code generation for function 'introsort'
 *
 */

#ifndef INTROSORT_H
#define INTROSORT_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_introsort(emxArray_int32_T *x, int xstart, int xend);
extern void introsort(emxArray_int32_T *x, int xend, const cell_wrap_3
                      cmp_tunableEnvironment[2]);

#endif

/* End of code generation (introsort.h) */
