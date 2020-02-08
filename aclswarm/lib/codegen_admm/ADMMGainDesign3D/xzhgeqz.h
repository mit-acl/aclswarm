/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzhgeqz.h
 *
 * Code generation for function 'xzhgeqz'
 *
 */

#ifndef XZHGEQZ_H
#define XZHGEQZ_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void xzhgeqz(emxArray_creal_T *A, int ilo, int ihi, emxArray_creal_T *Z,
                    int *info, emxArray_creal_T *alpha1, emxArray_creal_T *beta1);

#endif

/* End of code generation (xzhgeqz.h) */
