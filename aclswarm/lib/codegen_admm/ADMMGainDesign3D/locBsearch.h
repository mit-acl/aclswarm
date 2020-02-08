/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * locBsearch.h
 *
 * Code generation for function 'locBsearch'
 *
 */

#ifndef LOCBSEARCH_H
#define LOCBSEARCH_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void sparse_locBsearch(const emxArray_int32_T *x, int xi, int xstart, int
  xend, int *n, bool *found);

#endif

/* End of code generation (locBsearch.h) */
