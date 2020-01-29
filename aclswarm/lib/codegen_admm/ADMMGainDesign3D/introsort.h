//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: introsort.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef INTROSORT_H
#define INTROSORT_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_introsort(emxArray_int32_T *x, int xstart, int xend);
extern void introsort(emxArray_int32_T *x, int xend, const cell_wrap_3
                      cmp_tunableEnvironment[2]);

#endif

//
// File trailer for introsort.h
//
// [EOF]
//
