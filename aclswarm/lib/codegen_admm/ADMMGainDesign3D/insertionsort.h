//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: insertionsort.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef INSERTIONSORT_H
#define INSERTIONSORT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_insertionsort(emxArray_int32_T *x, int xstart, int xend);
extern void insertionsort(emxArray_int32_T *x, int xstart, int xend, const
  cell_wrap_3 cmp_tunableEnvironment[2]);

#endif

//
// File trailer for insertionsort.h
//
// [EOF]
//
