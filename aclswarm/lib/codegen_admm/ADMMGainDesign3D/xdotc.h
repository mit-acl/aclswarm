//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef XDOTC_H
#define XDOTC_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern double b_xdotc(int n, const double x[16], int ix0, const double y[16],
                      int iy0);
extern double xdotc(int n, const emxArray_real_T *x, int ix0, const
                    emxArray_real_T *y, int iy0);

#endif

//
// File trailer for xdotc.h
//
// [EOF]
//
