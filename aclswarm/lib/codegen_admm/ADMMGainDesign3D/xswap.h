//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xswap.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef XSWAP_H
#define XSWAP_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
extern void c_xswap(double x[4]);
extern void xswap(double x[16], int ix0, int iy0);

#endif

//
// File trailer for xswap.h
//
// [EOF]
//
