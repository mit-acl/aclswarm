//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef XROT_H
#define XROT_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_xrot(int n, emxArray_real_T *x, int ix0, int iy0, double c, double
                   s);
extern void c_xrot(double x[4], int iy0, double c, double s);
extern void xrot(double x[16], int ix0, int iy0, double c, double s);

#endif

//
// File trailer for xrot.h
//
// [EOF]
//
