//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "xdotc.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : int n
//                const double x[16]
//                int ix0
//                const double y[16]
//                int iy0
// Return Type  : double
//
double xdotc(int n, const double x[16], int ix0, const double y[16], int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  ix = ix0;
  iy = iy0;
  d = 0.0;
  for (k = 0; k < n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

//
// File trailer for xdotc.cpp
//
// [EOF]
//
