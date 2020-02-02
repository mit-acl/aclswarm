//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "xrot.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : double x[4]
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void b_xrot(double x[4], int iy0, double c, double s)
{
  int iy;
  double temp;
  iy = iy0 - 1;
  temp = c * x[0] + s * x[iy];
  x[iy] = c * x[iy] - s * x[0];
  x[0] = temp;
  iy++;
  temp = c * x[1] + s * x[iy];
  x[iy] = c * x[iy] - s * x[1];
  x[1] = temp;
}

//
// Arguments    : double x[16]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(double x[16], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
  iy++;
  ix++;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
  iy++;
  ix++;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
  iy++;
  ix++;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
}

//
// File trailer for xrot.cpp
//
// [EOF]
//
