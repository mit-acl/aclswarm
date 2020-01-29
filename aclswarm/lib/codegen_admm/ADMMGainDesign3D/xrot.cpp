//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xrot.h"

// Function Definitions

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void b_xrot(int n, emxArray_real_T *x, int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  if (n >= 1) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      temp = c * x->data[ix] + s * x->data[iy];
      x->data[iy] = c * x->data[iy] - s * x->data[ix];
      x->data[ix] = temp;
      iy++;
      ix++;
    }
  }
}

//
// Arguments    : double x[4]
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void c_xrot(double x[4], int iy0, double c, double s)
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
