//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlarf.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xzlarf.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                emxArray_real_T *C
//                int ic0
//                int ldc
//                emxArray_real_T *work
// Return Type  : void
//
void xzlarf(int m, int n, int iv0, double tau, emxArray_real_T *C, int ic0, int
            ldc, emxArray_real_T *work)
{
  int lastv;
  int lastc;
  int i;
  bool exitg2;
  int jy;
  int i55;
  int ia;
  int ix;
  int exitg1;
  double c;
  int i56;
  int ijA;
  if (tau != 0.0) {
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C->data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      i = ic0 + lastc * ldc;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C->data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    if (lastc + 1 != 0) {
      for (i = 0; i <= lastc; i++) {
        work->data[i] = 0.0;
      }

      i = 0;
      i55 = ic0 + ldc * lastc;
      for (jy = ic0; ldc < 0 ? jy >= i55 : jy <= i55; jy += ldc) {
        ix = iv0;
        c = 0.0;
        i56 = (jy + lastv) - 1;
        for (ia = jy; ia <= i56; ia++) {
          c += C->data[ia - 1] * C->data[ix - 1];
          ix++;
        }

        work->data[i] += c;
        i++;
      }
    }

    if (!(-tau == 0.0)) {
      i = ic0 - 1;
      jy = 0;
      for (ia = 0; ia <= lastc; ia++) {
        if (work->data[jy] != 0.0) {
          c = work->data[jy] * -tau;
          ix = iv0;
          i55 = i + 1;
          i56 = lastv + i;
          for (ijA = i55; ijA <= i56; ijA++) {
            C->data[ijA - 1] += C->data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        i += ldc;
      }
    }
  }
}

//
// File trailer for xzlarf.cpp
//
// [EOF]
//
