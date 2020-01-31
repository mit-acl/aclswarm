//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: anyNonFinite.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "anyNonFinite.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
// Return Type  : bool
//
bool anyNonFinite(const emxArray_real_T *x)
{
  bool p;
  int nx;
  int k;
  double b_x;
  nx = x->size[0] * x->size[1];
  p = true;
  for (k = 0; k < nx; k++) {
    if (p) {
      b_x = x->data[k];
      if ((!rtIsInf(b_x)) && (!rtIsNaN(b_x))) {
        p = true;
      } else {
        p = false;
      }
    } else {
      p = false;
    }
  }

  return !p;
}

//
// File trailer for anyNonFinite.cpp
//
// [EOF]
//
