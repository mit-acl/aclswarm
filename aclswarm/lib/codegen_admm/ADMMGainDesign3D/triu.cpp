//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: triu.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "triu.h"

// Function Definitions

//
// Arguments    : emxArray_real_T *x
// Return Type  : void
//
void triu(emxArray_real_T *x)
{
  int m;
  int istart;
  int jend;
  int j;
  int i;
  m = x->size[0];
  if ((x->size[0] == 0) || (x->size[1] == 0) || (1 >= x->size[0])) {
  } else {
    istart = 2;
    if (x->size[0] - 2 < x->size[1] - 1) {
      jend = x->size[0] - 1;
    } else {
      jend = x->size[1];
    }

    for (j = 0; j < jend; j++) {
      for (i = istart; i <= m; i++) {
        x->data[(i + x->size[0] * j) - 1] = 0.0;
      }

      istart++;
    }
  }
}

//
// File trailer for triu.cpp
//
// [EOF]
//
