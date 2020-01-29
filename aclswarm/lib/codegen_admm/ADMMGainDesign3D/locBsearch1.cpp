//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: locBsearch1.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "locBsearch1.h"

// Function Definitions

//
// Arguments    : const emxArray_int32_T *x
//                int xi
//                int xstart
//                int xend
//                int *n
//                bool *found
// Return Type  : void
//
void locBsearch(const emxArray_int32_T *x, int xi, int xstart, int xend, int *n,
                bool *found)
{
  int high_i;
  int low_ip1;
  int mid_i;
  if (xstart < xend) {
    if (xi < x->data[xstart - 1]) {
      *n = xstart - 1;
      *found = false;
    } else {
      high_i = xend;
      *n = xstart;
      low_ip1 = xstart;
      while (high_i > low_ip1 + 1) {
        mid_i = (*n >> 1) + (high_i >> 1);
        if (((*n & 1) == 1) && ((high_i & 1) == 1)) {
          mid_i++;
        }

        if (xi >= x->data[mid_i - 1]) {
          *n = mid_i;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      *found = (x->data[*n - 1] == xi);
    }
  } else if (xstart == xend) {
    *n = xstart - 1;
    *found = false;
  } else {
    *n = 0;
    *found = false;
  }
}

//
// File trailer for locBsearch1.cpp
//
// [EOF]
//
