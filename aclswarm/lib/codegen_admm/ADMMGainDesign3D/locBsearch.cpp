/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * locBsearch.cpp
 *
 * Code generation for function 'locBsearch'
 *
 */

/* Include files */
#include "locBsearch.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void sparse_locBsearch(const emxArray_int32_T *x, int xi, int xstart, int xend,
  int *n, bool *found)
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

/* End of code generation (locBsearch.cpp) */
