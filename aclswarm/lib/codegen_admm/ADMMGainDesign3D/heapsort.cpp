//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: heapsort.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "heapsort.h"

// Function Declarations
static void b_heapify(emxArray_int32_T *x, int idx, int xstart, int xend);
static void heapify(emxArray_int32_T *x, int idx, int xstart, int xend, const
                    cell_wrap_3 cmp_tunableEnvironment[2]);

// Function Definitions

//
// Arguments    : emxArray_int32_T *x
//                int idx
//                int xstart
//                int xend
// Return Type  : void
//
static void b_heapify(emxArray_int32_T *x, int idx, int xstart, int xend)
{
  bool changed;
  int extremumIdx;
  int leftIdx;
  int extremum;
  int cmpIdx;
  int xcmp;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 1;
  while (changed && (leftIdx < xend)) {
    changed = false;
    extremum = x->data[extremumIdx];
    cmpIdx = leftIdx - 1;
    xcmp = x->data[leftIdx - 1];
    if (x->data[leftIdx - 1] < x->data[leftIdx]) {
      cmpIdx = leftIdx;
      xcmp = x->data[leftIdx];
    }

    if (x->data[extremumIdx] < xcmp) {
      x->data[extremumIdx] = xcmp;
      x->data[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 1;
      changed = true;
    }
  }

  if (changed && (leftIdx <= xend)) {
    extremum = x->data[extremumIdx];
    if (x->data[extremumIdx] < x->data[leftIdx - 1]) {
      x->data[extremumIdx] = x->data[leftIdx - 1];
      x->data[leftIdx - 1] = extremum;
    }
  }
}

//
// Arguments    : emxArray_int32_T *x
//                int idx
//                int xstart
//                int xend
//                const cell_wrap_3 cmp_tunableEnvironment[2]
// Return Type  : void
//
static void heapify(emxArray_int32_T *x, int idx, int xstart, int xend, const
                    cell_wrap_3 cmp_tunableEnvironment[2])
{
  bool changed;
  int extremumIdx;
  int leftIdx;
  int extremum;
  bool varargout_1;
  int cmpIdx;
  int xcmp;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 2;
  while (changed && (leftIdx + 1 < xend)) {
    changed = false;
    extremum = x->data[extremumIdx];
    cmpIdx = leftIdx;
    xcmp = x->data[leftIdx];
    varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[leftIdx] - 1] <
                    cmp_tunableEnvironment[0].f1->data[x->data[leftIdx + 1] - 1])
                   || ((cmp_tunableEnvironment[0].f1->data[x->data[leftIdx] - 1]
                        == cmp_tunableEnvironment[0].f1->data[x->data[leftIdx +
                        1] - 1]) && (cmp_tunableEnvironment[1].f1->data[x->
      data[leftIdx] - 1] < cmp_tunableEnvironment[1].f1->data[x->data[leftIdx +
      1] - 1])));
    if (varargout_1) {
      cmpIdx = leftIdx + 1;
      xcmp = x->data[leftIdx + 1];
    }

    varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[extremumIdx] - 1]
                    < cmp_tunableEnvironment[0].f1->data[xcmp - 1]) ||
                   ((cmp_tunableEnvironment[0].f1->data[x->data[extremumIdx] - 1]
                     == cmp_tunableEnvironment[0].f1->data[xcmp - 1]) &&
                    (cmp_tunableEnvironment[1].f1->data[x->data[extremumIdx] - 1]
                     < cmp_tunableEnvironment[1].f1->data[xcmp - 1])));
    if (varargout_1) {
      x->data[extremumIdx] = xcmp;
      x->data[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 2;
      changed = true;
    }
  }

  if (changed && (leftIdx + 1 <= xend)) {
    extremum = x->data[extremumIdx];
    varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[extremumIdx] - 1]
                    < cmp_tunableEnvironment[0].f1->data[x->data[leftIdx] - 1]) ||
                   ((cmp_tunableEnvironment[0].f1->data[x->data[extremumIdx] - 1]
                     == cmp_tunableEnvironment[0].f1->data[x->data[leftIdx] - 1])
                    && (cmp_tunableEnvironment[1].f1->data[x->data[extremumIdx]
                        - 1] < cmp_tunableEnvironment[1].f1->data[x->
                        data[leftIdx] - 1])));
    if (varargout_1) {
      x->data[extremumIdx] = x->data[leftIdx];
      x->data[leftIdx] = extremum;
    }
  }
}

//
// Arguments    : emxArray_int32_T *x
//                int xstart
//                int xend
//                const cell_wrap_3 cmp_tunableEnvironment[2]
// Return Type  : void
//
void b_heapsort(emxArray_int32_T *x, int xstart, int xend, const cell_wrap_3
                cmp_tunableEnvironment[2])
{
  int n;
  int idx;
  int t;
  n = (xend - xstart) - 1;
  for (idx = n + 2; idx >= 1; idx--) {
    heapify(x, idx, xstart, xend, cmp_tunableEnvironment);
  }

  for (idx = 0; idx <= n; idx++) {
    t = x->data[xend - 1];
    x->data[xend - 1] = x->data[xstart - 1];
    x->data[xstart - 1] = t;
    xend--;
    heapify(x, 1, xstart, xend, cmp_tunableEnvironment);
  }
}

//
// Arguments    : emxArray_int32_T *x
//                int xstart
//                int xend
// Return Type  : void
//
void c_heapsort(emxArray_int32_T *x, int xstart, int xend)
{
  int n;
  int idx;
  int t;
  n = (xend - xstart) - 1;
  for (idx = n + 2; idx >= 1; idx--) {
    b_heapify(x, idx, xstart, xend);
  }

  for (idx = 0; idx <= n; idx++) {
    t = x->data[xend - 1];
    x->data[xend - 1] = x->data[xstart - 1];
    x->data[xstart - 1] = t;
    xend--;
    b_heapify(x, 1, xstart, xend);
  }
}

//
// File trailer for heapsort.cpp
//
// [EOF]
//
