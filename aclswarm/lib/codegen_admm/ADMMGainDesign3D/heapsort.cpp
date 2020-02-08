/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * heapsort.cpp
 *
 * Code generation for function 'heapsort'
 *
 */

/* Include files */
#include "heapsort.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void b_heapify(emxArray_int32_T *x, int idx, int xstart, int xend);
static void heapify(emxArray_int32_T *x, int idx, int xstart, int xend, const
                    cell_wrap_3 cmp_tunableEnvironment[2]);

/* Function Definitions */
static void b_heapify(emxArray_int32_T *x, int idx, int xstart, int xend)
{
  bool changed;
  int extremumIdx;
  int leftIdx;
  bool exitg1;
  int extremum;
  int cmpIdx;
  int xcmp;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 1;
  exitg1 = false;
  while ((!exitg1) && (leftIdx < xend)) {
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
    } else {
      exitg1 = true;
    }
  }

  if (changed && (leftIdx <= xend)) {
    extremum = x->data[extremumIdx];
    cmpIdx = x->data[leftIdx - 1];
    if (x->data[extremumIdx] < cmpIdx) {
      x->data[extremumIdx] = cmpIdx;
      x->data[leftIdx - 1] = extremum;
    }
  }
}

static void heapify(emxArray_int32_T *x, int idx, int xstart, int xend, const
                    cell_wrap_3 cmp_tunableEnvironment[2])
{
  bool changed;
  int extremumIdx;
  int leftIdx;
  bool exitg1;
  int extremum;
  int i;
  int cmpIdx;
  int i1;
  int xcmp;
  bool varargout_1;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 2;
  exitg1 = false;
  while ((!exitg1) && (leftIdx + 1 < xend)) {
    changed = false;
    extremum = x->data[extremumIdx];
    cmpIdx = leftIdx;
    xcmp = x->data[leftIdx];
    i = x->data[leftIdx + 1] - 1;
    i1 = cmp_tunableEnvironment[0].f1->data[x->data[leftIdx] - 1];
    varargout_1 = ((i1 < cmp_tunableEnvironment[0].f1->data[i]) || ((i1 ==
      cmp_tunableEnvironment[0].f1->data[i]) && (cmp_tunableEnvironment[1]
      .f1->data[x->data[leftIdx] - 1] < cmp_tunableEnvironment[1].f1->data[i])));
    if (varargout_1) {
      cmpIdx = leftIdx + 1;
      xcmp = x->data[leftIdx + 1];
    }

    i = cmp_tunableEnvironment[0].f1->data[x->data[extremumIdx] - 1];
    i1 = cmp_tunableEnvironment[0].f1->data[xcmp - 1];
    varargout_1 = ((i < i1) || ((i == i1) && (cmp_tunableEnvironment[1].f1->
      data[x->data[extremumIdx] - 1] < cmp_tunableEnvironment[1].f1->data[xcmp -
      1])));
    if (varargout_1) {
      x->data[extremumIdx] = xcmp;
      x->data[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 2;
      changed = true;
    } else {
      exitg1 = true;
    }
  }

  if (changed && (leftIdx + 1 <= xend)) {
    extremum = x->data[extremumIdx];
    i = cmp_tunableEnvironment[0].f1->data[x->data[extremumIdx] - 1];
    i1 = cmp_tunableEnvironment[0].f1->data[x->data[leftIdx] - 1];
    varargout_1 = ((i < i1) || ((i == i1) && (cmp_tunableEnvironment[1].f1->
      data[x->data[extremumIdx] - 1] < cmp_tunableEnvironment[1].f1->data
      [x->data[leftIdx] - 1])));
    if (varargout_1) {
      x->data[extremumIdx] = x->data[leftIdx];
      x->data[leftIdx] = extremum;
    }
  }
}

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

/* End of code generation (heapsort.cpp) */
