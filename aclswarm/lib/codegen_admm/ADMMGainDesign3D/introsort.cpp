//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: introsort.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "introsort.h"
#include "insertionsort.h"
#include "heapsort.h"

// Type Definitions
typedef struct {
  int xstart;
  int xend;
  int depth;
} struct_T;

// Function Definitions

//
// Arguments    : emxArray_int32_T *x
//                int xstart
//                int xend
// Return Type  : void
//
void b_introsort(emxArray_int32_T *x, int xstart, int xend)
{
  int nsort;
  int pmax;
  int pmin;
  bool exitg1;
  int MAXDEPTH;
  int pivot;
  struct_T frame;
  int pow2p;
  int i47;
  struct_T st_d_data[120];
  int exitg2;
  if (xstart < xend) {
    nsort = (xend - xstart) + 1;
    if (nsort <= 32) {
      b_insertionsort(x, xstart, xend);
    } else {
      pmax = 31;
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        pivot = (pmin + pmax) >> 1;
        pow2p = 1 << pivot;
        if (pow2p == nsort) {
          pmax = pivot;
          exitg1 = true;
        } else if (pow2p > nsort) {
          pmax = pivot;
        } else {
          pmin = pivot;
        }
      }

      MAXDEPTH = (pmax - 1) << 1;
      frame.xstart = xstart;
      frame.xend = xend;
      frame.depth = 0;
      nsort = MAXDEPTH << 1;
      for (i47 = 0; i47 < nsort; i47++) {
        st_d_data[i47] = frame;
      }

      st_d_data[0] = frame;
      pmax = 1;
      while (pmax > 0) {
        frame = st_d_data[pmax - 1];
        pmax--;
        i47 = frame.xend - frame.xstart;
        if (i47 + 1 <= 32) {
          b_insertionsort(x, frame.xstart, frame.xend);
        } else if (frame.depth == MAXDEPTH) {
          c_heapsort(x, frame.xstart, frame.xend);
        } else {
          nsort = (frame.xstart + ((i47 + (i47 < 0)) >> 1)) - 1;
          pow2p = frame.xstart - 1;
          if (x->data[nsort] < x->data[pow2p]) {
            pmin = x->data[pow2p];
            x->data[pow2p] = x->data[nsort];
            x->data[nsort] = pmin;
          }

          i47 = frame.xend - 1;
          if (x->data[i47] < x->data[pow2p]) {
            pmin = x->data[pow2p];
            x->data[pow2p] = x->data[i47];
            x->data[i47] = pmin;
          }

          if (x->data[i47] < x->data[nsort]) {
            pmin = x->data[nsort];
            x->data[nsort] = x->data[i47];
            x->data[i47] = pmin;
          }

          pivot = x->data[nsort];
          i47 = frame.xend - 2;
          x->data[nsort] = x->data[i47];
          x->data[i47] = pivot;
          nsort = i47;
          do {
            exitg2 = 0;
            for (pow2p++; x->data[pow2p] < pivot; pow2p++) {
            }

            for (nsort--; pivot < x->data[nsort]; nsort--) {
            }

            if (pow2p + 1 >= nsort + 1) {
              exitg2 = 1;
            } else {
              pmin = x->data[pow2p];
              x->data[pow2p] = x->data[nsort];
              x->data[nsort] = pmin;
            }
          } while (exitg2 == 0);

          x->data[i47] = x->data[pow2p];
          x->data[pow2p] = pivot;
          if (pow2p + 2 < frame.xend) {
            st_d_data[pmax].xstart = pow2p + 2;
            st_d_data[pmax].xend = frame.xend;
            st_d_data[pmax].depth = frame.depth + 1;
            pmax++;
          }

          if (frame.xstart < pow2p + 1) {
            st_d_data[pmax].xstart = frame.xstart;
            st_d_data[pmax].xend = pow2p + 1;
            st_d_data[pmax].depth = frame.depth + 1;
            pmax++;
          }
        }
      }
    }
  }
}

//
// Arguments    : emxArray_int32_T *x
//                int xend
//                const cell_wrap_3 cmp_tunableEnvironment[2]
// Return Type  : void
//
void introsort(emxArray_int32_T *x, int xend, const cell_wrap_3
               cmp_tunableEnvironment[2])
{
  int pmax;
  int pmin;
  bool exitg1;
  int MAXDEPTH;
  int pivot;
  struct_T frame;
  int pow2p;
  struct_T st_d_data[120];
  int st_n;
  int i;
  bool varargout_1;
  int t;
  int exitg2;
  int exitg3;
  if (1 < xend) {
    if (xend <= 32) {
      insertionsort(x, 1, xend, cmp_tunableEnvironment);
    } else {
      pmax = 31;
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        pivot = (pmin + pmax) >> 1;
        pow2p = 1 << pivot;
        if (pow2p == xend) {
          pmax = pivot;
          exitg1 = true;
        } else if (pow2p > xend) {
          pmax = pivot;
        } else {
          pmin = pivot;
        }
      }

      MAXDEPTH = (pmax - 1) << 1;
      frame.xstart = 1;
      frame.xend = xend;
      frame.depth = 0;
      pmax = MAXDEPTH << 1;
      for (pow2p = 0; pow2p < pmax; pow2p++) {
        st_d_data[pow2p] = frame;
      }

      st_d_data[0] = frame;
      st_n = 1;
      while (st_n > 0) {
        frame = st_d_data[st_n - 1];
        st_n--;
        pow2p = frame.xend - frame.xstart;
        if (pow2p + 1 <= 32) {
          insertionsort(x, frame.xstart, frame.xend, cmp_tunableEnvironment);
        } else if (frame.depth == MAXDEPTH) {
          b_heapsort(x, frame.xstart, frame.xend, cmp_tunableEnvironment);
        } else {
          pmin = (frame.xstart + ((pow2p + (pow2p < 0)) >> 1)) - 1;
          i = frame.xstart - 1;
          varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[pmin] - 1] <
                          cmp_tunableEnvironment[0].f1->data[x->data[i] - 1]) ||
                         ((cmp_tunableEnvironment[0].f1->data[x->data[pmin] - 1]
                           == cmp_tunableEnvironment[0].f1->data[x->data[i] - 1])
                          && (cmp_tunableEnvironment[1].f1->data[x->data[pmin] -
                              1] < cmp_tunableEnvironment[1].f1->data[x->data[i]
                              - 1])));
          if (varargout_1) {
            t = x->data[i];
            x->data[i] = x->data[pmin];
            x->data[pmin] = t;
          }

          pmax = frame.xend - 1;
          varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1] <
                          cmp_tunableEnvironment[0].f1->data[x->data[i] - 1]) ||
                         ((cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1]
                           == cmp_tunableEnvironment[0].f1->data[x->data[i] - 1])
                          && (cmp_tunableEnvironment[1].f1->data[x->data[pmax] -
                              1] < cmp_tunableEnvironment[1].f1->data[x->data[i]
                              - 1])));
          if (varargout_1) {
            t = x->data[i];
            x->data[i] = x->data[pmax];
            x->data[pmax] = t;
          }

          varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1] <
                          cmp_tunableEnvironment[0].f1->data[x->data[pmin] - 1])
                         || ((cmp_tunableEnvironment[0].f1->data[x->data[pmax] -
                              1] == cmp_tunableEnvironment[0].f1->data[x->
                              data[pmin] - 1]) && (cmp_tunableEnvironment[1].
            f1->data[x->data[pmax] - 1] < cmp_tunableEnvironment[1].f1->data
            [x->data[pmin] - 1])));
          if (varargout_1) {
            t = x->data[pmin];
            x->data[pmin] = x->data[pmax];
            x->data[pmax] = t;
          }

          pivot = x->data[pmin] - 1;
          pow2p = frame.xend - 2;
          x->data[pmin] = x->data[pow2p];
          x->data[pow2p] = pivot + 1;
          pmax = pow2p;
          do {
            exitg2 = 0;
            i++;
            do {
              exitg3 = 0;
              varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[i] - 1]
                              < cmp_tunableEnvironment[0].f1->data[pivot]) ||
                             ((cmp_tunableEnvironment[0].f1->data[x->data[i] - 1]
                               == cmp_tunableEnvironment[0].f1->data[pivot]) &&
                              (cmp_tunableEnvironment[1].f1->data[x->data[i] - 1]
                               < cmp_tunableEnvironment[1].f1->data[pivot])));
              if (varargout_1) {
                i++;
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            pmax--;
            do {
              exitg3 = 0;
              varargout_1 = ((cmp_tunableEnvironment[0].f1->data[pivot] <
                              cmp_tunableEnvironment[0].f1->data[x->data[pmax] -
                              1]) || ((cmp_tunableEnvironment[0].f1->data[pivot]
                == cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1]) &&
                (cmp_tunableEnvironment[1].f1->data[pivot] <
                 cmp_tunableEnvironment[1].f1->data[x->data[pmax] - 1])));
              if (varargout_1) {
                pmax--;
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            if (i + 1 >= pmax + 1) {
              exitg2 = 1;
            } else {
              t = x->data[i];
              x->data[i] = x->data[pmax];
              x->data[pmax] = t;
            }
          } while (exitg2 == 0);

          x->data[pow2p] = x->data[i];
          x->data[i] = pivot + 1;
          if (i + 2 < frame.xend) {
            st_d_data[st_n].xstart = i + 2;
            st_d_data[st_n].xend = frame.xend;
            st_d_data[st_n].depth = frame.depth + 1;
            st_n++;
          }

          if (frame.xstart < i + 1) {
            st_d_data[st_n].xstart = frame.xstart;
            st_d_data[st_n].xend = i + 1;
            st_d_data[st_n].depth = frame.depth + 1;
            st_n++;
          }
        }
      }
    }
  }
}

//
// File trailer for introsort.cpp
//
// [EOF]
//
