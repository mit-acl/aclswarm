/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * introsort.cpp
 *
 * Code generation for function 'introsort'
 *
 */

/* Include files */
#include "introsort.h"
#include "ADMMGainDesign3D.h"
#include "heapsort.h"
#include "insertionsort.h"
#include "rt_nonfinite.h"

/* Type Definitions */
struct struct_T
{
  int xstart;
  int xend;
  int depth;
};

/* Function Definitions */
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
  int i;
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
      for (i = 0; i < nsort; i++) {
        st_d_data[i] = frame;
      }

      st_d_data[0] = frame;
      pmax = 1;
      while (pmax > 0) {
        frame = st_d_data[pmax - 1];
        pmax--;
        i = frame.xend - frame.xstart;
        if (i + 1 <= 32) {
          b_insertionsort(x, frame.xstart, frame.xend);
        } else if (frame.depth == MAXDEPTH) {
          c_heapsort(x, frame.xstart, frame.xend);
        } else {
          nsort = (frame.xstart + ((i + (i < 0)) >> 1)) - 1;
          pow2p = frame.xstart - 1;
          if (x->data[nsort] < x->data[pow2p]) {
            pmin = x->data[pow2p];
            x->data[pow2p] = x->data[nsort];
            x->data[nsort] = pmin;
          }

          i = frame.xend - 1;
          if (x->data[i] < x->data[pow2p]) {
            pmin = x->data[pow2p];
            x->data[pow2p] = x->data[i];
            x->data[i] = pmin;
          }

          if (x->data[i] < x->data[nsort]) {
            pmin = x->data[nsort];
            x->data[nsort] = x->data[i];
            x->data[i] = pmin;
          }

          pivot = x->data[nsort];
          i = frame.xend - 2;
          x->data[nsort] = x->data[i];
          x->data[i] = pivot;
          nsort = i;
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

          x->data[i] = x->data[pow2p];
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

void introsort(emxArray_int32_T *x, int xend, const cell_wrap_3
               cmp_tunableEnvironment[2])
{
  int pmax;
  int pmin;
  bool exitg1;
  int MAXDEPTH;
  int j;
  struct_T frame;
  int pow2p;
  int i;
  struct_T st_d_data[120];
  int b_i;
  bool varargout_1;
  int pivot;
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
        j = (pmin + pmax) >> 1;
        pow2p = 1 << j;
        if (pow2p == xend) {
          pmax = j;
          exitg1 = true;
        } else if (pow2p > xend) {
          pmax = j;
        } else {
          pmin = j;
        }
      }

      MAXDEPTH = (pmax - 1) << 1;
      frame.xstart = 1;
      frame.xend = xend;
      frame.depth = 0;
      pmax = MAXDEPTH << 1;
      for (i = 0; i < pmax; i++) {
        st_d_data[i] = frame;
      }

      st_d_data[0] = frame;
      pow2p = 1;
      while (pow2p > 0) {
        frame = st_d_data[pow2p - 1];
        pow2p--;
        i = frame.xend - frame.xstart;
        if (i + 1 <= 32) {
          insertionsort(x, frame.xstart, frame.xend, cmp_tunableEnvironment);
        } else if (frame.depth == MAXDEPTH) {
          b_heapsort(x, frame.xstart, frame.xend, cmp_tunableEnvironment);
        } else {
          pmax = (frame.xstart + ((i + (i < 0)) >> 1)) - 1;
          b_i = frame.xstart - 1;
          i = x->data[b_i] - 1;
          varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1] <
                          cmp_tunableEnvironment[0].f1->data[i]) ||
                         ((cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1]
                           == cmp_tunableEnvironment[0].f1->data[i]) &&
                          (cmp_tunableEnvironment[1].f1->data[x->data[pmax] - 1]
                           < cmp_tunableEnvironment[1].f1->data[i])));
          if (varargout_1) {
            pmin = x->data[b_i];
            x->data[b_i] = x->data[pmax];
            x->data[pmax] = pmin;
          }

          i = frame.xend - 1;
          varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[i] - 1] <
                          cmp_tunableEnvironment[0].f1->data[x->data[b_i] - 1]) ||
                         ((cmp_tunableEnvironment[0].f1->data[x->data[i] - 1] ==
                           cmp_tunableEnvironment[0].f1->data[x->data[b_i] - 1])
                          && (cmp_tunableEnvironment[1].f1->data[x->data[i] - 1]
                              < cmp_tunableEnvironment[1].f1->data[x->data[b_i]
                              - 1])));
          if (varargout_1) {
            pmin = x->data[b_i];
            x->data[b_i] = x->data[i];
            x->data[i] = pmin;
          }

          varargout_1 = ((cmp_tunableEnvironment[0].f1->data[x->data[i] - 1] <
                          cmp_tunableEnvironment[0].f1->data[x->data[pmax] - 1])
                         || ((cmp_tunableEnvironment[0].f1->data[x->data[i] - 1]
                              == cmp_tunableEnvironment[0].f1->data[x->data[pmax]
                              - 1]) && (cmp_tunableEnvironment[1].f1->data
            [x->data[i] - 1] < cmp_tunableEnvironment[1].f1->data[x->data[pmax]
            - 1])));
          if (varargout_1) {
            pmin = x->data[pmax];
            x->data[pmax] = x->data[i];
            x->data[i] = pmin;
          }

          pivot = x->data[pmax] - 1;
          i = frame.xend - 2;
          x->data[pmax] = x->data[i];
          x->data[i] = pivot + 1;
          j = i;
          do {
            exitg2 = 0;
            b_i++;
            do {
              exitg3 = 0;
              pmax = cmp_tunableEnvironment[0].f1->data[x->data[b_i] - 1];
              varargout_1 = ((pmax < cmp_tunableEnvironment[0].f1->data[pivot]) ||
                             ((pmax == cmp_tunableEnvironment[0].f1->data[pivot])
                              && (cmp_tunableEnvironment[1].f1->data[x->data[b_i]
                                  - 1] < cmp_tunableEnvironment[1].f1->
                                  data[pivot])));
              if (varargout_1) {
                b_i++;
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            j--;
            do {
              exitg3 = 0;
              pmax = cmp_tunableEnvironment[0].f1->data[x->data[j] - 1];
              varargout_1 = ((cmp_tunableEnvironment[0].f1->data[pivot] < pmax) ||
                             ((cmp_tunableEnvironment[0].f1->data[pivot] == pmax)
                              && (cmp_tunableEnvironment[1].f1->data[pivot] <
                                  cmp_tunableEnvironment[1].f1->data[x->data[j]
                                  - 1])));
              if (varargout_1) {
                j--;
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            if (b_i + 1 >= j + 1) {
              exitg2 = 1;
            } else {
              pmin = x->data[b_i];
              x->data[b_i] = x->data[j];
              x->data[j] = pmin;
            }
          } while (exitg2 == 0);

          x->data[i] = x->data[b_i];
          x->data[b_i] = pivot + 1;
          if (b_i + 2 < frame.xend) {
            st_d_data[pow2p].xstart = b_i + 2;
            st_d_data[pow2p].xend = frame.xend;
            st_d_data[pow2p].depth = frame.depth + 1;
            pow2p++;
          }

          if (frame.xstart < b_i + 1) {
            st_d_data[pow2p].xstart = frame.xstart;
            st_d_data[pow2p].xend = b_i + 1;
            st_d_data[pow2p].depth = frame.depth + 1;
            pow2p++;
          }
        }
      }
    }
  }
}

/* End of code generation (introsort.cpp) */
