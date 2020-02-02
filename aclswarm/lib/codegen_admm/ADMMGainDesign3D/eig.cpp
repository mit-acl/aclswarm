//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eig.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "eig.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "ADMMGainDesign3D_rtwutil.h"
#include "rt_nonfinite.h"
#include "schur.h"
#include "xdlanv2.h"
#include "xzggev.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_creal_T *V
//                emxArray_creal_T *D
// Return Type  : void
//
void eig(const emxArray_real_T *A, emxArray_creal_T *V, emxArray_creal_T *D)
{
  int nx;
  int i;
  bool p;
  int k;
  unsigned int unnamed_idx_0;
  signed char D_size_idx_0;
  unsigned int unnamed_idx_1;
  signed char D_size_idx_1;
  bool exitg2;
  emxArray_creal_T *At;
  emxArray_real_T *b_V;
  emxArray_real_T *b_D;
  int exitg1;
  emxArray_creal_T *alpha1;
  emxArray_creal_T *beta1;
  int n;
  int lastcol;
  int coltop;
  double colnorm;
  double scale;
  double absxk;
  double t;
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
    i = V->size[0] * V->size[1];
    V->size[0] = A->size[0];
    V->size[1] = A->size[1];
    emxEnsureCapacity_creal_T(V, i);
    nx = A->size[0] * A->size[1];
    for (i = 0; i < nx; i++) {
      V->data[i].re = A->data[i];
      V->data[i].im = 0.0;
    }

    i = D->size[0] * D->size[1];
    D->size[0] = A->size[0];
    D->size[1] = A->size[1];
    emxEnsureCapacity_creal_T(D, i);
    nx = A->size[0] * A->size[1];
    for (i = 0; i < nx; i++) {
      D->data[i].re = A->data[i];
      D->data[i].im = 0.0;
    }
  } else {
    nx = A->size[0] * A->size[1];
    p = true;
    for (k = 0; k < nx; k++) {
      if ((!p) || (rtIsInf(A->data[k]) || rtIsNaN(A->data[k]))) {
        p = false;
      }
    }

    if (!p) {
      if ((A->size[0] == 1) && (A->size[1] == 1)) {
        D_size_idx_0 = static_cast<signed char>(A->size[0]);
        D_size_idx_1 = static_cast<signed char>(A->size[1]);
        i = V->size[0] * V->size[1];
        V->size[0] = 1;
        V->size[1] = 1;
        emxEnsureCapacity_creal_T(V, i);
        for (i = 0; i < 1; i++) {
          V->data[0].re = rtNaN;
          V->data[0].im = 0.0;
        }

        i = D->size[0] * D->size[1];
        D->size[0] = D_size_idx_0;
        D->size[1] = D_size_idx_1;
        emxEnsureCapacity_creal_T(D, i);
        nx = D_size_idx_0 * D_size_idx_1;
        for (i = 0; i < nx; i++) {
          D->data[i].re = rtNaN;
          D->data[i].im = 0.0;
        }
      } else {
        unnamed_idx_0 = static_cast<unsigned int>(A->size[0]);
        unnamed_idx_1 = static_cast<unsigned int>(A->size[1]);
        i = V->size[0] * V->size[1];
        V->size[0] = static_cast<int>(unnamed_idx_0);
        V->size[1] = static_cast<int>(unnamed_idx_1);
        emxEnsureCapacity_creal_T(V, i);
        nx = static_cast<int>(unnamed_idx_0) * static_cast<int>(unnamed_idx_1);
        for (i = 0; i < nx; i++) {
          V->data[i].re = rtNaN;
          V->data[i].im = 0.0;
        }

        unnamed_idx_0 = static_cast<unsigned int>(A->size[0]);
        unnamed_idx_1 = static_cast<unsigned int>(A->size[1]);
        i = D->size[0] * D->size[1];
        D->size[0] = static_cast<int>(unnamed_idx_0);
        D->size[1] = static_cast<int>(unnamed_idx_1);
        emxEnsureCapacity_creal_T(D, i);
        nx = static_cast<int>(unnamed_idx_0) * static_cast<int>(unnamed_idx_1);
        for (i = 0; i < nx; i++) {
          D->data[i].re = 0.0;
          D->data[i].im = 0.0;
        }

        i = static_cast<int>(unnamed_idx_0);
        for (k = 0; k < i; k++) {
          D->data[k + D->size[0] * k].re = rtNaN;
          D->data[k + D->size[0] * k].im = 0.0;
        }
      }
    } else if ((A->size[0] == 1) && (A->size[1] == 1)) {
      i = V->size[0] * V->size[1];
      V->size[0] = 1;
      V->size[1] = 1;
      emxEnsureCapacity_creal_T(V, i);
      V->data[0].re = 1.0;
      V->data[0].im = 0.0;
      i = D->size[0] * D->size[1];
      D->size[0] = A->size[0];
      D->size[1] = A->size[1];
      emxEnsureCapacity_creal_T(D, i);
      nx = A->size[0] * A->size[1];
      for (i = 0; i < nx; i++) {
        D->data[i].re = A->data[i];
        D->data[i].im = 0.0;
      }
    } else {
      p = (A->size[0] == A->size[1]);
      if (p) {
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= A->size[1] - 1)) {
          nx = 0;
          do {
            exitg1 = 0;
            if (nx <= k) {
              if (!(A->data[nx + A->size[0] * k] == A->data[k + A->size[0] * nx]))
              {
                p = false;
                exitg1 = 1;
              } else {
                nx++;
              }
            } else {
              k++;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      }

      if (p) {
        emxInit_real_T(&b_V, 2);
        emxInit_real_T(&b_D, 2);
        schur(A, b_V, b_D);
        i = V->size[0] * V->size[1];
        V->size[0] = b_V->size[0];
        V->size[1] = b_V->size[1];
        emxEnsureCapacity_creal_T(V, i);
        nx = b_V->size[0] * b_V->size[1];
        for (i = 0; i < nx; i++) {
          V->data[i].re = b_V->data[i];
          V->data[i].im = 0.0;
        }

        emxFree_real_T(&b_V);
        n = b_D->size[0];
        for (k = 2; k <= n; k++) {
          b_D->data[(k + b_D->size[0] * (k - 2)) - 1] = 0.0;
          for (nx = 0; nx <= k - 2; nx++) {
            b_D->data[nx + b_D->size[0] * (k - 1)] = 0.0;
          }
        }

        i = D->size[0] * D->size[1];
        D->size[0] = b_D->size[0];
        D->size[1] = b_D->size[1];
        emxEnsureCapacity_creal_T(D, i);
        nx = b_D->size[0] * b_D->size[1];
        for (i = 0; i < nx; i++) {
          D->data[i].re = b_D->data[i];
          D->data[i].im = 0.0;
        }

        emxFree_real_T(&b_D);
      } else {
        emxInit_creal_T(&At, 2);
        i = At->size[0] * At->size[1];
        At->size[0] = A->size[0];
        At->size[1] = A->size[1];
        emxEnsureCapacity_creal_T(At, i);
        nx = A->size[0] * A->size[1];
        for (i = 0; i < nx; i++) {
          At->data[i].re = A->data[i];
          At->data[i].im = 0.0;
        }

        emxInit_creal_T(&alpha1, 1);
        emxInit_creal_T(&beta1, 1);
        xzggev(At, &nx, alpha1, beta1, V);
        n = A->size[0];
        lastcol = (A->size[0] - 1) * A->size[0] + 1;
        emxFree_creal_T(&At);
        for (coltop = 1; n < 0 ? coltop >= lastcol : coltop <= lastcol; coltop +=
             n) {
          colnorm = 0.0;
          if (n == 1) {
            colnorm = rt_hypotd_snf(V->data[coltop - 1].re, V->data[coltop - 1].
              im);
          } else {
            scale = 3.3121686421112381E-170;
            nx = (coltop + n) - 1;
            for (k = coltop; k <= nx; k++) {
              absxk = std::abs(V->data[k - 1].re);
              if (absxk > scale) {
                t = scale / absxk;
                colnorm = colnorm * t * t + 1.0;
                scale = absxk;
              } else {
                t = absxk / scale;
                colnorm += t * t;
              }

              absxk = std::abs(V->data[k - 1].im);
              if (absxk > scale) {
                t = scale / absxk;
                colnorm = colnorm * t * t + 1.0;
                scale = absxk;
              } else {
                t = absxk / scale;
                colnorm += t * t;
              }
            }

            colnorm = scale * std::sqrt(colnorm);
          }

          i = (coltop + n) - 1;
          for (k = coltop; k <= i; k++) {
            absxk = V->data[k - 1].re;
            scale = V->data[k - 1].im;
            if (scale == 0.0) {
              absxk /= colnorm;
              scale = 0.0;
            } else if (absxk == 0.0) {
              absxk = 0.0;
              scale /= colnorm;
            } else {
              absxk /= colnorm;
              scale /= colnorm;
            }

            V->data[k - 1].re = absxk;
            V->data[k - 1].im = scale;
          }
        }

        i = D->size[0] * D->size[1];
        D->size[0] = alpha1->size[0];
        D->size[1] = alpha1->size[0];
        emxEnsureCapacity_creal_T(D, i);
        nx = alpha1->size[0] * alpha1->size[0];
        for (i = 0; i < nx; i++) {
          D->data[i].re = 0.0;
          D->data[i].im = 0.0;
        }

        i = alpha1->size[0];
        for (k = 0; k < i; k++) {
          if (beta1->data[k].im == 0.0) {
            if (alpha1->data[k].im == 0.0) {
              D->data[k + D->size[0] * k].re = alpha1->data[k].re / beta1->
                data[k].re;
              D->data[k + D->size[0] * k].im = 0.0;
            } else if (alpha1->data[k].re == 0.0) {
              D->data[k + D->size[0] * k].re = 0.0;
              D->data[k + D->size[0] * k].im = alpha1->data[k].im / beta1->
                data[k].re;
            } else {
              D->data[k + D->size[0] * k].re = alpha1->data[k].re / beta1->
                data[k].re;
              D->data[k + D->size[0] * k].im = alpha1->data[k].im / beta1->
                data[k].re;
            }
          } else if (beta1->data[k].re == 0.0) {
            if (alpha1->data[k].re == 0.0) {
              D->data[k + D->size[0] * k].re = alpha1->data[k].im / beta1->
                data[k].im;
              D->data[k + D->size[0] * k].im = 0.0;
            } else if (alpha1->data[k].im == 0.0) {
              D->data[k + D->size[0] * k].re = 0.0;
              D->data[k + D->size[0] * k].im = -(alpha1->data[k].re /
                beta1->data[k].im);
            } else {
              D->data[k + D->size[0] * k].re = alpha1->data[k].im / beta1->
                data[k].im;
              D->data[k + D->size[0] * k].im = -(alpha1->data[k].re /
                beta1->data[k].im);
            }
          } else {
            t = std::abs(beta1->data[k].re);
            scale = std::abs(beta1->data[k].im);
            if (t > scale) {
              scale = beta1->data[k].im / beta1->data[k].re;
              absxk = beta1->data[k].re + scale * beta1->data[k].im;
              D->data[k + D->size[0] * k].re = (alpha1->data[k].re + scale *
                alpha1->data[k].im) / absxk;
              D->data[k + D->size[0] * k].im = (alpha1->data[k].im - scale *
                alpha1->data[k].re) / absxk;
            } else if (scale == t) {
              if (beta1->data[k].re > 0.0) {
                scale = 0.5;
              } else {
                scale = -0.5;
              }

              if (beta1->data[k].im > 0.0) {
                absxk = 0.5;
              } else {
                absxk = -0.5;
              }

              D->data[k + D->size[0] * k].re = (alpha1->data[k].re * scale +
                alpha1->data[k].im * absxk) / t;
              D->data[k + D->size[0] * k].im = (alpha1->data[k].im * scale -
                alpha1->data[k].re * absxk) / t;
            } else {
              scale = beta1->data[k].re / beta1->data[k].im;
              absxk = beta1->data[k].im + scale * beta1->data[k].re;
              D->data[k + D->size[0] * k].re = (scale * alpha1->data[k].re +
                alpha1->data[k].im) / absxk;
              D->data[k + D->size[0] * k].im = (scale * alpha1->data[k].im -
                alpha1->data[k].re) / absxk;
            }
          }
        }

        emxFree_creal_T(&beta1);
        emxFree_creal_T(&alpha1);
      }
    }
  }
}

//
// File trailer for eig.cpp
//
// [EOF]
//
