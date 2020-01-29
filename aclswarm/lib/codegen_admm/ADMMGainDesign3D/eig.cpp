//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eig.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "eig.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "schur.h"
#include "xzggev.h"
#include "anyNonFinite.h"
#include "ADMMGainDesign3D_rtwutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_creal_T *V
//                emxArray_creal_T *D
// Return Type  : void
//
void eig(const emxArray_real_T *A, emxArray_creal_T *V, emxArray_creal_T *D)
{
  int i25;
  bool p;
  unsigned int unnamed_idx_0;
  signed char D_size_idx_0;
  unsigned int unnamed_idx_1;
  signed char D_size_idx_1;
  int j;
  int info;
  bool exitg2;
  emxArray_creal_T *At;
  int n;
  double scale;
  int exitg1;
  emxArray_creal_T *alpha1;
  emxArray_creal_T *beta1;
  int lastcol;
  int coltop;
  double colnorm;
  double absxk;
  double t;
  double alpha1_re;
  double alpha1_im;
  double brm;
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
    i25 = V->size[0] * V->size[1];
    V->size[0] = A->size[0];
    V->size[1] = A->size[1];
    emxEnsureCapacity_creal_T(V, i25);
    info = A->size[0] * A->size[1];
    for (i25 = 0; i25 < info; i25++) {
      V->data[i25].re = A->data[i25];
      V->data[i25].im = 0.0;
    }

    i25 = D->size[0] * D->size[1];
    D->size[0] = A->size[0];
    D->size[1] = A->size[1];
    emxEnsureCapacity_creal_T(D, i25);
    info = A->size[0] * A->size[1];
    for (i25 = 0; i25 < info; i25++) {
      D->data[i25].re = A->data[i25];
      D->data[i25].im = 0.0;
    }
  } else if (anyNonFinite(A)) {
    if ((A->size[0] == 1) && (A->size[1] == 1)) {
      D_size_idx_0 = (signed char)A->size[0];
      D_size_idx_1 = (signed char)A->size[1];
      i25 = V->size[0] * V->size[1];
      V->size[0] = 1;
      V->size[1] = 1;
      emxEnsureCapacity_creal_T(V, i25);
      for (i25 = 0; i25 < 1; i25++) {
        V->data[0].re = rtNaN;
        V->data[0].im = 0.0;
      }

      i25 = D->size[0] * D->size[1];
      D->size[0] = D_size_idx_0;
      D->size[1] = D_size_idx_1;
      emxEnsureCapacity_creal_T(D, i25);
      info = D_size_idx_0 * D_size_idx_1;
      for (i25 = 0; i25 < info; i25++) {
        D->data[i25].re = rtNaN;
        D->data[i25].im = 0.0;
      }
    } else {
      unnamed_idx_0 = (unsigned int)A->size[0];
      unnamed_idx_1 = (unsigned int)A->size[1];
      i25 = V->size[0] * V->size[1];
      V->size[0] = (int)unnamed_idx_0;
      V->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity_creal_T(V, i25);
      info = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i25 = 0; i25 < info; i25++) {
        V->data[i25].re = rtNaN;
        V->data[i25].im = 0.0;
      }

      unnamed_idx_0 = (unsigned int)A->size[0];
      unnamed_idx_1 = (unsigned int)A->size[1];
      i25 = D->size[0] * D->size[1];
      D->size[0] = (int)unnamed_idx_0;
      D->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity_creal_T(D, i25);
      info = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i25 = 0; i25 < info; i25++) {
        D->data[i25].re = 0.0;
        D->data[i25].im = 0.0;
      }

      i25 = (int)unnamed_idx_0;
      for (j = 0; j < i25; j++) {
        D->data[j + D->size[0] * j].re = rtNaN;
        D->data[j + D->size[0] * j].im = 0.0;
      }
    }
  } else if ((A->size[0] == 1) && (A->size[1] == 1)) {
    i25 = V->size[0] * V->size[1];
    V->size[0] = 1;
    V->size[1] = 1;
    emxEnsureCapacity_creal_T(V, i25);
    V->data[0].re = 1.0;
    V->data[0].im = 0.0;
    i25 = D->size[0] * D->size[1];
    D->size[0] = A->size[0];
    D->size[1] = A->size[1];
    emxEnsureCapacity_creal_T(D, i25);
    info = A->size[0] * A->size[1];
    for (i25 = 0; i25 < info; i25++) {
      D->data[i25].re = A->data[i25];
      D->data[i25].im = 0.0;
    }
  } else {
    p = (A->size[0] == A->size[1]);
    if (p) {
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j <= A->size[1] - 1)) {
        info = 0;
        do {
          exitg1 = 0;
          if (info <= j) {
            if (!(A->data[info + A->size[0] * j] == A->data[j + A->size[0] *
                  info])) {
              p = false;
              exitg1 = 1;
            } else {
              info++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }

    if (p) {
      schur(A, V, D);
      n = D->size[0];
      scale = D->data[0].re;
      D->data[0].re = scale;
      D->data[0].im = 0.0;
      for (j = 2; j <= n; j++) {
        scale = D->data[(j + D->size[0] * (j - 1)) - 1].re;
        D->data[(j + D->size[0] * (j - 1)) - 1].re = scale;
        D->data[(j + D->size[0] * (j - 1)) - 1].im = 0.0;
        D->data[(j + D->size[0] * (j - 2)) - 1].re = 0.0;
        D->data[(j + D->size[0] * (j - 2)) - 1].im = 0.0;
        for (info = 0; info <= j - 2; info++) {
          D->data[info + D->size[0] * (j - 1)].re = 0.0;
          D->data[info + D->size[0] * (j - 1)].im = 0.0;
        }
      }
    } else {
      emxInit_creal_T(&At, 2);
      i25 = At->size[0] * At->size[1];
      At->size[0] = A->size[0];
      At->size[1] = A->size[1];
      emxEnsureCapacity_creal_T(At, i25);
      info = A->size[0] * A->size[1];
      for (i25 = 0; i25 < info; i25++) {
        At->data[i25].re = A->data[i25];
        At->data[i25].im = 0.0;
      }

      emxInit_creal_T(&alpha1, 1);
      emxInit_creal_T(&beta1, 1);
      xzggev(At, &info, alpha1, beta1, V);
      n = A->size[0];
      lastcol = (A->size[0] - 1) * A->size[0] + 1;
      emxFree_creal_T(&At);
      for (coltop = 1; n < 0 ? coltop >= lastcol : coltop <= lastcol; coltop +=
           n) {
        colnorm = 0.0;
        if (n == 1) {
          colnorm = rt_hypotd_snf(V->data[coltop - 1].re, V->data[coltop - 1].im);
        } else {
          scale = 3.3121686421112381E-170;
          info = (coltop + n) - 1;
          for (j = coltop; j <= info; j++) {
            absxk = std::abs(V->data[j - 1].re);
            if (absxk > scale) {
              t = scale / absxk;
              colnorm = 1.0 + colnorm * t * t;
              scale = absxk;
            } else {
              t = absxk / scale;
              colnorm += t * t;
            }

            absxk = std::abs(V->data[j - 1].im);
            if (absxk > scale) {
              t = scale / absxk;
              colnorm = 1.0 + colnorm * t * t;
              scale = absxk;
            } else {
              t = absxk / scale;
              colnorm += t * t;
            }
          }

          colnorm = scale * std::sqrt(colnorm);
        }

        i25 = (coltop + n) - 1;
        for (j = coltop; j <= i25; j++) {
          scale = V->data[j - 1].re;
          absxk = V->data[j - 1].im;
          if (absxk == 0.0) {
            V->data[j - 1].re = scale / colnorm;
            V->data[j - 1].im = 0.0;
          } else if (scale == 0.0) {
            V->data[j - 1].re = 0.0;
            V->data[j - 1].im = absxk / colnorm;
          } else {
            V->data[j - 1].re = scale / colnorm;
            V->data[j - 1].im = absxk / colnorm;
          }
        }
      }

      i25 = D->size[0] * D->size[1];
      D->size[0] = alpha1->size[0];
      D->size[1] = alpha1->size[0];
      emxEnsureCapacity_creal_T(D, i25);
      info = alpha1->size[0] * alpha1->size[0];
      for (i25 = 0; i25 < info; i25++) {
        D->data[i25].re = 0.0;
        D->data[i25].im = 0.0;
      }

      i25 = alpha1->size[0];
      for (j = 0; j < i25; j++) {
        alpha1_re = alpha1->data[j].re;
        alpha1_im = alpha1->data[j].im;
        t = beta1->data[j].re;
        colnorm = beta1->data[j].im;
        if (colnorm == 0.0) {
          if (alpha1_im == 0.0) {
            D->data[j + D->size[0] * j].re = alpha1_re / t;
            D->data[j + D->size[0] * j].im = 0.0;
          } else if (alpha1_re == 0.0) {
            D->data[j + D->size[0] * j].re = 0.0;
            D->data[j + D->size[0] * j].im = alpha1_im / t;
          } else {
            D->data[j + D->size[0] * j].re = alpha1_re / t;
            D->data[j + D->size[0] * j].im = alpha1_im / t;
          }
        } else if (t == 0.0) {
          if (alpha1_re == 0.0) {
            D->data[j + D->size[0] * j].re = alpha1_im / colnorm;
            D->data[j + D->size[0] * j].im = 0.0;
          } else if (alpha1_im == 0.0) {
            D->data[j + D->size[0] * j].re = 0.0;
            D->data[j + D->size[0] * j].im = -(alpha1_re / colnorm);
          } else {
            D->data[j + D->size[0] * j].re = alpha1_im / colnorm;
            D->data[j + D->size[0] * j].im = -(alpha1_re / colnorm);
          }
        } else {
          brm = std::abs(t);
          scale = std::abs(colnorm);
          if (brm > scale) {
            absxk = colnorm / t;
            scale = t + absxk * colnorm;
            D->data[j + D->size[0] * j].re = (alpha1_re + absxk * alpha1_im) /
              scale;
            D->data[j + D->size[0] * j].im = (alpha1_im - absxk * alpha1_re) /
              scale;
          } else if (scale == brm) {
            if (t > 0.0) {
              absxk = 0.5;
            } else {
              absxk = -0.5;
            }

            if (colnorm > 0.0) {
              scale = 0.5;
            } else {
              scale = -0.5;
            }

            D->data[j + D->size[0] * j].re = (alpha1_re * absxk + alpha1_im *
              scale) / brm;
            D->data[j + D->size[0] * j].im = (alpha1_im * absxk - alpha1_re *
              scale) / brm;
          } else {
            absxk = t / colnorm;
            scale = colnorm + absxk * t;
            D->data[j + D->size[0] * j].re = (absxk * alpha1_re + alpha1_im) /
              scale;
            D->data[j + D->size[0] * j].im = (absxk * alpha1_im - alpha1_re) /
              scale;
          }
        }
      }

      emxFree_creal_T(&beta1);
      emxFree_creal_T(&alpha1);
    }
  }
}

//
// File trailer for eig.cpp
//
// [EOF]
//
