//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: schur.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "schur.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "ADMMGainDesign3D_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdhseqr.h"
#include "xdlanv2.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *V
//                emxArray_real_T *T
// Return Type  : void
//
void schur(const emxArray_real_T *A, emxArray_real_T *V, emxArray_real_T *T)
{
  int nx;
  bool p;
  int k;
  int n;
  int i;
  emxArray_real_T *tau;
  emxArray_real_T *work;
  int itau;
  int b_n;
  int nh;
  int j;
  int b_i;
  int im1n;
  int in;
  double alpha1;
  int ix;
  int ia;
  double temp;
  int jy;
  double beta1;
  int lastv;
  int lastc;
  int i1;
  bool exitg2;
  int exitg1;
  nx = A->size[0] * A->size[1];
  p = true;
  for (k = 0; k < nx; k++) {
    if ((!p) || (rtIsInf(A->data[k]) || rtIsNaN(A->data[k]))) {
      p = false;
    }
  }

  if (!p) {
    i = V->size[0] * V->size[1];
    V->size[0] = A->size[0];
    V->size[1] = A->size[1];
    emxEnsureCapacity_real_T(V, i);
    nx = A->size[0] * A->size[1];
    for (i = 0; i < nx; i++) {
      V->data[i] = rtNaN;
    }

    nx = V->size[0];
    if ((V->size[0] != 0) && (V->size[1] != 0) && (1 < V->size[0])) {
      itau = 2;
      if (V->size[0] - 2 < V->size[1] - 1) {
        nh = V->size[0] - 1;
      } else {
        nh = V->size[1];
      }

      for (j = 0; j < nh; j++) {
        for (b_i = itau; b_i <= nx; b_i++) {
          V->data[(b_i + V->size[0] * j) - 1] = 0.0;
        }

        itau++;
      }
    }

    i = T->size[0] * T->size[1];
    T->size[0] = A->size[0];
    T->size[1] = A->size[1];
    emxEnsureCapacity_real_T(T, i);
    nx = A->size[0] * A->size[1];
    for (i = 0; i < nx; i++) {
      T->data[i] = rtNaN;
    }
  } else {
    n = A->size[0];
    i = T->size[0] * T->size[1];
    T->size[0] = A->size[0];
    T->size[1] = A->size[1];
    emxEnsureCapacity_real_T(T, i);
    nx = A->size[0] * A->size[1];
    for (i = 0; i < nx; i++) {
      T->data[i] = A->data[i];
    }

    emxInit_real_T(&tau, 1);
    emxInit_real_T(&work, 1);
    b_n = A->size[0];
    i = tau->size[0];
    if (A->size[0] < 1) {
      tau->size[0] = 0;
    } else {
      tau->size[0] = A->size[0] - 1;
    }

    emxEnsureCapacity_real_T(tau, i);
    i = work->size[0];
    work->size[0] = A->size[0];
    emxEnsureCapacity_real_T(work, i);
    nx = A->size[0];
    for (i = 0; i < nx; i++) {
      work->data[i] = 0.0;
    }

    i = A->size[0];
    for (b_i = 0; b_i <= i - 2; b_i++) {
      nx = b_i * b_n;
      im1n = nx + 2;
      in = (b_i + 1) * b_n;
      alpha1 = T->data[(b_i + T->size[0] * b_i) + 1];
      itau = b_i + 3;
      if (itau >= b_n) {
        itau = b_n;
      }

      ix = itau + nx;
      itau = (b_n - b_i) - 3;
      tau->data[b_i] = 0.0;
      if (itau + 2 > 0) {
        temp = xnrm2(itau + 1, T, ix);
        if (temp != 0.0) {
          beta1 = rt_hypotd_snf(alpha1, temp);
          if (alpha1 >= 0.0) {
            beta1 = -beta1;
          }

          if (std::abs(beta1) < 1.0020841800044864E-292) {
            nx = -1;
            i1 = ix + itau;
            do {
              nx++;
              for (k = ix; k <= i1; k++) {
                T->data[k - 1] *= 9.9792015476736E+291;
              }

              beta1 *= 9.9792015476736E+291;
              alpha1 *= 9.9792015476736E+291;
            } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

            beta1 = rt_hypotd_snf(alpha1, xnrm2(itau + 1, T, ix));
            if (alpha1 >= 0.0) {
              beta1 = -beta1;
            }

            tau->data[b_i] = (beta1 - alpha1) / beta1;
            temp = 1.0 / (alpha1 - beta1);
            for (k = ix; k <= i1; k++) {
              T->data[k - 1] *= temp;
            }

            for (k = 0; k <= nx; k++) {
              beta1 *= 1.0020841800044864E-292;
            }

            alpha1 = beta1;
          } else {
            tau->data[b_i] = (beta1 - alpha1) / beta1;
            temp = 1.0 / (alpha1 - beta1);
            i1 = ix + itau;
            for (k = ix; k <= i1; k++) {
              T->data[k - 1] *= temp;
            }

            alpha1 = beta1;
          }
        }
      }

      T->data[(b_i + T->size[0] * b_i) + 1] = 1.0;
      jy = (b_i + im1n) - 1;
      nh = in + 1;
      if (tau->data[b_i] != 0.0) {
        lastv = itau + 1;
        nx = jy + itau;
        while ((lastv + 1 > 0) && (T->data[nx + 1] == 0.0)) {
          lastv--;
          nx--;
        }

        lastc = b_n;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          nx = in + lastc;
          ia = nx;
          do {
            exitg1 = 0;
            if ((b_n > 0) && (ia <= nx + lastv * b_n)) {
              if (T->data[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia += b_n;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        lastc = 0;
      }

      if (lastv + 1 > 0) {
        if (lastc != 0) {
          for (nx = 0; nx < lastc; nx++) {
            work->data[nx] = 0.0;
          }

          ix = jy;
          i1 = (in + b_n * lastv) + 1;
          for (itau = nh; b_n < 0 ? itau >= i1 : itau <= i1; itau += b_n) {
            nx = 0;
            k = (itau + lastc) - 1;
            for (ia = itau; ia <= k; ia++) {
              work->data[nx] += T->data[ia - 1] * T->data[ix];
              nx++;
            }

            ix++;
          }
        }

        if (!(-tau->data[b_i] == 0.0)) {
          nx = in;
          for (j = 0; j <= lastv; j++) {
            if (T->data[jy] != 0.0) {
              temp = T->data[jy] * -tau->data[b_i];
              ix = 0;
              i1 = nx + 1;
              k = lastc + nx;
              for (itau = i1; itau <= k; itau++) {
                T->data[itau - 1] += work->data[ix] * temp;
                ix++;
              }
            }

            jy++;
            nx += b_n;
          }
        }
      }

      xzlarf((b_n - b_i) - 1, (b_n - b_i) - 1, b_i + im1n, tau->data[b_i], T,
             (b_i + in) + 2, b_n, work);
      T->data[(b_i + T->size[0] * b_i) + 1] = alpha1;
    }

    i = V->size[0] * V->size[1];
    V->size[0] = T->size[0];
    V->size[1] = T->size[1];
    emxEnsureCapacity_real_T(V, i);
    nx = T->size[0] * T->size[1];
    for (i = 0; i < nx; i++) {
      V->data[i] = T->data[i];
    }

    if (A->size[0] != 0) {
      nh = A->size[0] - 1;
      for (j = n; j >= 2; j--) {
        ia = (j - 1) * n - 1;
        for (b_i = 0; b_i <= j - 2; b_i++) {
          V->data[(ia + b_i) + 1] = 0.0;
        }

        nx = ia - n;
        i = j + 1;
        for (b_i = i; b_i <= n; b_i++) {
          V->data[ia + b_i] = V->data[nx + b_i];
        }

        i = n + 1;
        for (b_i = i; b_i <= n; b_i++) {
          V->data[ia + b_i] = 0.0;
        }
      }

      for (b_i = 0; b_i < n; b_i++) {
        V->data[b_i] = 0.0;
      }

      V->data[0] = 1.0;
      i = A->size[0] + 1;
      for (j = i; j <= n; j++) {
        ia = (j - 1) * n;
        for (b_i = 0; b_i < n; b_i++) {
          V->data[ia + b_i] = 0.0;
        }

        V->data[(ia + j) - 1] = 1.0;
      }

      if (A->size[0] - 1 >= 1) {
        i = A->size[0] - 2;
        for (j = nh; j <= i; j++) {
          ia = (n + j * n) + 1;
          i1 = n - 2;
          for (b_i = 0; b_i <= i1; b_i++) {
            V->data[ia + b_i] = 0.0;
          }

          V->data[ia + j] = 1.0;
        }

        itau = A->size[0] - 2;
        i = work->size[0];
        work->size[0] = V->size[1];
        emxEnsureCapacity_real_T(work, i);
        nx = V->size[1];
        for (i = 0; i < nx; i++) {
          work->data[i] = 0.0;
        }

        for (b_i = A->size[0] - 1; b_i >= 1; b_i--) {
          nx = (n + b_i) + (b_i - 1) * n;
          if (b_i < n - 1) {
            V->data[nx] = 1.0;
            xzlarf(n - b_i, nh - b_i, nx + 1, tau->data[itau], V, (nx + n) + 1,
                   n, work);
            ix = nx + 2;
            i = (nx + n) - b_i;
            for (k = ix; k <= i; k++) {
              V->data[k - 1] *= -tau->data[itau];
            }
          }

          V->data[nx] = 1.0 - tau->data[itau];
          for (j = 0; j <= b_i - 2; j++) {
            V->data[(nx - j) - 1] = 0.0;
          }

          itau--;
        }
      }
    }

    emxFree_real_T(&work);
    emxFree_real_T(&tau);
    eml_dlahqr(T, V);
    nx = T->size[0];
    if ((T->size[0] != 0) && (T->size[1] != 0) && (3 < T->size[0])) {
      itau = 4;
      if (T->size[0] - 4 < T->size[1] - 1) {
        nh = T->size[0] - 3;
      } else {
        nh = T->size[1];
      }

      for (j = 0; j < nh; j++) {
        for (b_i = itau; b_i <= nx; b_i++) {
          T->data[(b_i + T->size[0] * j) - 1] = 0.0;
        }

        itau++;
      }
    }
  }
}

//
// File trailer for schur.cpp
//
// [EOF]
//
