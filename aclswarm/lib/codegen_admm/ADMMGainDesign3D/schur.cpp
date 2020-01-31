//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: schur.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "schur.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "xscal.h"
#include "xzlarf.h"
#include "xdlanv2.h"
#include "xdhseqr.h"
#include "xnrm2.h"
#include "anyNonFinite.h"
#include "ADMMGainDesign3D_rtwutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_creal_T *V
//                emxArray_creal_T *T
// Return Type  : void
//
void schur(const emxArray_real_T *A, emxArray_creal_T *V, emxArray_creal_T *T)
{
  emxArray_real_T *b_A;
  int jend;
  int n;
  int ntau;
  int i26;
  int b_n;
  int knt;
  emxArray_real_T *tau;
  emxArray_real_T *work;
  int iv0;
  int i;
  emxArray_real_T *Vr;
  int im1n;
  int in;
  double alpha1;
  int ix;
  int ia;
  double xnorm;
  int lastv;
  int lastc;
  int i27;
  bool exitg2;
  int exitg1;
  int i28;
  double c;
  double d;
  double s;
  double rt1i;
  double t1_re;
  double t1_im;
  double mu1_im;
  double mu1_re;
  if (anyNonFinite(A)) {
    jend = A->size[0];
    ntau = A->size[1];
    i26 = V->size[0] * V->size[1];
    V->size[0] = jend;
    V->size[1] = ntau;
    emxEnsureCapacity_creal_T(V, i26);
    ntau *= jend;
    for (i26 = 0; i26 < ntau; i26++) {
      V->data[i26].re = rtNaN;
      V->data[i26].im = 0.0;
    }

    knt = V->size[0];
    if ((V->size[0] == 0) || (V->size[1] == 0) || (2 >= V->size[0])) {
    } else {
      ntau = 3;
      if (V->size[0] - 3 < V->size[1] - 1) {
        jend = V->size[0] - 2;
      } else {
        jend = V->size[1];
      }

      for (iv0 = 0; iv0 < jend; iv0++) {
        for (i = ntau; i <= knt; i++) {
          V->data[(i + V->size[0] * iv0) - 1].re = 0.0;
          V->data[(i + V->size[0] * iv0) - 1].im = 0.0;
        }

        ntau++;
      }
    }

    jend = A->size[0];
    ntau = A->size[1];
    i26 = T->size[0] * T->size[1];
    T->size[0] = jend;
    T->size[1] = ntau;
    emxEnsureCapacity_creal_T(T, i26);
    ntau *= jend;
    for (i26 = 0; i26 < ntau; i26++) {
      T->data[i26].re = rtNaN;
      T->data[i26].im = 0.0;
    }
  } else {
    emxInit_real_T(&b_A, 2);
    n = A->size[0];
    i26 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i26);
    ntau = A->size[0] * A->size[1];
    for (i26 = 0; i26 < ntau; i26++) {
      b_A->data[i26] = A->data[i26];
    }

    b_n = A->size[0];
    if (A->size[0] < 1) {
      ntau = 0;
    } else {
      ntau = A->size[0] - 1;
    }

    emxInit_real_T(&tau, 1);
    emxInit_real_T(&work, 1);
    i26 = tau->size[0];
    tau->size[0] = ntau;
    emxEnsureCapacity_real_T(tau, i26);
    jend = A->size[0];
    i26 = work->size[0];
    work->size[0] = jend;
    emxEnsureCapacity_real_T(work, i26);
    for (i26 = 0; i26 < jend; i26++) {
      work->data[i26] = 0.0;
    }

    i26 = A->size[0];
    for (i = 0; i <= i26 - 2; i++) {
      ntau = i * b_n;
      im1n = ntau + 2;
      in = (i + 1) * b_n;
      alpha1 = b_A->data[(i + b_A->size[0] * i) + 1];
      jend = i + 3;
      if (jend >= b_n) {
        jend = b_n;
      }

      ntau += jend;
      jend = b_n - i;
      ix = jend - 2;
      tau->data[i] = 0.0;
      if (ix + 1 > 0) {
        xnorm = xnrm2(ix, b_A, ntau);
        if (xnorm != 0.0) {
          xnorm = rt_hypotd_snf(alpha1, xnorm);
          if (alpha1 >= 0.0) {
            xnorm = -xnorm;
          }

          if (std::abs(xnorm) < 1.0020841800044864E-292) {
            knt = -1;
            do {
              knt++;
              xscal(ix, 9.9792015476736E+291, b_A, ntau);
              xnorm *= 9.9792015476736E+291;
              alpha1 *= 9.9792015476736E+291;
            } while (!(std::abs(xnorm) >= 1.0020841800044864E-292));

            xnorm = rt_hypotd_snf(alpha1, xnrm2(ix, b_A, ntau));
            if (alpha1 >= 0.0) {
              xnorm = -xnorm;
            }

            tau->data[i] = (xnorm - alpha1) / xnorm;
            xscal(ix, 1.0 / (alpha1 - xnorm), b_A, ntau);
            for (ntau = 0; ntau <= knt; ntau++) {
              xnorm *= 1.0020841800044864E-292;
            }

            alpha1 = xnorm;
          } else {
            tau->data[i] = (xnorm - alpha1) / xnorm;
            xscal(ix, 1.0 / (alpha1 - xnorm), b_A, ntau);
            alpha1 = xnorm;
          }
        }
      }

      b_A->data[(i + b_A->size[0] * i) + 1] = 1.0;
      ix = jend - 3;
      iv0 = i + im1n;
      knt = in + 1;
      if (tau->data[i] != 0.0) {
        lastv = ix + 2;
        ntau = iv0 + ix;
        while ((lastv > 0) && (b_A->data[ntau] == 0.0)) {
          lastv--;
          ntau--;
        }

        lastc = b_n;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          ntau = in + lastc;
          ia = ntau;
          do {
            exitg1 = 0;
            if ((b_n > 0) && (ia <= ntau + (lastv - 1) * b_n)) {
              if (b_A->data[ia - 1] != 0.0) {
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
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc != 0) {
          for (ntau = 0; ntau < lastc; ntau++) {
            work->data[ntau] = 0.0;
          }

          ix = iv0;
          i27 = (in + b_n * (lastv - 1)) + 1;
          for (jend = knt; b_n < 0 ? jend >= i27 : jend <= i27; jend += b_n) {
            ntau = 0;
            i28 = (jend + lastc) - 1;
            for (ia = jend; ia <= i28; ia++) {
              work->data[ntau] += b_A->data[ia - 1] * b_A->data[ix - 1];
              ntau++;
            }

            ix++;
          }
        }

        if (!(-tau->data[i] == 0.0)) {
          knt = in;
          ntau = iv0 - 1;
          for (iv0 = 0; iv0 < lastv; iv0++) {
            if (b_A->data[ntau] != 0.0) {
              xnorm = b_A->data[ntau] * -tau->data[i];
              ix = 0;
              i27 = knt + 1;
              i28 = lastc + knt;
              for (jend = i27; jend <= i28; jend++) {
                b_A->data[jend - 1] += work->data[ix] * xnorm;
                ix++;
              }
            }

            ntau++;
            knt += b_n;
          }
        }
      }

      xzlarf((b_n - i) - 1, (b_n - i) - 1, i + im1n, tau->data[i], b_A, (i + in)
             + 2, b_n, work);
      b_A->data[(i + b_A->size[0] * i) + 1] = alpha1;
    }

    emxInit_real_T(&Vr, 2);
    i26 = Vr->size[0] * Vr->size[1];
    Vr->size[0] = b_A->size[0];
    Vr->size[1] = b_A->size[1];
    emxEnsureCapacity_real_T(Vr, i26);
    ntau = b_A->size[0] * b_A->size[1];
    for (i26 = 0; i26 < ntau; i26++) {
      Vr->data[i26] = b_A->data[i26];
    }

    if (A->size[0] != 0) {
      ix = A->size[0] - 1;
      for (iv0 = n; iv0 >= 2; iv0--) {
        ia = (iv0 - 1) * n;
        for (i = 0; i <= iv0 - 2; i++) {
          Vr->data[ia + i] = 0.0;
        }

        ntau = ia - n;
        i26 = iv0 + 1;
        for (i = i26; i <= n; i++) {
          Vr->data[(ia + i) - 1] = Vr->data[(ntau + i) - 1];
        }

        i26 = n + 1;
        for (i = i26; i <= n; i++) {
          Vr->data[(ia + i) - 1] = 0.0;
        }
      }

      for (i = 0; i < n; i++) {
        Vr->data[i] = 0.0;
      }

      Vr->data[0] = 1.0;
      i26 = A->size[0] + 1;
      for (iv0 = i26; iv0 <= n; iv0++) {
        ia = (iv0 - 1) * n;
        for (i = 0; i < n; i++) {
          Vr->data[ia + i] = 0.0;
        }

        Vr->data[(ia + iv0) - 1] = 1.0;
      }

      if (A->size[0] - 1 >= 1) {
        i26 = A->size[0] - 2;
        for (iv0 = ix; iv0 <= i26; iv0++) {
          ia = (n + iv0 * n) + 1;
          i27 = n - 2;
          for (i = 0; i <= i27; i++) {
            Vr->data[ia + i] = 0.0;
          }

          Vr->data[ia + iv0] = 1.0;
        }

        knt = A->size[0] - 2;
        jend = Vr->size[1];
        i26 = work->size[0];
        work->size[0] = jend;
        emxEnsureCapacity_real_T(work, i26);
        for (i26 = 0; i26 < jend; i26++) {
          work->data[i26] = 0.0;
        }

        for (i = A->size[0] - 1; i >= 1; i--) {
          ntau = (n + i) + (i - 1) * n;
          if (i < n - 1) {
            Vr->data[ntau] = 1.0;
            xzlarf(n - i, ix - i, ntau + 1, tau->data[knt], Vr, (ntau + n) + 1,
                   n, work);
            xscal(ix - i, -tau->data[knt], Vr, ntau + 2);
          }

          Vr->data[ntau] = 1.0 - tau->data[knt];
          for (iv0 = 0; iv0 <= i - 2; iv0++) {
            Vr->data[(ntau - iv0) - 1] = 0.0;
          }

          knt--;
        }
      }
    }

    emxFree_real_T(&work);
    emxFree_real_T(&tau);
    eml_dlahqr(b_A, Vr);
    knt = b_A->size[0];
    if ((b_A->size[0] == 0) || (b_A->size[1] == 0) || (3 >= b_A->size[0])) {
    } else {
      ntau = 4;
      if (b_A->size[0] - 4 < b_A->size[1] - 1) {
        jend = b_A->size[0] - 3;
      } else {
        jend = b_A->size[1];
      }

      for (iv0 = 0; iv0 < jend; iv0++) {
        for (i = ntau; i <= knt; i++) {
          b_A->data[(i + b_A->size[0] * iv0) - 1] = 0.0;
        }

        ntau++;
      }
    }

    i26 = T->size[0] * T->size[1];
    T->size[0] = b_A->size[0];
    T->size[1] = b_A->size[1];
    emxEnsureCapacity_creal_T(T, i26);
    ntau = b_A->size[0] * b_A->size[1];
    for (i26 = 0; i26 < ntau; i26++) {
      T->data[i26].re = b_A->data[i26];
      T->data[i26].im = 0.0;
    }

    i26 = V->size[0] * V->size[1];
    V->size[0] = Vr->size[0];
    V->size[1] = Vr->size[1];
    emxEnsureCapacity_creal_T(V, i26);
    ntau = Vr->size[0] * Vr->size[1];
    for (i26 = 0; i26 < ntau; i26++) {
      V->data[i26].re = Vr->data[i26];
      V->data[i26].im = 0.0;
    }

    jend = b_A->size[0];
    ntau = b_A->size[1];
    if (jend < ntau) {
      ntau = jend;
    }

    jend = Vr->size[0];
    n = Vr->size[1];
    if (jend < n) {
      n = jend;
    }

    emxFree_real_T(&Vr);
    if (ntau < n) {
      n = ntau;
    }

    if (n != 0) {
      for (knt = n - 1; knt + 1 >= 2; knt--) {
        if (b_A->data[knt + b_A->size[0] * (knt - 1)] != 0.0) {
          xnorm = b_A->data[(knt + b_A->size[0] * (knt - 1)) - 1];
          alpha1 = b_A->data[(knt + b_A->size[0] * knt) - 1];
          c = b_A->data[knt + b_A->size[0] * (knt - 1)];
          d = b_A->data[knt + b_A->size[0] * knt];
          xdlanv2(&xnorm, &alpha1, &c, &d, &s, &rt1i, &t1_re, &t1_im, &mu1_im,
                  &mu1_re);
          mu1_re = s - b_A->data[knt + b_A->size[0] * knt];
          xnorm = rt_hypotd_snf(rt_hypotd_snf(mu1_re, rt1i), b_A->data[knt +
                                b_A->size[0] * (knt - 1)]);
          if (rt1i == 0.0) {
            mu1_re /= xnorm;
            mu1_im = 0.0;
          } else if (mu1_re == 0.0) {
            mu1_re = 0.0;
            mu1_im = rt1i / xnorm;
          } else {
            mu1_re /= xnorm;
            mu1_im = rt1i / xnorm;
          }

          s = b_A->data[knt + b_A->size[0] * (knt - 1)] / xnorm;
          for (iv0 = knt; iv0 <= n; iv0++) {
            t1_re = T->data[(knt + T->size[0] * (iv0 - 1)) - 1].re;
            t1_im = T->data[(knt + T->size[0] * (iv0 - 1)) - 1].im;
            c = T->data[(knt + T->size[0] * (iv0 - 1)) - 1].re;
            d = T->data[(knt + T->size[0] * (iv0 - 1)) - 1].im;
            xnorm = T->data[(knt + T->size[0] * (iv0 - 1)) - 1].im;
            alpha1 = T->data[(knt + T->size[0] * (iv0 - 1)) - 1].re;
            T->data[(knt + T->size[0] * (iv0 - 1)) - 1].re = (mu1_re * c +
              mu1_im * d) + s * T->data[knt + T->size[0] * (iv0 - 1)].re;
            T->data[(knt + T->size[0] * (iv0 - 1)) - 1].im = (mu1_re * xnorm -
              mu1_im * alpha1) + s * T->data[knt + T->size[0] * (iv0 - 1)].im;
            xnorm = mu1_re * T->data[knt + T->size[0] * (iv0 - 1)].re - mu1_im *
              T->data[knt + T->size[0] * (iv0 - 1)].im;
            alpha1 = mu1_re * T->data[knt + T->size[0] * (iv0 - 1)].im + mu1_im *
              T->data[knt + T->size[0] * (iv0 - 1)].re;
            T->data[knt + T->size[0] * (iv0 - 1)].re = xnorm - s * t1_re;
            T->data[knt + T->size[0] * (iv0 - 1)].im = alpha1 - s * t1_im;
          }

          for (i = 0; i <= knt; i++) {
            t1_re = T->data[i + T->size[0] * (knt - 1)].re;
            t1_im = T->data[i + T->size[0] * (knt - 1)].im;
            xnorm = mu1_re * T->data[i + T->size[0] * (knt - 1)].re - mu1_im *
              T->data[i + T->size[0] * (knt - 1)].im;
            alpha1 = mu1_re * T->data[i + T->size[0] * (knt - 1)].im + mu1_im *
              T->data[i + T->size[0] * (knt - 1)].re;
            c = T->data[i + T->size[0] * knt].re;
            d = T->data[i + T->size[0] * knt].im;
            T->data[i + T->size[0] * (knt - 1)].re = xnorm + s * c;
            T->data[i + T->size[0] * (knt - 1)].im = alpha1 + s * d;
            c = T->data[i + T->size[0] * knt].re;
            d = T->data[i + T->size[0] * knt].im;
            xnorm = T->data[i + T->size[0] * knt].im;
            alpha1 = T->data[i + T->size[0] * knt].re;
            T->data[i + T->size[0] * knt].re = (mu1_re * c + mu1_im * d) - s *
              t1_re;
            T->data[i + T->size[0] * knt].im = (mu1_re * xnorm - mu1_im * alpha1)
              - s * t1_im;
          }

          for (i = 0; i < n; i++) {
            t1_re = V->data[i + V->size[0] * (knt - 1)].re;
            t1_im = V->data[i + V->size[0] * (knt - 1)].im;
            xnorm = mu1_re * V->data[i + V->size[0] * (knt - 1)].re - mu1_im *
              V->data[i + V->size[0] * (knt - 1)].im;
            alpha1 = mu1_re * V->data[i + V->size[0] * (knt - 1)].im + mu1_im *
              V->data[i + V->size[0] * (knt - 1)].re;
            c = V->data[i + V->size[0] * knt].re;
            d = V->data[i + V->size[0] * knt].im;
            V->data[i + V->size[0] * (knt - 1)].re = xnorm + s * c;
            V->data[i + V->size[0] * (knt - 1)].im = alpha1 + s * d;
            c = V->data[i + V->size[0] * knt].re;
            d = V->data[i + V->size[0] * knt].im;
            xnorm = V->data[i + V->size[0] * knt].im;
            alpha1 = V->data[i + V->size[0] * knt].re;
            V->data[i + V->size[0] * knt].re = (mu1_re * c + mu1_im * d) - s *
              t1_re;
            V->data[i + V->size[0] * knt].im = (mu1_re * xnorm - mu1_im * alpha1)
              - s * t1_im;
          }

          T->data[knt + T->size[0] * (knt - 1)].re = 0.0;
          T->data[knt + T->size[0] * (knt - 1)].im = 0.0;
        }
      }
    }

    emxFree_real_T(&b_A);
  }
}

//
// File trailer for schur.cpp
//
// [EOF]
//
