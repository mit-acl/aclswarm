//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzggev.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "xzggev.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "ADMMGainDesign3D_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xzggbal.h"
#include "xzhgeqz.h"
#include "xzlartg.h"
#include "xztgevc.h"
#include <cmath>

// Function Definitions

//
// Arguments    : emxArray_creal_T *A
//                int *info
//                emxArray_creal_T *alpha1
//                emxArray_creal_T *beta1
//                emxArray_creal_T *V
// Return Type  : void
//
void xzggev(emxArray_creal_T *A, int *info, emxArray_creal_T *alpha1,
            emxArray_creal_T *beta1, emxArray_creal_T *V)
{
  int n;
  int i;
  int jcol;
  double anrm;
  bool ilascl;
  bool notdone;
  int jcolp1;
  bool exitg1;
  double anrmto;
  bool guard1 = false;
  double absxk;
  emxArray_int32_T *rscale;
  double ctoc;
  emxArray_int8_T *b_I;
  int ilo;
  int ihi;
  int b_n;
  double stemp_im;
  double cto1;
  double a;
  int jrow;
  int b_i;
  creal_T tmp;
  int j;
  *info = 0;
  n = A->size[0] - 1;
  i = alpha1->size[0];
  alpha1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(alpha1, i);
  jcol = A->size[0];
  for (i = 0; i < jcol; i++) {
    alpha1->data[i].re = 0.0;
    alpha1->data[i].im = 0.0;
  }

  i = beta1->size[0];
  beta1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(beta1, i);
  jcol = A->size[0];
  for (i = 0; i < jcol; i++) {
    beta1->data[i].re = 0.0;
    beta1->data[i].im = 0.0;
  }

  i = V->size[0] * V->size[1];
  V->size[0] = A->size[0];
  V->size[1] = A->size[0];
  emxEnsureCapacity_creal_T(V, i);
  jcol = A->size[0] * A->size[0];
  for (i = 0; i < jcol; i++) {
    V->data[i].re = 0.0;
    V->data[i].im = 0.0;
  }

  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    anrm = 0.0;
    ilascl = (A->size[0] == 0);
    notdone = (A->size[1] == 0);
    if ((!ilascl) && (!notdone)) {
      jcolp1 = 0;
      exitg1 = false;
      while ((!exitg1) && (jcolp1 <= A->size[0] * A->size[1] - 1)) {
        absxk = rt_hypotd_snf(A->data[jcolp1].re, A->data[jcolp1].im);
        if (rtIsNaN(absxk)) {
          anrm = rtNaN;
          exitg1 = true;
        } else {
          if (absxk > anrm) {
            anrm = absxk;
          }

          jcolp1++;
        }
      }
    }

    if (rtIsInf(anrm) || rtIsNaN(anrm)) {
      i = alpha1->size[0];
      alpha1->size[0] = A->size[0];
      emxEnsureCapacity_creal_T(alpha1, i);
      jcol = A->size[0];
      for (i = 0; i < jcol; i++) {
        alpha1->data[i].re = rtNaN;
        alpha1->data[i].im = 0.0;
      }

      i = beta1->size[0];
      beta1->size[0] = A->size[0];
      emxEnsureCapacity_creal_T(beta1, i);
      jcol = A->size[0];
      for (i = 0; i < jcol; i++) {
        beta1->data[i].re = rtNaN;
        beta1->data[i].im = 0.0;
      }

      i = V->size[0] * V->size[1];
      V->size[0] = A->size[0];
      V->size[1] = A->size[0];
      emxEnsureCapacity_creal_T(V, i);
      jcol = A->size[0] * A->size[0];
      for (i = 0; i < jcol; i++) {
        V->data[i].re = rtNaN;
        V->data[i].im = 0.0;
      }
    } else {
      ilascl = false;
      anrmto = anrm;
      guard1 = false;
      if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
        anrmto = 6.7178761075670888E-139;
        ilascl = true;
        guard1 = true;
      } else {
        if (anrm > 1.4885657073574029E+138) {
          anrmto = 1.4885657073574029E+138;
          ilascl = true;
          guard1 = true;
        }
      }

      if (guard1) {
        absxk = anrm;
        ctoc = anrmto;
        notdone = true;
        while (notdone) {
          stemp_im = absxk * 2.0041683600089728E-292;
          cto1 = ctoc / 4.9896007738368E+291;
          if ((stemp_im > ctoc) && (ctoc != 0.0)) {
            a = 2.0041683600089728E-292;
            absxk = stemp_im;
          } else if (cto1 > absxk) {
            a = 4.9896007738368E+291;
            ctoc = cto1;
          } else {
            a = ctoc / absxk;
            notdone = false;
          }

          jcol = A->size[1];
          for (i = 0; i < jcol; i++) {
            jcolp1 = A->size[0];
            for (jrow = 0; jrow < jcolp1; jrow++) {
              A->data[jrow + A->size[0] * i].re *= a;
              A->data[jrow + A->size[0] * i].im *= a;
            }
          }
        }
      }

      emxInit_int32_T(&rscale, 1);
      emxInit_int8_T(&b_I, 2);
      xzggbal(A, &ilo, &ihi, rscale);
      b_n = A->size[0];
      i = b_I->size[0] * b_I->size[1];
      b_I->size[0] = A->size[0];
      b_I->size[1] = A->size[0];
      emxEnsureCapacity_int8_T(b_I, i);
      jcol = A->size[0] * A->size[0];
      for (i = 0; i < jcol; i++) {
        b_I->data[i] = 0;
      }

      if (A->size[0] > 0) {
        for (jcolp1 = 0; jcolp1 < b_n; jcolp1++) {
          b_I->data[jcolp1 + b_I->size[0] * jcolp1] = 1;
        }
      }

      i = V->size[0] * V->size[1];
      V->size[0] = b_I->size[0];
      V->size[1] = b_I->size[1];
      emxEnsureCapacity_creal_T(V, i);
      jcol = b_I->size[0] * b_I->size[1];
      for (i = 0; i < jcol; i++) {
        V->data[i].re = b_I->data[i];
        V->data[i].im = 0.0;
      }

      emxFree_int8_T(&b_I);
      if ((A->size[0] > 1) && (ihi >= ilo + 2)) {
        for (jcol = ilo - 1; jcol + 1 < ihi - 1; jcol++) {
          jcolp1 = jcol + 2;
          for (jrow = ihi - 1; jrow + 1 > jcol + 2; jrow--) {
            xzlartg(A->data[(jrow + A->size[0] * jcol) - 1], A->data[jrow +
                    A->size[0] * jcol], &absxk, &tmp, &A->data[(jrow + A->size[0]
                     * jcol) - 1]);
            A->data[jrow + A->size[0] * jcol].re = 0.0;
            A->data[jrow + A->size[0] * jcol].im = 0.0;
            for (j = jcolp1; j <= b_n; j++) {
              ctoc = absxk * A->data[(jrow + A->size[0] * (j - 1)) - 1].re +
                (tmp.re * A->data[jrow + A->size[0] * (j - 1)].re - tmp.im *
                 A->data[jrow + A->size[0] * (j - 1)].im);
              stemp_im = absxk * A->data[(jrow + A->size[0] * (j - 1)) - 1].im +
                (tmp.re * A->data[jrow + A->size[0] * (j - 1)].im + tmp.im *
                 A->data[jrow + A->size[0] * (j - 1)].re);
              cto1 = A->data[(jrow + A->size[0] * (j - 1)) - 1].im;
              a = A->data[(jrow + A->size[0] * (j - 1)) - 1].re;
              A->data[jrow + A->size[0] * (j - 1)].re = absxk * A->data[jrow +
                A->size[0] * (j - 1)].re - (tmp.re * A->data[(jrow + A->size[0] *
                (j - 1)) - 1].re + tmp.im * A->data[(jrow + A->size[0] * (j - 1))
                - 1].im);
              A->data[jrow + A->size[0] * (j - 1)].im = absxk * A->data[jrow +
                A->size[0] * (j - 1)].im - (tmp.re * cto1 - tmp.im * a);
              A->data[(jrow + A->size[0] * (j - 1)) - 1].re = ctoc;
              A->data[(jrow + A->size[0] * (j - 1)) - 1].im = stemp_im;
            }

            tmp.re = -tmp.re;
            tmp.im = -tmp.im;
            for (b_i = 1; b_i <= ihi; b_i++) {
              ctoc = absxk * A->data[(b_i + A->size[0] * jrow) - 1].re + (tmp.re
                * A->data[(b_i + A->size[0] * (jrow - 1)) - 1].re - tmp.im *
                A->data[(b_i + A->size[0] * (jrow - 1)) - 1].im);
              stemp_im = absxk * A->data[(b_i + A->size[0] * jrow) - 1].im +
                (tmp.re * A->data[(b_i + A->size[0] * (jrow - 1)) - 1].im +
                 tmp.im * A->data[(b_i + A->size[0] * (jrow - 1)) - 1].re);
              cto1 = A->data[(b_i + A->size[0] * jrow) - 1].im;
              a = A->data[(b_i + A->size[0] * jrow) - 1].re;
              A->data[(b_i + A->size[0] * (jrow - 1)) - 1].re = absxk * A->data
                [(b_i + A->size[0] * (jrow - 1)) - 1].re - (tmp.re * A->data
                [(b_i + A->size[0] * jrow) - 1].re + tmp.im * A->data[(b_i +
                A->size[0] * jrow) - 1].im);
              A->data[(b_i + A->size[0] * (jrow - 1)) - 1].im = absxk * A->data
                [(b_i + A->size[0] * (jrow - 1)) - 1].im - (tmp.re * cto1 -
                tmp.im * a);
              A->data[(b_i + A->size[0] * jrow) - 1].re = ctoc;
              A->data[(b_i + A->size[0] * jrow) - 1].im = stemp_im;
            }

            for (b_i = 1; b_i <= b_n; b_i++) {
              ctoc = absxk * V->data[(b_i + V->size[0] * jrow) - 1].re + (tmp.re
                * V->data[(b_i + V->size[0] * (jrow - 1)) - 1].re - tmp.im *
                V->data[(b_i + V->size[0] * (jrow - 1)) - 1].im);
              stemp_im = absxk * V->data[(b_i + V->size[0] * jrow) - 1].im +
                (tmp.re * V->data[(b_i + V->size[0] * (jrow - 1)) - 1].im +
                 tmp.im * V->data[(b_i + V->size[0] * (jrow - 1)) - 1].re);
              cto1 = V->data[(b_i + V->size[0] * jrow) - 1].re;
              V->data[(b_i + V->size[0] * (jrow - 1)) - 1].re = absxk * V->data
                [(b_i + V->size[0] * (jrow - 1)) - 1].re - (tmp.re * V->data
                [(b_i + V->size[0] * jrow) - 1].re + tmp.im * V->data[(b_i +
                V->size[0] * jrow) - 1].im);
              V->data[(b_i + V->size[0] * (jrow - 1)) - 1].im = absxk * V->data
                [(b_i + V->size[0] * (jrow - 1)) - 1].im - (tmp.re * V->data
                [(b_i + V->size[0] * jrow) - 1].im - tmp.im * cto1);
              V->data[(b_i + V->size[0] * jrow) - 1].re = ctoc;
              V->data[(b_i + V->size[0] * jrow) - 1].im = stemp_im;
            }
          }
        }
      }

      xzhgeqz(A, ilo, ihi, V, info, alpha1, beta1);
      if (*info == 0) {
        xztgevc(A, V);
        b_n = V->size[0];
        jcol = V->size[1] - 1;
        if (ilo > 1) {
          for (b_i = ilo - 2; b_i + 1 >= 1; b_i--) {
            jcolp1 = rscale->data[b_i] - 1;
            if (rscale->data[b_i] != b_i + 1) {
              for (j = 0; j <= jcol; j++) {
                tmp = V->data[b_i + V->size[0] * j];
                V->data[b_i + V->size[0] * j] = V->data[jcolp1 + V->size[0] * j];
                V->data[jcolp1 + V->size[0] * j] = tmp;
              }
            }
          }
        }

        if (ihi < b_n) {
          i = ihi + 1;
          for (b_i = i; b_i <= b_n; b_i++) {
            jrow = rscale->data[b_i - 1];
            if (jrow != b_i) {
              for (j = 0; j <= jcol; j++) {
                tmp = V->data[(b_i + V->size[0] * j) - 1];
                V->data[(b_i + V->size[0] * j) - 1] = V->data[(jrow + V->size[0]
                  * j) - 1];
                V->data[(jrow + V->size[0] * j) - 1] = tmp;
              }
            }
          }
        }

        for (jcolp1 = 0; jcolp1 <= n; jcolp1++) {
          absxk = std::abs(V->data[V->size[0] * jcolp1].re) + std::abs(V->data
            [V->size[0] * jcolp1].im);
          if (n + 1 > 1) {
            for (jcol = 0; jcol < n; jcol++) {
              ctoc = std::abs(V->data[(jcol + V->size[0] * jcolp1) + 1].re) +
                std::abs(V->data[(jcol + V->size[0] * jcolp1) + 1].im);
              if (ctoc > absxk) {
                absxk = ctoc;
              }
            }
          }

          if (absxk >= 6.7178761075670888E-139) {
            absxk = 1.0 / absxk;
            for (jcol = 0; jcol <= n; jcol++) {
              V->data[jcol + V->size[0] * jcolp1].re *= absxk;
              V->data[jcol + V->size[0] * jcolp1].im *= absxk;
            }
          }
        }

        if (ilascl) {
          notdone = true;
          while (notdone) {
            stemp_im = anrmto * 2.0041683600089728E-292;
            cto1 = anrm / 4.9896007738368E+291;
            if ((stemp_im > anrm) && (anrm != 0.0)) {
              a = 2.0041683600089728E-292;
              anrmto = stemp_im;
            } else if (cto1 > anrmto) {
              a = 4.9896007738368E+291;
              anrm = cto1;
            } else {
              a = anrm / anrmto;
              notdone = false;
            }

            jcol = alpha1->size[0];
            for (i = 0; i < jcol; i++) {
              alpha1->data[i].re *= a;
              alpha1->data[i].im *= a;
            }
          }
        }
      }

      emxFree_int32_T(&rscale);
    }
  }
}

//
// File trailer for xzggev.cpp
//
// [EOF]
//
