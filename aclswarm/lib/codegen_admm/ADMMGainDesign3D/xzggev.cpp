//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzggev.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xzggev.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "xzlartg.h"
#include "xztgevc.h"
#include "xzhgeqz.h"
#include "xzlascl.h"
#include "schur.h"
#include "ADMMGainDesign3D_rtwutil.h"

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
  int jrow;
  int nzcount;
  double anrm;
  bool ilascl;
  bool found;
  int ii;
  bool exitg1;
  double anrmto;
  double absxk;
  emxArray_int32_T *rscale;
  int ilo;
  int ihi;
  int exitg3;
  int i;
  emxArray_int8_T *b_I;
  int j;
  int b_n;
  bool exitg4;
  creal_T atmp;
  int exitg2;
  bool b_A;
  creal_T c_A;
  creal_T d_A;
  double c;
  double cto1;
  double stemp_re;
  double stemp_im;
  double A_im;
  double A_re;
  *info = 0;
  n = A->size[0] - 1;
  jrow = alpha1->size[0];
  alpha1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(alpha1, jrow);
  nzcount = A->size[0];
  for (jrow = 0; jrow < nzcount; jrow++) {
    alpha1->data[jrow].re = 0.0;
    alpha1->data[jrow].im = 0.0;
  }

  jrow = beta1->size[0];
  beta1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(beta1, jrow);
  nzcount = A->size[0];
  for (jrow = 0; jrow < nzcount; jrow++) {
    beta1->data[jrow].re = 0.0;
    beta1->data[jrow].im = 0.0;
  }

  jrow = V->size[0] * V->size[1];
  V->size[0] = A->size[0];
  V->size[1] = A->size[0];
  emxEnsureCapacity_creal_T(V, jrow);
  nzcount = A->size[0] * A->size[0];
  for (jrow = 0; jrow < nzcount; jrow++) {
    V->data[jrow].re = 0.0;
    V->data[jrow].im = 0.0;
  }

  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    anrm = 0.0;
    ilascl = (A->size[0] == 0);
    found = (A->size[1] == 0);
    if ((!ilascl) && (!found)) {
      ii = 0;
      exitg1 = false;
      while ((!exitg1) && (ii <= A->size[0] * A->size[1] - 1)) {
        absxk = rt_hypotd_snf(A->data[ii].re, A->data[ii].im);
        if (rtIsNaN(absxk)) {
          anrm = rtNaN;
          exitg1 = true;
        } else {
          if (absxk > anrm) {
            anrm = absxk;
          }

          ii++;
        }
      }
    }

    if (rtIsInf(anrm) || rtIsNaN(anrm)) {
      jrow = alpha1->size[0];
      alpha1->size[0] = A->size[0];
      emxEnsureCapacity_creal_T(alpha1, jrow);
      nzcount = A->size[0];
      for (jrow = 0; jrow < nzcount; jrow++) {
        alpha1->data[jrow].re = rtNaN;
        alpha1->data[jrow].im = 0.0;
      }

      jrow = beta1->size[0];
      beta1->size[0] = A->size[0];
      emxEnsureCapacity_creal_T(beta1, jrow);
      nzcount = A->size[0];
      for (jrow = 0; jrow < nzcount; jrow++) {
        beta1->data[jrow].re = rtNaN;
        beta1->data[jrow].im = 0.0;
      }

      jrow = V->size[0] * V->size[1];
      V->size[0] = A->size[0];
      V->size[1] = A->size[0];
      emxEnsureCapacity_creal_T(V, jrow);
      nzcount = A->size[0] * A->size[0];
      for (jrow = 0; jrow < nzcount; jrow++) {
        V->data[jrow].re = rtNaN;
        V->data[jrow].im = 0.0;
      }
    } else {
      ilascl = false;
      anrmto = anrm;
      if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
        anrmto = 6.7178761075670888E-139;
        ilascl = true;
      } else {
        if (anrm > 1.4885657073574029E+138) {
          anrmto = 1.4885657073574029E+138;
          ilascl = true;
        }
      }

      if (ilascl) {
        xzlascl(anrm, anrmto, A);
      }

      emxInit_int32_T(&rscale, 1);
      jrow = rscale->size[0];
      rscale->size[0] = A->size[0];
      emxEnsureCapacity_int32_T(rscale, jrow);
      nzcount = A->size[0];
      for (jrow = 0; jrow < nzcount; jrow++) {
        rscale->data[jrow] = 1;
      }

      ilo = 1;
      ihi = A->size[0];
      if (A->size[0] <= 1) {
        ihi = 1;
      } else {
        do {
          exitg3 = 0;
          i = 0;
          j = -1;
          found = false;
          ii = ihi;
          exitg1 = false;
          while ((!exitg1) && (ii > 0)) {
            nzcount = 0;
            i = ii;
            j = ihi - 1;
            jrow = 0;
            exitg4 = false;
            while ((!exitg4) && (jrow <= ihi - 1)) {
              b_A = ((A->data[(ii + A->size[0] * jrow) - 1].re != 0.0) ||
                     (A->data[(ii + A->size[0] * jrow) - 1].im != 0.0));
              if (b_A || (ii == jrow + 1)) {
                if (nzcount == 0) {
                  j = jrow;
                  nzcount = 1;
                  jrow++;
                } else {
                  nzcount = 2;
                  exitg4 = true;
                }
              } else {
                jrow++;
              }
            }

            if (nzcount < 2) {
              found = true;
              exitg1 = true;
            } else {
              ii--;
            }
          }

          if (!found) {
            exitg3 = 2;
          } else {
            b_n = A->size[0];
            if (i != ihi) {
              for (ii = 1; ii <= b_n; ii++) {
                atmp = A->data[(i + A->size[0] * (ii - 1)) - 1];
                A->data[(i + A->size[0] * (ii - 1)) - 1] = A->data[(ihi +
                  A->size[0] * (ii - 1)) - 1];
                A->data[(ihi + A->size[0] * (ii - 1)) - 1] = atmp;
              }
            }

            if (j + 1 != ihi) {
              for (ii = 0; ii < ihi; ii++) {
                atmp = A->data[ii + A->size[0] * j];
                A->data[ii + A->size[0] * j] = A->data[ii + A->size[0] * (ihi -
                  1)];
                A->data[ii + A->size[0] * (ihi - 1)] = atmp;
              }
            }

            rscale->data[ihi - 1] = j + 1;
            ihi--;
            if (ihi == 1) {
              rscale->data[0] = 1;
              exitg3 = 1;
            }
          }
        } while (exitg3 == 0);

        if (exitg3 == 1) {
        } else {
          do {
            exitg2 = 0;
            i = 0;
            j = 0;
            found = false;
            jrow = ilo;
            exitg1 = false;
            while ((!exitg1) && (jrow <= ihi)) {
              nzcount = 0;
              i = ihi;
              j = jrow;
              ii = ilo;
              exitg4 = false;
              while ((!exitg4) && (ii <= ihi)) {
                b_A = ((A->data[(ii + A->size[0] * (jrow - 1)) - 1].re != 0.0) ||
                       (A->data[(ii + A->size[0] * (jrow - 1)) - 1].im != 0.0));
                if (b_A || (ii == jrow)) {
                  if (nzcount == 0) {
                    i = ii;
                    nzcount = 1;
                    ii++;
                  } else {
                    nzcount = 2;
                    exitg4 = true;
                  }
                } else {
                  ii++;
                }
              }

              if (nzcount < 2) {
                found = true;
                exitg1 = true;
              } else {
                jrow++;
              }
            }

            if (!found) {
              exitg2 = 1;
            } else {
              b_n = A->size[0];
              if (i != ilo) {
                for (ii = ilo; ii <= b_n; ii++) {
                  atmp = A->data[(i + A->size[0] * (ii - 1)) - 1];
                  A->data[(i + A->size[0] * (ii - 1)) - 1] = A->data[(ilo +
                    A->size[0] * (ii - 1)) - 1];
                  A->data[(ilo + A->size[0] * (ii - 1)) - 1] = atmp;
                }
              }

              if (j != ilo) {
                for (ii = 0; ii < ihi; ii++) {
                  atmp = A->data[ii + A->size[0] * (j - 1)];
                  A->data[ii + A->size[0] * (j - 1)] = A->data[ii + A->size[0] *
                    (ilo - 1)];
                  A->data[ii + A->size[0] * (ilo - 1)] = atmp;
                }
              }

              rscale->data[ilo - 1] = j;
              ilo++;
              if (ilo == ihi) {
                rscale->data[ilo - 1] = ilo;
                exitg2 = 1;
              }
            }
          } while (exitg2 == 0);
        }
      }

      emxInit_int8_T(&b_I, 2);
      b_n = A->size[0];
      jrow = b_I->size[0] * b_I->size[1];
      b_I->size[0] = A->size[0];
      b_I->size[1] = A->size[0];
      emxEnsureCapacity_int8_T(b_I, jrow);
      nzcount = A->size[0] * A->size[0];
      for (jrow = 0; jrow < nzcount; jrow++) {
        b_I->data[jrow] = 0;
      }

      if (A->size[0] > 0) {
        for (ii = 0; ii < b_n; ii++) {
          b_I->data[ii + b_I->size[0] * ii] = 1;
        }
      }

      jrow = V->size[0] * V->size[1];
      V->size[0] = b_I->size[0];
      V->size[1] = b_I->size[1];
      emxEnsureCapacity_creal_T(V, jrow);
      nzcount = b_I->size[0] * b_I->size[1];
      for (jrow = 0; jrow < nzcount; jrow++) {
        V->data[jrow].re = b_I->data[jrow];
        V->data[jrow].im = 0.0;
      }

      emxFree_int8_T(&b_I);
      if ((A->size[0] > 1) && (ihi >= ilo + 2)) {
        for (ii = ilo - 1; ii + 1 < ihi - 1; ii++) {
          nzcount = ii + 2;
          for (jrow = ihi - 1; jrow + 1 > ii + 2; jrow--) {
            c_A = A->data[(jrow + A->size[0] * ii) - 1];
            d_A = A->data[jrow + A->size[0] * ii];
            xzlartg(c_A, d_A, &c, &atmp, &A->data[(jrow + A->size[0] * ii) - 1]);
            A->data[jrow + A->size[0] * ii].re = 0.0;
            A->data[jrow + A->size[0] * ii].im = 0.0;
            for (j = nzcount; j <= b_n; j++) {
              absxk = atmp.re * A->data[jrow + A->size[0] * (j - 1)].re -
                atmp.im * A->data[jrow + A->size[0] * (j - 1)].im;
              cto1 = atmp.re * A->data[jrow + A->size[0] * (j - 1)].im + atmp.im
                * A->data[jrow + A->size[0] * (j - 1)].re;
              stemp_re = c * A->data[(jrow + A->size[0] * (j - 1)) - 1].re +
                absxk;
              stemp_im = c * A->data[(jrow + A->size[0] * (j - 1)) - 1].im +
                cto1;
              absxk = A->data[(jrow + A->size[0] * (j - 1)) - 1].re;
              cto1 = A->data[(jrow + A->size[0] * (j - 1)) - 1].im;
              A_im = A->data[(jrow + A->size[0] * (j - 1)) - 1].im;
              A_re = A->data[(jrow + A->size[0] * (j - 1)) - 1].re;
              A->data[jrow + A->size[0] * (j - 1)].re = c * A->data[jrow +
                A->size[0] * (j - 1)].re - (atmp.re * absxk + atmp.im * cto1);
              A->data[jrow + A->size[0] * (j - 1)].im = c * A->data[jrow +
                A->size[0] * (j - 1)].im - (atmp.re * A_im - atmp.im * A_re);
              A->data[(jrow + A->size[0] * (j - 1)) - 1].re = stemp_re;
              A->data[(jrow + A->size[0] * (j - 1)) - 1].im = stemp_im;
            }

            atmp.re = -atmp.re;
            atmp.im = -atmp.im;
            for (i = 1; i <= ihi; i++) {
              absxk = atmp.re * A->data[(i + A->size[0] * (jrow - 1)) - 1].re -
                atmp.im * A->data[(i + A->size[0] * (jrow - 1)) - 1].im;
              cto1 = atmp.re * A->data[(i + A->size[0] * (jrow - 1)) - 1].im +
                atmp.im * A->data[(i + A->size[0] * (jrow - 1)) - 1].re;
              stemp_re = c * A->data[(i + A->size[0] * jrow) - 1].re + absxk;
              stemp_im = c * A->data[(i + A->size[0] * jrow) - 1].im + cto1;
              absxk = A->data[(i + A->size[0] * jrow) - 1].re;
              cto1 = A->data[(i + A->size[0] * jrow) - 1].im;
              A_im = A->data[(i + A->size[0] * jrow) - 1].im;
              A_re = A->data[(i + A->size[0] * jrow) - 1].re;
              A->data[(i + A->size[0] * (jrow - 1)) - 1].re = c * A->data[(i +
                A->size[0] * (jrow - 1)) - 1].re - (atmp.re * absxk + atmp.im *
                cto1);
              A->data[(i + A->size[0] * (jrow - 1)) - 1].im = c * A->data[(i +
                A->size[0] * (jrow - 1)) - 1].im - (atmp.re * A_im - atmp.im *
                A_re);
              A->data[(i + A->size[0] * jrow) - 1].re = stemp_re;
              A->data[(i + A->size[0] * jrow) - 1].im = stemp_im;
            }

            for (i = 1; i <= b_n; i++) {
              absxk = atmp.re * V->data[(i + V->size[0] * (jrow - 1)) - 1].re -
                atmp.im * V->data[(i + V->size[0] * (jrow - 1)) - 1].im;
              cto1 = atmp.re * V->data[(i + V->size[0] * (jrow - 1)) - 1].im +
                atmp.im * V->data[(i + V->size[0] * (jrow - 1)) - 1].re;
              stemp_re = c * V->data[(i + V->size[0] * jrow) - 1].re + absxk;
              stemp_im = c * V->data[(i + V->size[0] * jrow) - 1].im + cto1;
              absxk = V->data[(i + V->size[0] * jrow) - 1].re;
              cto1 = V->data[(i + V->size[0] * jrow) - 1].im;
              A_im = V->data[(i + V->size[0] * jrow) - 1].im;
              A_re = V->data[(i + V->size[0] * jrow) - 1].re;
              V->data[(i + V->size[0] * (jrow - 1)) - 1].re = c * V->data[(i +
                V->size[0] * (jrow - 1)) - 1].re - (atmp.re * absxk + atmp.im *
                cto1);
              V->data[(i + V->size[0] * (jrow - 1)) - 1].im = c * V->data[(i +
                V->size[0] * (jrow - 1)) - 1].im - (atmp.re * A_im - atmp.im *
                A_re);
              V->data[(i + V->size[0] * jrow) - 1].re = stemp_re;
              V->data[(i + V->size[0] * jrow) - 1].im = stemp_im;
            }
          }
        }
      }

      xzhgeqz(A, ilo, ihi, V, info, alpha1, beta1);
      if (*info == 0) {
        xztgevc(A, V);
        b_n = V->size[0];
        nzcount = V->size[1] - 1;
        if (ilo > 1) {
          for (i = ilo - 2; i + 1 >= 1; i--) {
            ii = rscale->data[i] - 1;
            if (rscale->data[i] != i + 1) {
              for (j = 0; j <= nzcount; j++) {
                atmp = V->data[i + V->size[0] * j];
                V->data[i + V->size[0] * j] = V->data[ii + V->size[0] * j];
                V->data[ii + V->size[0] * j] = atmp;
              }
            }
          }
        }

        if (ihi < b_n) {
          jrow = ihi + 1;
          for (i = jrow; i <= b_n; i++) {
            ii = rscale->data[i - 1] - 1;
            if (rscale->data[i - 1] != i) {
              for (j = 0; j <= nzcount; j++) {
                atmp = V->data[(i + V->size[0] * j) - 1];
                V->data[(i + V->size[0] * j) - 1] = V->data[ii + V->size[0] * j];
                V->data[ii + V->size[0] * j] = atmp;
              }
            }
          }
        }

        for (nzcount = 0; nzcount <= n; nzcount++) {
          absxk = std::abs(V->data[V->size[0] * nzcount].re) + std::abs(V->
            data[V->size[0] * nzcount].im);
          if (n + 1 > 1) {
            for (ii = 0; ii < n; ii++) {
              cto1 = std::abs(V->data[(ii + V->size[0] * nzcount) + 1].re) + std::
                abs(V->data[(ii + V->size[0] * nzcount) + 1].im);
              if (cto1 > absxk) {
                absxk = cto1;
              }
            }
          }

          if (absxk >= 6.7178761075670888E-139) {
            absxk = 1.0 / absxk;
            for (ii = 0; ii <= n; ii++) {
              V->data[ii + V->size[0] * nzcount].re *= absxk;
              V->data[ii + V->size[0] * nzcount].im *= absxk;
            }
          }
        }

        if (ilascl) {
          ilascl = true;
          while (ilascl) {
            absxk = anrmto * 2.0041683600089728E-292;
            cto1 = anrm / 4.9896007738368E+291;
            if ((absxk > anrm) && (anrm != 0.0)) {
              A_im = 2.0041683600089728E-292;
              anrmto = absxk;
            } else if (cto1 > anrmto) {
              A_im = 4.9896007738368E+291;
              anrm = cto1;
            } else {
              A_im = anrm / anrmto;
              ilascl = false;
            }

            jrow = alpha1->size[0];
            emxEnsureCapacity_creal_T(alpha1, jrow);
            nzcount = alpha1->size[0];
            for (jrow = 0; jrow < nzcount; jrow++) {
              alpha1->data[jrow].re *= A_im;
              alpha1->data[jrow].im *= A_im;
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
