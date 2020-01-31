//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xztgevc.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xztgevc.h"
#include "ADMMGainDesign3D_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_creal_T *A
//                emxArray_creal_T *V
// Return Type  : void
//
void xztgevc(const emxArray_creal_T *A, emxArray_creal_T *V)
{
  emxArray_creal_T *work1;
  int n;
  int i59;
  int loop_ub;
  emxArray_creal_T *work2;
  emxArray_real_T *rworka;
  double SMALL;
  double BIG;
  double BIGNUM;
  double anorm;
  int j;
  double xmx;
  double y;
  double ascale;
  int je;
  int b_je;
  double temp;
  double temp_re;
  double salpha_re;
  double salpha_im;
  double acoeff;
  bool lscalea;
  bool lscaleb;
  double scale;
  double acoefa;
  int jr;
  double dmin;
  double d2;
  int i60;
  double b_j;
  int d_re_tmp_tmp;
  double d_re;
  double d_im;
  emxInit_creal_T(&work1, 1);
  n = A->size[0] - 1;
  i59 = work1->size[0];
  work1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(work1, i59);
  loop_ub = A->size[0];
  for (i59 = 0; i59 < loop_ub; i59++) {
    work1->data[i59].re = 0.0;
    work1->data[i59].im = 0.0;
  }

  emxInit_creal_T(&work2, 1);
  i59 = work2->size[0];
  work2->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(work2, i59);
  loop_ub = A->size[0];
  for (i59 = 0; i59 < loop_ub; i59++) {
    work2->data[i59].re = 0.0;
    work2->data[i59].im = 0.0;
  }

  emxInit_real_T(&rworka, 1);
  SMALL = 2.2250738585072014E-308 * (double)A->size[0] / 2.2204460492503131E-16;
  BIG = 1.0 / SMALL;
  BIGNUM = 1.0 / (2.2250738585072014E-308 * (double)A->size[0]);
  i59 = rworka->size[0];
  rworka->size[0] = A->size[0];
  emxEnsureCapacity_real_T(rworka, i59);
  loop_ub = A->size[0];
  for (i59 = 0; i59 < loop_ub; i59++) {
    rworka->data[i59] = 0.0;
  }

  anorm = std::abs(A->data[0].re) + std::abs(A->data[0].im);
  i59 = A->size[0];
  for (j = 0; j <= i59 - 2; j++) {
    for (loop_ub = 0; loop_ub <= j; loop_ub++) {
      rworka->data[j + 1] += std::abs(A->data[loop_ub + A->size[0] * (j + 1)].re)
        + std::abs(A->data[loop_ub + A->size[0] * (j + 1)].im);
    }

    y = rworka->data[j + 1] + (std::abs(A->data[(j + A->size[0] * (j + 1)) + 1].
      re) + std::abs(A->data[(j + A->size[0] * (j + 1)) + 1].im));
    if (y > anorm) {
      anorm = y;
    }
  }

  xmx = anorm;
  if (2.2250738585072014E-308 > anorm) {
    xmx = 2.2250738585072014E-308;
  }

  ascale = 1.0 / xmx;
  i59 = (int)((1.0 + (-1.0 - (double)A->size[0])) / -1.0);
  for (je = 0; je < i59; je++) {
    b_je = n - je;
    xmx = (std::abs(A->data[b_je + A->size[0] * b_je].re) + std::abs(A->
            data[b_je + A->size[0] * b_je].im)) * ascale;
    if (1.0 > xmx) {
      xmx = 1.0;
    }

    temp = 1.0 / xmx;
    temp_re = temp * A->data[b_je + A->size[0] * b_je].re;
    xmx = temp * A->data[b_je + A->size[0] * b_je].im;
    salpha_re = ascale * temp_re;
    salpha_im = ascale * xmx;
    acoeff = temp * ascale;
    if ((temp >= 2.2250738585072014E-308) && (std::abs(acoeff) < SMALL)) {
      lscalea = true;
    } else {
      lscalea = false;
    }

    temp_re = std::abs(salpha_re) + std::abs(salpha_im);
    if ((temp_re >= 2.2250738585072014E-308) && (temp_re < SMALL)) {
      lscaleb = true;
    } else {
      lscaleb = false;
    }

    scale = 1.0;
    if (lscalea) {
      xmx = anorm;
      if (BIG < anorm) {
        xmx = BIG;
      }

      scale = SMALL / temp * xmx;
    }

    if (lscaleb) {
      xmx = 1.0;
      if (BIG < 1.0) {
        xmx = BIG;
      }

      y = SMALL / temp_re * xmx;
      if (y > scale) {
        scale = y;
      }
    }

    if (lscalea || lscaleb) {
      xmx = std::abs(acoeff);
      if (1.0 > xmx) {
        xmx = 1.0;
      }

      if (temp_re > xmx) {
        xmx = temp_re;
      }

      y = 1.0 / (2.2250738585072014E-308 * xmx);
      if (y < scale) {
        scale = y;
      }

      if (lscalea) {
        acoeff = ascale * (scale * temp);
      } else {
        acoeff *= scale;
      }

      salpha_re *= scale;
      salpha_im *= scale;
    }

    acoefa = std::abs(acoeff);
    for (jr = 0; jr <= n; jr++) {
      work1->data[jr].re = 0.0;
      work1->data[jr].im = 0.0;
    }

    work1->data[b_je].re = 1.0;
    work1->data[b_je].im = 0.0;
    dmin = 2.2204460492503131E-16 * acoefa * anorm;
    y = 2.2204460492503131E-16 * (std::abs(salpha_re) + std::abs(salpha_im));
    if (y > dmin) {
      dmin = y;
    }

    if (2.2250738585072014E-308 > dmin) {
      dmin = 2.2250738585072014E-308;
    }

    for (jr = 0; jr < b_je; jr++) {
      work1->data[jr].re = acoeff * A->data[jr + A->size[0] * b_je].re;
      work1->data[jr].im = acoeff * A->data[jr + A->size[0] * b_je].im;
    }

    work1->data[b_je].re = 1.0;
    work1->data[b_je].im = 0.0;
    d2 = (double)(b_je + 1) - 1.0;
    i60 = (int)((1.0 + (-1.0 - d2)) / -1.0);
    for (j = 0; j < i60; j++) {
      b_j = d2 + -(double)j;
      loop_ub = (int)b_j;
      d_re_tmp_tmp = loop_ub - 1;
      d_re = acoeff * A->data[(loop_ub + A->size[0] * d_re_tmp_tmp) - 1].re -
        salpha_re;
      d_im = acoeff * A->data[((int)b_j + A->size[0] * ((int)b_j - 1)) - 1].im -
        salpha_im;
      if (std::abs(d_re) + std::abs(d_im) <= dmin) {
        d_re = dmin;
        d_im = 0.0;
      }

      xmx = std::abs(d_re) + std::abs(d_im);
      if ((xmx < 1.0) && (std::abs(work1->data[d_re_tmp_tmp].re) + std::abs
                          (work1->data[d_re_tmp_tmp].im) >= BIGNUM * xmx)) {
        temp = 1.0 / (std::abs(work1->data[d_re_tmp_tmp].re) + std::abs
                      (work1->data[d_re_tmp_tmp].im));
        for (jr = 0; jr <= b_je; jr++) {
          work1->data[jr].re *= temp;
          work1->data[jr].im *= temp;
        }
      }

      y = -work1->data[d_re_tmp_tmp].re;
      temp = -work1->data[d_re_tmp_tmp].im;
      if (d_im == 0.0) {
        if (temp == 0.0) {
          work1->data[d_re_tmp_tmp].re = y / d_re;
          work1->data[d_re_tmp_tmp].im = 0.0;
        } else if (y == 0.0) {
          work1->data[d_re_tmp_tmp].re = 0.0;
          work1->data[d_re_tmp_tmp].im = temp / d_re;
        } else {
          work1->data[d_re_tmp_tmp].re = y / d_re;
          work1->data[d_re_tmp_tmp].im = temp / d_re;
        }
      } else if (d_re == 0.0) {
        if (y == 0.0) {
          work1->data[d_re_tmp_tmp].re = temp / d_im;
          work1->data[d_re_tmp_tmp].im = 0.0;
        } else if (temp == 0.0) {
          work1->data[d_re_tmp_tmp].re = 0.0;
          work1->data[d_re_tmp_tmp].im = -(y / d_im);
        } else {
          work1->data[d_re_tmp_tmp].re = temp / d_im;
          work1->data[d_re_tmp_tmp].im = -(y / d_im);
        }
      } else {
        scale = std::abs(d_re);
        xmx = std::abs(d_im);
        if (scale > xmx) {
          temp_re = d_im / d_re;
          xmx = d_re + temp_re * d_im;
          work1->data[d_re_tmp_tmp].re = (y + temp_re * temp) / xmx;
          work1->data[d_re_tmp_tmp].im = (temp - temp_re * y) / xmx;
        } else if (xmx == scale) {
          if (d_re > 0.0) {
            temp_re = 0.5;
          } else {
            temp_re = -0.5;
          }

          if (d_im > 0.0) {
            xmx = 0.5;
          } else {
            xmx = -0.5;
          }

          work1->data[d_re_tmp_tmp].re = (y * temp_re + temp * xmx) / scale;
          work1->data[d_re_tmp_tmp].im = (temp * temp_re - y * xmx) / scale;
        } else {
          temp_re = d_re / d_im;
          xmx = d_im + temp_re * d_re;
          work1->data[d_re_tmp_tmp].re = (temp_re * y + temp) / xmx;
          work1->data[d_re_tmp_tmp].im = (temp_re * temp - y) / xmx;
        }
      }

      if (b_j > 1.0) {
        if (std::abs(work1->data[(int)b_j - 1].re) + std::abs(work1->data[(int)
             b_j - 1].im) > 1.0) {
          temp = 1.0 / (std::abs(work1->data[(int)b_j - 1].re) + std::abs
                        (work1->data[(int)b_j - 1].im));
          if (acoefa * rworka->data[(int)b_j - 1] >= BIGNUM * temp) {
            for (jr = 0; jr <= b_je; jr++) {
              work1->data[jr].re *= temp;
              work1->data[jr].im *= temp;
            }
          }
        }

        d_re = acoeff * work1->data[(int)b_j - 1].re;
        d_im = acoeff * work1->data[(int)b_j - 1].im;
        loop_ub = (int)b_j;
        for (jr = 0; jr <= loop_ub - 2; jr++) {
          xmx = d_re * A->data[jr + A->size[0] * ((int)b_j - 1)].re - d_im *
            A->data[jr + A->size[0] * ((int)b_j - 1)].im;
          temp_re = d_re * A->data[jr + A->size[0] * ((int)b_j - 1)].im + d_im *
            A->data[jr + A->size[0] * ((int)b_j - 1)].re;
          work1->data[jr].re += xmx;
          work1->data[jr].im += temp_re;
        }
      }
    }

    for (jr = 0; jr <= n; jr++) {
      work2->data[jr].re = 0.0;
      work2->data[jr].im = 0.0;
    }

    for (loop_ub = 0; loop_ub <= b_je; loop_ub++) {
      for (jr = 0; jr <= n; jr++) {
        xmx = V->data[jr + V->size[0] * loop_ub].re * work1->data[loop_ub].re -
          V->data[jr + V->size[0] * loop_ub].im * work1->data[loop_ub].im;
        temp_re = V->data[jr + V->size[0] * loop_ub].re * work1->data[loop_ub].
          im + V->data[jr + V->size[0] * loop_ub].im * work1->data[loop_ub].re;
        work2->data[jr].re += xmx;
        work2->data[jr].im += temp_re;
      }
    }

    xmx = std::abs(work2->data[0].re) + std::abs(work2->data[0].im);
    if (n + 1 > 1) {
      for (jr = 0; jr < n; jr++) {
        y = std::abs(work2->data[jr + 1].re) + std::abs(work2->data[jr + 1].im);
        if (y > xmx) {
          xmx = y;
        }
      }
    }

    if (xmx > 2.2250738585072014E-308) {
      temp = 1.0 / xmx;
      for (jr = 0; jr <= n; jr++) {
        V->data[jr + V->size[0] * b_je].re = temp * work2->data[jr].re;
        V->data[jr + V->size[0] * b_je].im = temp * work2->data[jr].im;
      }
    } else {
      for (jr = 0; jr <= n; jr++) {
        V->data[jr + V->size[0] * b_je].re = 0.0;
        V->data[jr + V->size[0] * b_je].im = 0.0;
      }
    }
  }

  emxFree_real_T(&rworka);
  emxFree_creal_T(&work2);
  emxFree_creal_T(&work1);
}

//
// File trailer for xztgevc.cpp
//
// [EOF]
//
