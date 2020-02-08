/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xztgevc.cpp
 *
 * Code generation for function 'xztgevc'
 *
 */

/* Include files */
#include "xztgevc.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"
#include <cmath>

/* Function Definitions */
void xztgevc(const emxArray_creal_T *A, emxArray_creal_T *V)
{
  emxArray_creal_T *work1;
  int n;
  int i;
  int loop_ub;
  emxArray_creal_T *work2;
  emxArray_real_T *rworka;
  double SMALL;
  double BIG;
  double BIGNUM;
  double anorm;
  int j;
  double xmx;
  double d_re;
  double ascale;
  int je;
  int b_je;
  double temp;
  double salpha_re;
  double salpha_im;
  double acoeff;
  bool lscalea;
  double z;
  bool lscaleb;
  double scale;
  int jr;
  double dmin;
  double d;
  double b_j;
  int d_re_tmp;
  int b_d_re_tmp;
  double d_im;
  double brm;
  emxInit_creal_T(&work1, 1);
  n = A->size[0] - 1;
  i = work1->size[0];
  work1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(work1, i);
  loop_ub = A->size[0];
  for (i = 0; i < loop_ub; i++) {
    work1->data[i].re = 0.0;
    work1->data[i].im = 0.0;
  }

  emxInit_creal_T(&work2, 1);
  i = work2->size[0];
  work2->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(work2, i);
  loop_ub = A->size[0];
  for (i = 0; i < loop_ub; i++) {
    work2->data[i].re = 0.0;
    work2->data[i].im = 0.0;
  }

  emxInit_real_T(&rworka, 1);
  SMALL = 2.2250738585072014E-308 * static_cast<double>(A->size[0]) /
    2.2204460492503131E-16;
  BIG = 1.0 / SMALL;
  BIGNUM = 1.0 / (2.2250738585072014E-308 * static_cast<double>(A->size[0]));
  i = rworka->size[0];
  rworka->size[0] = A->size[0];
  emxEnsureCapacity_real_T(rworka, i);
  loop_ub = A->size[0];
  for (i = 0; i < loop_ub; i++) {
    rworka->data[i] = 0.0;
  }

  anorm = std::abs(A->data[0].re) + std::abs(A->data[0].im);
  i = A->size[0];
  for (j = 0; j <= i - 2; j++) {
    for (loop_ub = 0; loop_ub <= j; loop_ub++) {
      rworka->data[j + 1] += std::abs(A->data[loop_ub + A->size[0] * (j + 1)].re)
        + std::abs(A->data[loop_ub + A->size[0] * (j + 1)].im);
    }

    d_re = rworka->data[j + 1] + (std::abs(A->data[(j + A->size[0] * (j + 1)) +
      1].re) + std::abs(A->data[(j + A->size[0] * (j + 1)) + 1].im));
    if (d_re > anorm) {
      anorm = d_re;
    }
  }

  xmx = anorm;
  if (2.2250738585072014E-308 > anorm) {
    xmx = 2.2250738585072014E-308;
  }

  ascale = 1.0 / xmx;
  i = static_cast<int>((((-1.0 - static_cast<double>(A->size[0])) + 1.0) / -1.0));
  for (je = 0; je < i; je++) {
    b_je = n - je;
    xmx = (std::abs(A->data[b_je + A->size[0] * b_je].re) + std::abs(A->
            data[b_je + A->size[0] * b_je].im)) * ascale;
    if (1.0 > xmx) {
      xmx = 1.0;
    }

    temp = 1.0 / xmx;
    salpha_re = ascale * (temp * A->data[b_je + A->size[0] * b_je].re);
    salpha_im = ascale * (temp * A->data[b_je + A->size[0] * b_je].im);
    acoeff = temp * ascale;
    if ((temp >= 2.2250738585072014E-308) && (acoeff < SMALL)) {
      lscalea = true;
    } else {
      lscalea = false;
    }

    z = std::abs(salpha_re) + std::abs(salpha_im);
    if ((z >= 2.2250738585072014E-308) && (z < SMALL)) {
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
      d_re = SMALL / z;
      if (d_re > scale) {
        scale = d_re;
      }
    }

    if (lscalea || lscaleb) {
      xmx = acoeff;
      if (1.0 > acoeff) {
        xmx = 1.0;
      }

      if (z > xmx) {
        xmx = z;
      }

      d_re = 1.0 / (2.2250738585072014E-308 * xmx);
      if (d_re < scale) {
        scale = d_re;
      }

      if (lscalea) {
        acoeff = ascale * (scale * temp);
      } else {
        acoeff *= scale;
      }

      salpha_re *= scale;
      salpha_im *= scale;
    }

    for (jr = 0; jr <= n; jr++) {
      work1->data[jr].re = 0.0;
      work1->data[jr].im = 0.0;
    }

    work1->data[b_je].re = 1.0;
    work1->data[b_je].im = 0.0;
    dmin = 2.2204460492503131E-16 * acoeff * anorm;
    d_re = 2.2204460492503131E-16 * (std::abs(salpha_re) + std::abs(salpha_im));
    if (d_re > dmin) {
      dmin = d_re;
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
    d = static_cast<double>((b_je + 1)) - 1.0;
    loop_ub = static_cast<int>((((-1.0 - d) + 1.0) / -1.0));
    for (j = 0; j < loop_ub; j++) {
      b_j = d + -static_cast<double>(j);
      d_re_tmp = static_cast<int>(b_j);
      b_d_re_tmp = d_re_tmp - 1;
      d_re = acoeff * A->data[b_d_re_tmp + A->size[0] * b_d_re_tmp].re -
        salpha_re;
      d_im = acoeff * A->data[b_d_re_tmp + A->size[0] * b_d_re_tmp].im -
        salpha_im;
      if (std::abs(d_re) + std::abs(d_im) <= dmin) {
        d_re = dmin;
        d_im = 0.0;
      }

      brm = std::abs(d_re);
      scale = std::abs(d_im);
      xmx = brm + scale;
      if (xmx < 1.0) {
        z = std::abs(work1->data[b_d_re_tmp].re) + std::abs(work1->
          data[b_d_re_tmp].im);
        if (z >= BIGNUM * xmx) {
          temp = 1.0 / z;
          for (jr = 0; jr <= b_je; jr++) {
            work1->data[jr].re *= temp;
            work1->data[jr].im *= temp;
          }
        }
      }

      if (d_im == 0.0) {
        if (-work1->data[b_d_re_tmp].im == 0.0) {
          scale = -work1->data[b_d_re_tmp].re / d_re;
          xmx = 0.0;
        } else if (-work1->data[b_d_re_tmp].re == 0.0) {
          scale = 0.0;
          xmx = -work1->data[b_d_re_tmp].im / d_re;
        } else {
          scale = -work1->data[b_d_re_tmp].re / d_re;
          xmx = -work1->data[b_d_re_tmp].im / d_re;
        }
      } else if (d_re == 0.0) {
        if (-work1->data[b_d_re_tmp].re == 0.0) {
          scale = -work1->data[b_d_re_tmp].im / d_im;
          xmx = 0.0;
        } else if (-work1->data[b_d_re_tmp].im == 0.0) {
          scale = 0.0;
          xmx = -(-work1->data[b_d_re_tmp].re / d_im);
        } else {
          scale = -work1->data[b_d_re_tmp].im / d_im;
          xmx = -(-work1->data[b_d_re_tmp].re / d_im);
        }
      } else if (brm > scale) {
        z = d_im / d_re;
        xmx = d_re + z * d_im;
        scale = (-work1->data[b_d_re_tmp].re + z * -work1->data[b_d_re_tmp].im) /
          xmx;
        xmx = (-work1->data[b_d_re_tmp].im - z * -work1->data[b_d_re_tmp].re) /
          xmx;
      } else if (scale == brm) {
        if (d_re > 0.0) {
          z = 0.5;
        } else {
          z = -0.5;
        }

        if (d_im > 0.0) {
          xmx = 0.5;
        } else {
          xmx = -0.5;
        }

        scale = (-work1->data[b_d_re_tmp].re * z + -work1->data[b_d_re_tmp].im *
                 xmx) / brm;
        xmx = (-work1->data[b_d_re_tmp].im * z - -work1->data[b_d_re_tmp].re *
               xmx) / brm;
      } else {
        z = d_re / d_im;
        xmx = d_im + z * d_re;
        scale = (z * -work1->data[b_d_re_tmp].re + -work1->data[b_d_re_tmp].im) /
          xmx;
        xmx = (z * -work1->data[b_d_re_tmp].im - (-work1->data[b_d_re_tmp].re)) /
          xmx;
      }

      work1->data[b_d_re_tmp].re = scale;
      work1->data[b_d_re_tmp].im = xmx;
      if (b_j > 1.0) {
        if (std::abs(work1->data[b_d_re_tmp].re) + std::abs(work1->
             data[b_d_re_tmp].im) > 1.0) {
          temp = 1.0 / (std::abs(work1->data[b_d_re_tmp].re) + std::abs
                        (work1->data[b_d_re_tmp].im));
          if (acoeff * rworka->data[b_d_re_tmp] >= BIGNUM * temp) {
            for (jr = 0; jr <= b_je; jr++) {
              work1->data[jr].re *= temp;
              work1->data[jr].im *= temp;
            }
          }
        }

        d_re = acoeff * work1->data[b_d_re_tmp].re;
        d_im = acoeff * work1->data[b_d_re_tmp].im;
        for (jr = 0; jr <= d_re_tmp - 2; jr++) {
          work1->data[jr].re += d_re * A->data[jr + A->size[0] * b_d_re_tmp].re
            - d_im * A->data[jr + A->size[0] * b_d_re_tmp].im;
          work1->data[jr].im += d_re * A->data[jr + A->size[0] * b_d_re_tmp].im
            + d_im * A->data[jr + A->size[0] * b_d_re_tmp].re;
        }
      }
    }

    for (jr = 0; jr <= n; jr++) {
      work2->data[jr].re = 0.0;
      work2->data[jr].im = 0.0;
    }

    for (loop_ub = 0; loop_ub <= b_je; loop_ub++) {
      for (jr = 0; jr <= n; jr++) {
        work2->data[jr].re += V->data[jr + V->size[0] * loop_ub].re *
          work1->data[loop_ub].re - V->data[jr + V->size[0] * loop_ub].im *
          work1->data[loop_ub].im;
        work2->data[jr].im += V->data[jr + V->size[0] * loop_ub].re *
          work1->data[loop_ub].im + V->data[jr + V->size[0] * loop_ub].im *
          work1->data[loop_ub].re;
      }
    }

    xmx = std::abs(work2->data[0].re) + std::abs(work2->data[0].im);
    if (n + 1 > 1) {
      for (jr = 0; jr < n; jr++) {
        d_re = std::abs(work2->data[jr + 1].re) + std::abs(work2->data[jr + 1].
          im);
        if (d_re > xmx) {
          xmx = d_re;
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

/* End of code generation (xztgevc.cpp) */
