//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzhgeqz.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xzhgeqz.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "xzlartg.h"
#include "sqrt.h"

// Function Declarations
static int div_nzp_s32(int numerator, int denominator);

// Function Definitions

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_nzp_s32(int numerator, int denominator)
{
  int quotient;
  unsigned int b_numerator;
  unsigned int b_denominator;
  unsigned int tempAbsQuotient;
  if (numerator < 0) {
    b_numerator = ~(unsigned int)numerator + 1U;
  } else {
    b_numerator = (unsigned int)numerator;
  }

  if (denominator < 0) {
    b_denominator = ~(unsigned int)denominator + 1U;
  } else {
    b_denominator = (unsigned int)denominator;
  }

  tempAbsQuotient = b_numerator / b_denominator;
  if ((numerator < 0) != (denominator < 0)) {
    quotient = -(int)tempAbsQuotient;
  } else {
    quotient = (int)tempAbsQuotient;
  }

  return quotient;
}

//
// Arguments    : emxArray_creal_T *A
//                int ilo
//                int ihi
//                emxArray_creal_T *Z
//                int *info
//                emxArray_creal_T *alpha1
//                emxArray_creal_T *beta1
// Return Type  : void
//
void xzhgeqz(emxArray_creal_T *A, int ilo, int ihi, emxArray_creal_T *Z, int
             *info, emxArray_creal_T *alpha1, emxArray_creal_T *beta1)
{
  bool compz;
  int n;
  int ifirst;
  int loop_ub;
  double eshift_re;
  double eshift_im;
  creal_T ctemp;
  double anorm;
  double scale;
  double reAij;
  double sumsq;
  double b_atol;
  bool firstNonZero;
  int j;
  double ascale;
  int jp1;
  double bscale;
  double imAij;
  bool guard1 = false;
  bool guard2 = false;
  double temp2;
  int istart;
  int ilast;
  int ilastm1;
  int ifrstm;
  int ilastm;
  int iiter;
  bool goto60;
  bool goto70;
  bool goto90;
  int jiter;
  int jm1;
  int exitg1;
  bool b_guard1 = false;
  bool guard3 = false;
  bool exitg2;
  creal_T b_ascale;
  creal_T shift;
  creal_T b_A;
  double ad22_re;
  double ad22_im;
  double t1_re;
  double t1_im;
  *info = -1;
  compz = ((Z->size[0] != 0) && (Z->size[1] != 0));
  if ((A->size[0] == 1) && (A->size[1] == 1)) {
    ihi = 1;
  }

  n = A->size[0];
  ifirst = alpha1->size[0];
  alpha1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(alpha1, ifirst);
  loop_ub = A->size[0];
  for (ifirst = 0; ifirst < loop_ub; ifirst++) {
    alpha1->data[ifirst].re = 0.0;
    alpha1->data[ifirst].im = 0.0;
  }

  ifirst = beta1->size[0];
  beta1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(beta1, ifirst);
  loop_ub = A->size[0];
  for (ifirst = 0; ifirst < loop_ub; ifirst++) {
    beta1->data[ifirst].re = 1.0;
    beta1->data[ifirst].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = 0.0;
  if (ilo <= ihi) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      ifirst = j + 1;
      if (ihi < j + 1) {
        ifirst = ihi;
      }

      for (jp1 = ilo; jp1 <= ifirst; jp1++) {
        reAij = A->data[(jp1 + A->size[0] * (j - 1)) - 1].re;
        imAij = A->data[(jp1 + A->size[0] * (j - 1)) - 1].im;
        if (reAij != 0.0) {
          anorm = std::abs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          anorm = std::abs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    anorm = scale * std::sqrt(sumsq);
  }

  reAij = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (reAij > 2.2250738585072014E-308) {
    b_atol = reAij;
  }

  reAij = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    reAij = anorm;
  }

  ascale = 1.0 / reAij;
  bscale = 1.0 / std::sqrt((double)A->size[0]);
  firstNonZero = true;
  ifirst = ihi + 1;
  for (j = ifirst; j <= n; j++) {
    alpha1->data[j - 1] = A->data[(j + A->size[0] * (j - 1)) - 1];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    if (compz) {
      ifrstm = 1;
      ilastm = n;
    } else {
      ifrstm = ilo;
      ilastm = ihi;
    }

    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 0;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1) - 1) {
        b_guard1 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          b_guard1 = true;
        } else if (std::abs(A->data[ilast + A->size[0] * ilastm1].re) + std::abs
                   (A->data[ilast + A->size[0] * ilastm1].im) <= b_atol) {
          A->data[ilast + A->size[0] * ilastm1].re = 0.0;
          A->data[ilast + A->size[0] * ilastm1].im = 0.0;
          goto60 = true;
          b_guard1 = true;
        } else {
          j = ilastm1;
          guard3 = false;
          exitg2 = false;
          while ((!exitg2) && (j + 1 >= ilo)) {
            if (j + 1 == ilo) {
              guard3 = true;
              exitg2 = true;
            } else if (std::abs(A->data[j + A->size[0] * (j - 1)].re) + std::abs
                       (A->data[j + A->size[0] * (j - 1)].im) <= b_atol) {
              A->data[j + A->size[0] * (j - 1)].re = 0.0;
              A->data[j + A->size[0] * (j - 1)].im = 0.0;
              guard3 = true;
              exitg2 = true;
            } else {
              j--;
              guard3 = false;
            }
          }

          if (guard3) {
            ifirst = j + 1;
            goto70 = true;
          }

          if (goto70) {
            b_guard1 = true;
          } else {
            loop_ub = alpha1->size[0];
            ifirst = alpha1->size[0];
            alpha1->size[0] = loop_ub;
            emxEnsureCapacity_creal_T(alpha1, ifirst);
            for (ifirst = 0; ifirst < loop_ub; ifirst++) {
              alpha1->data[ifirst].re = rtNaN;
              alpha1->data[ifirst].im = 0.0;
            }

            loop_ub = beta1->size[0];
            ifirst = beta1->size[0];
            beta1->size[0] = loop_ub;
            emxEnsureCapacity_creal_T(beta1, ifirst);
            for (ifirst = 0; ifirst < loop_ub; ifirst++) {
              beta1->data[ifirst].re = rtNaN;
              beta1->data[ifirst].im = 0.0;
            }

            if (compz) {
              ifirst = Z->size[0] * Z->size[1];
              emxEnsureCapacity_creal_T(Z, ifirst);
              loop_ub = Z->size[1];
              for (ifirst = 0; ifirst < loop_ub; ifirst++) {
                jp1 = Z->size[0];
                for (jm1 = 0; jm1 < jp1; jm1++) {
                  Z->data[jm1 + Z->size[0] * ifirst].re = rtNaN;
                  Z->data[jm1 + Z->size[0] * ifirst].im = 0.0;
                }
              }
            }

            *info = 0;
            exitg1 = 1;
          }
        }

        if (b_guard1) {
          if (goto60) {
            goto60 = false;
            alpha1->data[ilast] = A->data[ilast + A->size[0] * ilast];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              firstNonZero = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              if (!compz) {
                ilastm = ilast + 1;
                if (ifrstm > ilast + 1) {
                  ifrstm = ilo;
                }
              }

              jiter++;
            }
          } else {
            if (goto70) {
              goto70 = false;
              iiter++;
              if (!compz) {
                ifrstm = ifirst;
              }

              if (iiter - div_nzp_s32(iiter, 10) * 10 != 0) {
                anorm = ascale * A->data[ilastm1 + A->size[0] * ilastm1].re;
                reAij = ascale * A->data[ilastm1 + A->size[0] * ilastm1].im;
                if (reAij == 0.0) {
                  shift.re = anorm / bscale;
                  shift.im = 0.0;
                } else if (anorm == 0.0) {
                  shift.re = 0.0;
                  shift.im = reAij / bscale;
                } else {
                  shift.re = anorm / bscale;
                  shift.im = reAij / bscale;
                }

                anorm = ascale * A->data[ilast + A->size[0] * ilast].re;
                reAij = ascale * A->data[ilast + A->size[0] * ilast].im;
                if (reAij == 0.0) {
                  ad22_re = anorm / bscale;
                  ad22_im = 0.0;
                } else if (anorm == 0.0) {
                  ad22_re = 0.0;
                  ad22_im = reAij / bscale;
                } else {
                  ad22_re = anorm / bscale;
                  ad22_im = reAij / bscale;
                }

                t1_re = 0.5 * (shift.re + ad22_re);
                t1_im = 0.5 * (shift.im + ad22_im);
                anorm = ascale * A->data[ilastm1 + A->size[0] * ilast].re;
                reAij = ascale * A->data[ilastm1 + A->size[0] * ilast].im;
                if (reAij == 0.0) {
                  imAij = anorm / bscale;
                  temp2 = 0.0;
                } else if (anorm == 0.0) {
                  imAij = 0.0;
                  temp2 = reAij / bscale;
                } else {
                  imAij = anorm / bscale;
                  temp2 = reAij / bscale;
                }

                anorm = ascale * A->data[ilast + A->size[0] * ilastm1].re;
                reAij = ascale * A->data[ilast + A->size[0] * ilastm1].im;
                if (reAij == 0.0) {
                  scale = anorm / bscale;
                  anorm = 0.0;
                } else if (anorm == 0.0) {
                  scale = 0.0;
                  anorm = reAij / bscale;
                } else {
                  scale = anorm / bscale;
                  anorm = reAij / bscale;
                }

                reAij = shift.re * ad22_re - shift.im * ad22_im;
                sumsq = shift.re * ad22_im + shift.im * ad22_re;
                shift.re = ((t1_re * t1_re - t1_im * t1_im) + (imAij * scale -
                  temp2 * anorm)) - reAij;
                shift.im = ((t1_re * t1_im + t1_im * t1_re) + (imAij * anorm +
                  temp2 * scale)) - sumsq;
                c_sqrt(&shift);
                if ((t1_re - ad22_re) * shift.re + (t1_im - ad22_im) * shift.im <=
                    0.0) {
                  shift.re += t1_re;
                  shift.im += t1_im;
                } else {
                  shift.re = t1_re - shift.re;
                  shift.im = t1_im - shift.im;
                }
              } else {
                anorm = ascale * A->data[ilast + A->size[0] * ilastm1].re;
                reAij = ascale * A->data[ilast + A->size[0] * ilastm1].im;
                if (reAij == 0.0) {
                  imAij = anorm / bscale;
                  temp2 = 0.0;
                } else if (anorm == 0.0) {
                  imAij = 0.0;
                  temp2 = reAij / bscale;
                } else {
                  imAij = anorm / bscale;
                  temp2 = reAij / bscale;
                }

                eshift_re += imAij;
                eshift_im += temp2;
                shift.re = eshift_re;
                shift.im = eshift_im;
              }

              j = ilastm1;
              jp1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                ctemp.re = ascale * A->data[j + A->size[0] * j].re - shift.re *
                  bscale;
                ctemp.im = ascale * A->data[j + A->size[0] * j].im - shift.im *
                  bscale;
                anorm = std::abs(ctemp.re) + std::abs(ctemp.im);
                temp2 = ascale * (std::abs(A->data[jp1 + A->size[0] * j].re) +
                                  std::abs(A->data[jp1 + A->size[0] * j].im));
                reAij = anorm;
                if (temp2 > anorm) {
                  reAij = temp2;
                }

                if ((reAij < 1.0) && (reAij != 0.0)) {
                  anorm /= reAij;
                  temp2 /= reAij;
                }

                if ((std::abs(A->data[j + A->size[0] * (j - 1)].re) + std::abs
                     (A->data[j + A->size[0] * (j - 1)].im)) * temp2 <= anorm *
                    b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  jp1 = j;
                  j--;
                }
              }

              if (!goto90) {
                istart = ifirst;
                ctemp.re = ascale * A->data[(ifirst + A->size[0] * (ifirst - 1))
                  - 1].re - shift.re * bscale;
                ctemp.im = ascale * A->data[(ifirst + A->size[0] * (ifirst - 1))
                  - 1].im - shift.im * bscale;
                goto90 = true;
              }
            }

            if (goto90) {
              goto90 = false;
              b_ascale.re = ascale * A->data[istart + A->size[0] * (istart - 1)]
                .re;
              b_ascale.im = ascale * A->data[istart + A->size[0] * (istart - 1)]
                .im;
              b_xzlartg(ctemp, b_ascale, &imAij, &shift);
              j = istart;
              jm1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  b_ascale = A->data[(j + A->size[0] * jm1) - 1];
                  b_A = A->data[j + A->size[0] * jm1];
                  xzlartg(b_ascale, b_A, &imAij, &shift, &A->data[(j + A->size[0]
                           * jm1) - 1]);
                  A->data[j + A->size[0] * jm1].re = 0.0;
                  A->data[j + A->size[0] * jm1].im = 0.0;
                }

                for (loop_ub = j; loop_ub <= ilastm; loop_ub++) {
                  reAij = shift.re * A->data[j + A->size[0] * (loop_ub - 1)].re
                    - shift.im * A->data[j + A->size[0] * (loop_ub - 1)].im;
                  sumsq = shift.re * A->data[j + A->size[0] * (loop_ub - 1)].im
                    + shift.im * A->data[j + A->size[0] * (loop_ub - 1)].re;
                  ad22_re = imAij * A->data[(j + A->size[0] * (loop_ub - 1)) - 1]
                    .re + reAij;
                  ad22_im = imAij * A->data[(j + A->size[0] * (loop_ub - 1)) - 1]
                    .im + sumsq;
                  anorm = A->data[(j + A->size[0] * (loop_ub - 1)) - 1].re;
                  reAij = A->data[(j + A->size[0] * (loop_ub - 1)) - 1].im;
                  scale = A->data[(j + A->size[0] * (loop_ub - 1)) - 1].im;
                  sumsq = A->data[(j + A->size[0] * (loop_ub - 1)) - 1].re;
                  A->data[j + A->size[0] * (loop_ub - 1)].re = imAij * A->data[j
                    + A->size[0] * (loop_ub - 1)].re - (shift.re * anorm +
                    shift.im * reAij);
                  A->data[j + A->size[0] * (loop_ub - 1)].im = imAij * A->data[j
                    + A->size[0] * (loop_ub - 1)].im - (shift.re * scale -
                    shift.im * sumsq);
                  A->data[(j + A->size[0] * (loop_ub - 1)) - 1].re = ad22_re;
                  A->data[(j + A->size[0] * (loop_ub - 1)) - 1].im = ad22_im;
                }

                shift.re = -shift.re;
                shift.im = -shift.im;
                loop_ub = j;
                if (ilast + 1 < j + 2) {
                  loop_ub = ilast - 1;
                }

                for (jp1 = ifrstm; jp1 <= loop_ub + 2; jp1++) {
                  reAij = shift.re * A->data[(jp1 + A->size[0] * (j - 1)) - 1].
                    re - shift.im * A->data[(jp1 + A->size[0] * (j - 1)) - 1].im;
                  sumsq = shift.re * A->data[(jp1 + A->size[0] * (j - 1)) - 1].
                    im + shift.im * A->data[(jp1 + A->size[0] * (j - 1)) - 1].re;
                  ad22_re = imAij * A->data[(jp1 + A->size[0] * j) - 1].re +
                    reAij;
                  ad22_im = imAij * A->data[(jp1 + A->size[0] * j) - 1].im +
                    sumsq;
                  anorm = A->data[(jp1 + A->size[0] * j) - 1].re;
                  reAij = A->data[(jp1 + A->size[0] * j) - 1].im;
                  scale = A->data[(jp1 + A->size[0] * j) - 1].im;
                  sumsq = A->data[(jp1 + A->size[0] * j) - 1].re;
                  A->data[(jp1 + A->size[0] * (j - 1)) - 1].re = imAij * A->
                    data[(jp1 + A->size[0] * (j - 1)) - 1].re - (shift.re *
                    anorm + shift.im * reAij);
                  A->data[(jp1 + A->size[0] * (j - 1)) - 1].im = imAij * A->
                    data[(jp1 + A->size[0] * (j - 1)) - 1].im - (shift.re *
                    scale - shift.im * sumsq);
                  A->data[(jp1 + A->size[0] * j) - 1].re = ad22_re;
                  A->data[(jp1 + A->size[0] * j) - 1].im = ad22_im;
                }

                if (compz) {
                  for (jp1 = 1; jp1 <= n; jp1++) {
                    reAij = shift.re * Z->data[(jp1 + Z->size[0] * (j - 1)) - 1]
                      .re - shift.im * Z->data[(jp1 + Z->size[0] * (j - 1)) - 1]
                      .im;
                    sumsq = shift.re * Z->data[(jp1 + Z->size[0] * (j - 1)) - 1]
                      .im + shift.im * Z->data[(jp1 + Z->size[0] * (j - 1)) - 1]
                      .re;
                    ad22_re = imAij * Z->data[(jp1 + Z->size[0] * j) - 1].re +
                      reAij;
                    ad22_im = imAij * Z->data[(jp1 + Z->size[0] * j) - 1].im +
                      sumsq;
                    anorm = Z->data[(jp1 + Z->size[0] * j) - 1].re;
                    reAij = Z->data[(jp1 + Z->size[0] * j) - 1].im;
                    scale = Z->data[(jp1 + Z->size[0] * j) - 1].im;
                    sumsq = Z->data[(jp1 + Z->size[0] * j) - 1].re;
                    Z->data[(jp1 + Z->size[0] * (j - 1)) - 1].re = imAij *
                      Z->data[(jp1 + Z->size[0] * (j - 1)) - 1].re - (shift.re *
                      anorm + shift.im * reAij);
                    Z->data[(jp1 + Z->size[0] * (j - 1)) - 1].im = imAij *
                      Z->data[(jp1 + Z->size[0] * (j - 1)) - 1].im - (shift.re *
                      scale - shift.im * sumsq);
                    Z->data[(jp1 + Z->size[0] * j) - 1].re = ad22_re;
                    Z->data[(jp1 + Z->size[0] * j) - 1].im = ad22_im;
                  }
                }

                jm1 = j - 1;
                j++;
              }
            }

            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (firstNonZero) {
      *info = ilast;
      for (jp1 = 0; jp1 <= ilast; jp1++) {
        alpha1->data[jp1].re = rtNaN;
        alpha1->data[jp1].im = 0.0;
        beta1->data[jp1].re = rtNaN;
        beta1->data[jp1].im = 0.0;
      }

      if (compz) {
        ifirst = Z->size[0] * Z->size[1];
        emxEnsureCapacity_creal_T(Z, ifirst);
        loop_ub = Z->size[1];
        for (ifirst = 0; ifirst < loop_ub; ifirst++) {
          jp1 = Z->size[0];
          for (jm1 = 0; jm1 < jp1; jm1++) {
            Z->data[jm1 + Z->size[0] * ifirst].re = rtNaN;
            Z->data[jm1 + Z->size[0] * ifirst].im = 0.0;
          }
        }
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (j = 0; j <= ilo - 2; j++) {
      alpha1->data[j] = A->data[j + A->size[0] * j];
    }
  }

  (*info)++;
}

//
// File trailer for xzhgeqz.cpp
//
// [EOF]
//
