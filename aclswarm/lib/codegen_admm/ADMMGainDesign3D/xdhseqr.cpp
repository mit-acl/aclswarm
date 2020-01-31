//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdhseqr.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xdhseqr.h"
#include "xrot.h"
#include "xdlanv2.h"
#include "xzlarfg.h"

// Function Definitions

//
// Arguments    : emxArray_real_T *h
//                emxArray_real_T *z
// Return Type  : int
//
int eml_dlahqr(emxArray_real_T *h, emxArray_real_T *z)
{
  int info;
  int n;
  int ldh;
  int ldz;
  double v[3];
  int nr;
  int iy;
  double itmax;
  double SMLNUM;
  int i;
  bool exitg1;
  int L;
  bool goto150;
  int its;
  bool exitg2;
  int k;
  bool exitg3;
  double d0;
  double d1;
  double tst;
  double htmp1;
  double aa;
  double ab;
  double ba;
  double h22;
  double s;
  int hoffset;
  int m;
  double b_SMLNUM;
  int b_k;
  n = h->size[0];
  ldh = h->size[0];
  ldz = z->size[0];
  info = 0;
  if ((h->size[0] != 0) && (1 != h->size[0])) {
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
    nr = h->size[0];
    for (iy = 0; iy <= nr - 4; iy++) {
      h->data[(iy + h->size[0] * iy) + 2] = 0.0;
      h->data[(iy + h->size[0] * iy) + 3] = 0.0;
    }

    if (1 <= n - 2) {
      h->data[(n + h->size[0] * (n - 3)) - 1] = 0.0;
    }

    if (10 > n) {
      nr = 10;
    } else {
      nr = n;
    }

    itmax = 30.0 * (double)nr;
    SMLNUM = 2.2250738585072014E-308 * ((double)n / 2.2204460492503131E-16);
    i = n - 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1)) {
      L = 1;
      goto150 = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its <= (int)itmax)) {
        k = i;
        exitg3 = false;
        while ((!exitg3) && ((k + 1 > L) && (!(std::abs(h->data[k + h->size[0] *
                   (k - 1)]) <= SMLNUM)))) {
          tst = std::abs(h->data[(k + h->size[0] * (k - 1)) - 1]) + std::abs
            (h->data[k + h->size[0] * k]);
          if (tst == 0.0) {
            if (k - 1 >= 1) {
              tst = std::abs(h->data[(k + h->size[0] * (k - 2)) - 1]);
            }

            if (k + 2 <= n) {
              tst += std::abs(h->data[(k + h->size[0] * k) + 1]);
            }
          }

          if (std::abs(h->data[k + h->size[0] * (k - 1)]) <=
              2.2204460492503131E-16 * tst) {
            htmp1 = std::abs(h->data[k + h->size[0] * (k - 1)]);
            tst = std::abs(h->data[(k + h->size[0] * k) - 1]);
            if (htmp1 > tst) {
              ab = htmp1;
              ba = tst;
            } else {
              ab = tst;
              ba = htmp1;
            }

            htmp1 = std::abs(h->data[k + h->size[0] * k]);
            tst = std::abs(h->data[(k + h->size[0] * (k - 1)) - 1] - h->data[k +
                           h->size[0] * k]);
            if (htmp1 > tst) {
              aa = htmp1;
              htmp1 = tst;
            } else {
              aa = tst;
            }

            s = aa + ab;
            tst = 2.2204460492503131E-16 * (htmp1 * (aa / s));
            if ((SMLNUM > tst) || rtIsNaN(tst)) {
              b_SMLNUM = SMLNUM;
            } else {
              b_SMLNUM = tst;
            }

            if (ba * (ab / s) <= b_SMLNUM) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }

        L = k + 1;
        if (k + 1 > 1) {
          h->data[k + h->size[0] * (k - 1)] = 0.0;
        }

        if (k + 1 >= i) {
          goto150 = true;
          exitg2 = true;
        } else {
          if (its == 10) {
            s = std::abs(h->data[(k + h->size[0] * k) + 1]) + std::abs(h->data
              [(k + h->size[0] * (k + 1)) + 2]);
            tst = 0.75 * s + h->data[k + h->size[0] * k];
            aa = -0.4375 * s;
            htmp1 = s;
            h22 = tst;
          } else if (its == 20) {
            s = std::abs(h->data[i + h->size[0] * (i - 1)]) + std::abs(h->data
              [(i + h->size[0] * (i - 2)) - 1]);
            tst = 0.75 * s + h->data[i + h->size[0] * i];
            aa = -0.4375 * s;
            htmp1 = s;
            h22 = tst;
          } else {
            tst = h->data[(i + h->size[0] * (i - 1)) - 1];
            htmp1 = h->data[i + h->size[0] * (i - 1)];
            aa = h->data[(i + h->size[0] * i) - 1];
            h22 = h->data[i + h->size[0] * i];
          }

          s = ((std::abs(tst) + std::abs(aa)) + std::abs(htmp1)) + std::abs(h22);
          if (s == 0.0) {
            ba = 0.0;
            tst = 0.0;
            ab = 0.0;
            htmp1 = 0.0;
          } else {
            tst /= s;
            htmp1 /= s;
            aa /= s;
            h22 /= s;
            ab = (tst + h22) / 2.0;
            tst = (tst - ab) * (h22 - ab) - aa * htmp1;
            htmp1 = std::sqrt(std::abs(tst));
            if (tst >= 0.0) {
              ba = ab * s;
              ab = ba;
              tst = htmp1 * s;
              htmp1 = -tst;
            } else {
              ba = ab + htmp1;
              ab -= htmp1;
              if (std::abs(ba - h22) <= std::abs(ab - h22)) {
                ba *= s;
                ab = ba;
              } else {
                ab *= s;
                ba = ab;
              }

              tst = 0.0;
              htmp1 = 0.0;
            }
          }

          m = i - 1;
          exitg3 = false;
          while ((!exitg3) && (m >= k + 1)) {
            s = (std::abs(h->data[(m + h->size[0] * (m - 1)) - 1] - ab) + std::
                 abs(htmp1)) + std::abs(h->data[m + h->size[0] * (m - 1)]);
            aa = h->data[m + h->size[0] * (m - 1)] / s;
            v[0] = (aa * h->data[(m + h->size[0] * m) - 1] + (h->data[(m +
                      h->size[0] * (m - 1)) - 1] - ba) * ((h->data[(m + h->size
                       [0] * (m - 1)) - 1] - ab) / s)) - tst * (htmp1 / s);
            v[1] = aa * (((h->data[(m + h->size[0] * (m - 1)) - 1] + h->data[m +
                           h->size[0] * m]) - ba) - ab);
            v[2] = aa * h->data[(m + h->size[0] * m) + 1];
            s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
            if ((m == k + 1) || (std::abs(h->data[(m + h->size[0] * (m - 2)) - 1])
                                 * (std::abs(v[1]) + std::abs(v[2])) <=
                                 2.2204460492503131E-16 * std::abs(v[0]) * ((std::
                   abs(h->data[(m + h->size[0] * (m - 2)) - 2]) + std::abs
                   (h->data[(m + h->size[0] * (m - 1)) - 1])) + std::abs(h->
                   data[m + h->size[0] * m])))) {
              exitg3 = true;
            } else {
              m--;
            }
          }

          for (b_k = m; b_k <= i; b_k++) {
            nr = (i - b_k) + 2;
            if (3 < nr) {
              nr = 3;
            }

            if (b_k > m) {
              hoffset = (b_k + ldh * (b_k - 2)) - 1;
              for (iy = 0; iy < nr; iy++) {
                v[iy] = h->data[iy + hoffset];
              }
            }

            tst = v[0];
            ab = xzlarfg(nr, &tst, v);
            v[0] = tst;
            if (b_k > m) {
              h->data[(b_k + h->size[0] * (b_k - 2)) - 1] = tst;
              h->data[b_k + h->size[0] * (b_k - 2)] = 0.0;
              if (b_k < i) {
                h->data[(b_k + h->size[0] * (b_k - 2)) + 1] = 0.0;
              }
            } else {
              if (m > k + 1) {
                h->data[(b_k + h->size[0] * (b_k - 2)) - 1] *= 1.0 - ab;
              }
            }

            d0 = v[1];
            tst = ab * v[1];
            if (nr == 3) {
              d1 = v[2];
              aa = ab * v[2];
              for (iy = b_k; iy <= n; iy++) {
                htmp1 = (h->data[(b_k + h->size[0] * (iy - 1)) - 1] + d0 *
                         h->data[b_k + h->size[0] * (iy - 1)]) + d1 * h->data
                  [(b_k + h->size[0] * (iy - 1)) + 1];
                h->data[(b_k + h->size[0] * (iy - 1)) - 1] -= htmp1 * ab;
                h->data[b_k + h->size[0] * (iy - 1)] -= htmp1 * tst;
                h->data[(b_k + h->size[0] * (iy - 1)) + 1] -= htmp1 * aa;
              }

              if (b_k + 3 < i + 1) {
                nr = b_k + 2;
              } else {
                nr = i;
              }

              for (iy = 0; iy <= nr; iy++) {
                htmp1 = (h->data[iy + h->size[0] * (b_k - 1)] + d0 * h->data[iy
                         + h->size[0] * b_k]) + d1 * h->data[iy + h->size[0] *
                  (b_k + 1)];
                h->data[iy + h->size[0] * (b_k - 1)] -= htmp1 * ab;
                h->data[iy + h->size[0] * b_k] -= htmp1 * tst;
                h->data[iy + h->size[0] * (b_k + 1)] -= htmp1 * aa;
              }

              for (iy = 0; iy < n; iy++) {
                htmp1 = (z->data[iy + z->size[0] * (b_k - 1)] + d0 * z->data[iy
                         + z->size[0] * b_k]) + d1 * z->data[iy + z->size[0] *
                  (b_k + 1)];
                z->data[iy + z->size[0] * (b_k - 1)] -= htmp1 * ab;
                z->data[iy + z->size[0] * b_k] -= htmp1 * tst;
                z->data[iy + z->size[0] * (b_k + 1)] -= htmp1 * aa;
              }
            } else {
              if (nr == 2) {
                for (iy = b_k; iy <= n; iy++) {
                  htmp1 = h->data[(b_k + h->size[0] * (iy - 1)) - 1] + d0 *
                    h->data[b_k + h->size[0] * (iy - 1)];
                  h->data[(b_k + h->size[0] * (iy - 1)) - 1] -= htmp1 * ab;
                  h->data[b_k + h->size[0] * (iy - 1)] -= htmp1 * tst;
                }

                for (iy = 0; iy <= i; iy++) {
                  htmp1 = h->data[iy + h->size[0] * (b_k - 1)] + d0 * h->data[iy
                    + h->size[0] * b_k];
                  h->data[iy + h->size[0] * (b_k - 1)] -= htmp1 * ab;
                  h->data[iy + h->size[0] * b_k] -= htmp1 * tst;
                }

                for (iy = 0; iy < n; iy++) {
                  htmp1 = z->data[iy + z->size[0] * (b_k - 1)] + d0 * z->data[iy
                    + z->size[0] * b_k];
                  z->data[iy + z->size[0] * (b_k - 1)] -= htmp1 * ab;
                  z->data[iy + z->size[0] * b_k] -= htmp1 * tst;
                }
              }
            }
          }

          its++;
        }
      }

      if (!goto150) {
        info = i + 1;
        exitg1 = true;
      } else {
        if ((L != i + 1) && (L == i)) {
          d0 = h->data[(i + h->size[0] * i) - 1];
          d1 = h->data[i + h->size[0] * (i - 1)];
          tst = h->data[i + h->size[0] * i];
          xdlanv2(&h->data[(i + h->size[0] * (i - 1)) - 1], &d0, &d1, &tst,
                  &htmp1, &aa, &ab, &ba, &h22, &s);
          h->data[(i + h->size[0] * i) - 1] = d0;
          h->data[i + h->size[0] * (i - 1)] = d1;
          h->data[i + h->size[0] * i] = tst;
          if (n > i + 1) {
            nr = (n - i) - 2;
            if (nr + 1 >= 1) {
              iy = i + (i + 1) * ldh;
              hoffset = iy - 1;
              for (k = 0; k <= nr; k++) {
                tst = h22 * h->data[hoffset] + s * h->data[iy];
                h->data[iy] = h22 * h->data[iy] - s * h->data[hoffset];
                h->data[hoffset] = tst;
                iy += ldh;
                hoffset += ldh;
              }
            }
          }

          b_xrot(i - 1, h, 1 + (i - 1) * ldh, 1 + i * ldh, h22, s);
          b_xrot(n, z, 1 + (i - 1) * ldz, 1 + i * ldz, h22, s);
        }

        i = L - 2;
      }
    }
  }

  return info;
}

//
// File trailer for xdhseqr.cpp
//
// [EOF]
//
