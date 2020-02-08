/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xdhseqr.cpp
 *
 * Code generation for function 'xdhseqr'
 *
 */

/* Include files */
#include "xdhseqr.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include <cmath>

/* Function Definitions */
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
  double s;
  double d;
  double tst;
  double ba;
  double aa;
  double ab;
  double bb;
  double rt2r;
  double rt1r;
  int hoffset;
  int m;
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

    itmax = 30.0 * static_cast<double>(nr);
    SMLNUM = 2.2250738585072014E-308 * (static_cast<double>(n) /
      2.2204460492503131E-16);
    i = n - 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1)) {
      L = 1;
      goto150 = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its <= static_cast<int>(itmax))) {
        k = i;
        exitg3 = false;
        while ((!exitg3) && (k + 1 > L)) {
          ba = std::abs(h->data[k + h->size[0] * (k - 1)]);
          if (ba <= SMLNUM) {
            exitg3 = true;
          } else {
            bb = std::abs(h->data[k + h->size[0] * k]);
            tst = std::abs(h->data[(k + h->size[0] * (k - 1)) - 1]) + bb;
            if (tst == 0.0) {
              if (k - 1 >= 1) {
                tst = std::abs(h->data[(k + h->size[0] * (k - 2)) - 1]);
              }

              if (k + 2 <= n) {
                tst += std::abs(h->data[(k + h->size[0] * k) + 1]);
              }
            }

            if (ba <= 2.2204460492503131E-16 * tst) {
              tst = std::abs(h->data[(k + h->size[0] * k) - 1]);
              if (ba > tst) {
                ab = ba;
                ba = tst;
              } else {
                ab = tst;
              }

              tst = std::abs(h->data[(k + h->size[0] * (k - 1)) - 1] - h->data[k
                             + h->size[0] * k]);
              if (bb > tst) {
                aa = bb;
                bb = tst;
              } else {
                aa = tst;
              }

              s = aa + ab;
              tst = 2.2204460492503131E-16 * (bb * (aa / s));
              if ((SMLNUM > tst) || rtIsNaN(tst)) {
                tst = SMLNUM;
              }

              if (ba * (ab / s) <= tst) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
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
            ab = -0.4375 * s;
            aa = s;
            ba = tst;
          } else if (its == 20) {
            s = std::abs(h->data[i + h->size[0] * (i - 1)]) + std::abs(h->data
              [(i + h->size[0] * (i - 2)) - 1]);
            tst = 0.75 * s + h->data[i + h->size[0] * i];
            ab = -0.4375 * s;
            aa = s;
            ba = tst;
          } else {
            tst = h->data[(i + h->size[0] * (i - 1)) - 1];
            aa = h->data[i + h->size[0] * (i - 1)];
            ab = h->data[(i + h->size[0] * i) - 1];
            ba = h->data[i + h->size[0] * i];
          }

          s = ((std::abs(tst) + std::abs(ab)) + std::abs(aa)) + std::abs(ba);
          if (s == 0.0) {
            rt1r = 0.0;
            aa = 0.0;
            rt2r = 0.0;
            ab = 0.0;
          } else {
            tst /= s;
            aa /= s;
            ab /= s;
            ba /= s;
            bb = (tst + ba) / 2.0;
            tst = (tst - bb) * (ba - bb) - ab * aa;
            aa = std::sqrt(std::abs(tst));
            if (tst >= 0.0) {
              rt1r = bb * s;
              rt2r = rt1r;
              aa *= s;
              ab = -aa;
            } else {
              rt1r = bb + aa;
              rt2r = bb - aa;
              if (std::abs(rt1r - ba) <= std::abs(rt2r - ba)) {
                rt1r *= s;
                rt2r = rt1r;
              } else {
                rt2r *= s;
                rt1r = rt2r;
              }

              aa = 0.0;
              ab = 0.0;
            }
          }

          m = i - 1;
          exitg3 = false;
          while ((!exitg3) && (m >= k + 1)) {
            tst = h->data[m + h->size[0] * (m - 1)];
            bb = h->data[(m + h->size[0] * (m - 1)) - 1];
            ba = bb - rt2r;
            s = (std::abs(ba) + std::abs(ab)) + std::abs(tst);
            tst /= s;
            v[0] = (tst * h->data[(m + h->size[0] * m) - 1] + (bb - rt1r) * (ba /
                     s)) - aa * (ab / s);
            v[1] = tst * (((bb + h->data[m + h->size[0] * m]) - rt1r) - rt2r);
            v[2] = tst * h->data[(m + h->size[0] * m) + 1];
            s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
            if ((m == k + 1) || (std::abs(h->data[(m + h->size[0] * (m - 2)) - 1])
                                 * (std::abs(v[1]) + std::abs(v[2])) <=
                                 2.2204460492503131E-16 * std::abs(v[0]) * ((std::
                   abs(h->data[(m + h->size[0] * (m - 2)) - 2]) + std::abs(bb))
                  + std::abs(h->data[m + h->size[0] * m])))) {
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
            ba = xzlarfg(nr, &tst, v);
            v[0] = tst;
            if (b_k > m) {
              h->data[(b_k + h->size[0] * (b_k - 2)) - 1] = tst;
              h->data[b_k + h->size[0] * (b_k - 2)] = 0.0;
              if (b_k < i) {
                h->data[(b_k + h->size[0] * (b_k - 2)) + 1] = 0.0;
              }
            } else {
              if (m > k + 1) {
                h->data[(b_k + h->size[0] * (b_k - 2)) - 1] *= 1.0 - ba;
              }
            }

            s = v[1];
            tst = ba * v[1];
            if (nr == 3) {
              d = v[2];
              bb = ba * v[2];
              for (iy = b_k; iy <= n; iy++) {
                ab = (h->data[(b_k + h->size[0] * (iy - 1)) - 1] + s * h->
                      data[b_k + h->size[0] * (iy - 1)]) + d * h->data[(b_k +
                  h->size[0] * (iy - 1)) + 1];
                h->data[(b_k + h->size[0] * (iy - 1)) - 1] -= ab * ba;
                h->data[b_k + h->size[0] * (iy - 1)] -= ab * tst;
                h->data[(b_k + h->size[0] * (iy - 1)) + 1] -= ab * bb;
              }

              if (b_k + 3 < i + 1) {
                nr = b_k + 2;
              } else {
                nr = i;
              }

              for (iy = 0; iy <= nr; iy++) {
                ab = (h->data[iy + h->size[0] * (b_k - 1)] + s * h->data[iy +
                      h->size[0] * b_k]) + d * h->data[iy + h->size[0] * (b_k +
                  1)];
                h->data[iy + h->size[0] * (b_k - 1)] -= ab * ba;
                h->data[iy + h->size[0] * b_k] -= ab * tst;
                h->data[iy + h->size[0] * (b_k + 1)] -= ab * bb;
              }

              for (iy = 0; iy < n; iy++) {
                aa = z->data[iy + z->size[0] * (b_k - 1)];
                ab = (aa + s * z->data[iy + z->size[0] * b_k]) + d * z->data[iy
                  + z->size[0] * (b_k + 1)];
                z->data[iy + z->size[0] * (b_k - 1)] = aa - ab * ba;
                z->data[iy + z->size[0] * b_k] -= ab * tst;
                z->data[iy + z->size[0] * (b_k + 1)] -= ab * bb;
              }
            } else {
              if (nr == 2) {
                for (iy = b_k; iy <= n; iy++) {
                  aa = h->data[(b_k + h->size[0] * (iy - 1)) - 1];
                  ab = aa + s * h->data[b_k + h->size[0] * (iy - 1)];
                  h->data[(b_k + h->size[0] * (iy - 1)) - 1] = aa - ab * ba;
                  h->data[b_k + h->size[0] * (iy - 1)] -= ab * tst;
                }

                for (iy = 0; iy <= i; iy++) {
                  ab = h->data[iy + h->size[0] * (b_k - 1)] + s * h->data[iy +
                    h->size[0] * b_k];
                  h->data[iy + h->size[0] * (b_k - 1)] -= ab * ba;
                  h->data[iy + h->size[0] * b_k] -= ab * tst;
                }

                for (iy = 0; iy < n; iy++) {
                  aa = z->data[iy + z->size[0] * (b_k - 1)];
                  ab = aa + s * z->data[iy + z->size[0] * b_k];
                  z->data[iy + z->size[0] * (b_k - 1)] = aa - ab * ba;
                  z->data[iy + z->size[0] * b_k] -= ab * tst;
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
          s = h->data[(i + h->size[0] * i) - 1];
          d = h->data[i + h->size[0] * (i - 1)];
          tst = h->data[i + h->size[0] * i];
          xdlanv2(&h->data[(i + h->size[0] * (i - 1)) - 1], &s, &d, &tst, &aa,
                  &ab, &bb, &ba, &rt2r, &rt1r);
          h->data[(i + h->size[0] * i) - 1] = s;
          h->data[i + h->size[0] * (i - 1)] = d;
          h->data[i + h->size[0] * i] = tst;
          if (n > i + 1) {
            nr = (n - i) - 2;
            if (nr + 1 >= 1) {
              iy = i + (i + 1) * ldh;
              hoffset = iy - 1;
              for (k = 0; k <= nr; k++) {
                tst = rt2r * h->data[hoffset] + rt1r * h->data[iy];
                h->data[iy] = rt2r * h->data[iy] - rt1r * h->data[hoffset];
                h->data[hoffset] = tst;
                iy += ldh;
                hoffset += ldh;
              }
            }
          }

          if (i - 1 >= 1) {
            hoffset = (i - 1) * ldh;
            iy = i * ldh;
            for (k = 0; k <= i - 2; k++) {
              tst = rt2r * h->data[hoffset] + rt1r * h->data[iy];
              h->data[iy] = rt2r * h->data[iy] - rt1r * h->data[hoffset];
              h->data[hoffset] = tst;
              iy++;
              hoffset++;
            }
          }

          if (n >= 1) {
            hoffset = (i - 1) * ldz;
            iy = i * ldz;
            for (k = 0; k < n; k++) {
              tst = rt2r * z->data[hoffset] + rt1r * z->data[iy];
              z->data[iy] = rt2r * z->data[iy] - rt1r * z->data[hoffset];
              z->data[hoffset] = tst;
              iy++;
              hoffset++;
            }
          }
        }

        i = L - 2;
      }
    }
  }

  return info;
}

/* End of code generation (xdhseqr.cpp) */
