//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "svd1.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "xrot.h"
#include "xrotg.h"
#include "sqrt.h"
#include "xswap.h"
#include "xscal.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                double s_data[]
//                int s_size[1]
//                double V[16]
// Return Type  : void
//
void b_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[], int
           s_size[1], double V[16])
{
  emxArray_real_T *b_A;
  int i8;
  int ns;
  int n;
  int minnp;
  double b_s_data[4];
  double e[4];
  emxArray_real_T *work;
  int nctp1;
  int nrt;
  int nct;
  int q;
  int qp1;
  int m;
  int qq;
  int nmq;
  bool apply_transform;
  double nrm;
  int k;
  int qjj;
  int ii;
  double rt;
  double snorm;
  bool exitg1;
  double scale;
  double sqds;
  double sm;
  double b;
  emxInit_real_T(&b_A, 2);
  i8 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = 4;
  emxEnsureCapacity_real_T(b_A, i8);
  ns = A->size[0] * A->size[1];
  for (i8 = 0; i8 < ns; i8++) {
    b_A->data[i8] = A->data[i8];
  }

  n = A->size[0];
  ns = A->size[0] + 1;
  if (ns >= 4) {
    ns = 4;
  }

  minnp = A->size[0];
  if (minnp >= 4) {
    minnp = 4;
  }

  if (0 <= ns - 1) {
    memset(&b_s_data[0], 0, (unsigned int)(ns * (int)sizeof(double)));
  }

  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  e[3] = 0.0;
  emxInit_real_T(&work, 1);
  ns = A->size[0];
  i8 = work->size[0];
  work->size[0] = ns;
  emxEnsureCapacity_real_T(work, i8);
  for (i8 = 0; i8 < ns; i8++) {
    work->data[i8] = 0.0;
  }

  ns = A->size[0];
  nctp1 = A->size[0];
  i8 = U->size[0] * U->size[1];
  U->size[0] = ns;
  U->size[1] = nctp1;
  emxEnsureCapacity_real_T(U, i8);
  ns *= nctp1;
  for (i8 = 0; i8 < ns; i8++) {
    U->data[i8] = 0.0;
  }

  memset(&V[0], 0, sizeof(double) << 4);
  if (A->size[0] == 0) {
    V[0] = 1.0;
    V[5] = 1.0;
    V[10] = 1.0;
    V[15] = 1.0;
  } else {
    nrt = A->size[0];
    if (2 < nrt) {
      nrt = 2;
    }

    if (A->size[0] > 1) {
      nct = A->size[0] - 1;
    } else {
      nct = 0;
    }

    if (nct >= 4) {
      nct = 4;
    }

    nctp1 = nct + 1;
    if (nct > nrt) {
      i8 = nct;
    } else {
      i8 = nrt;
    }

    for (q = 0; q < i8; q++) {
      qp1 = q + 2;
      qq = (q + n * q) + 1;
      nmq = n - q;
      apply_transform = false;
      if (q + 1 <= nct) {
        nrm = xnrm2(nmq, b_A, qq);
        if (nrm > 0.0) {
          apply_transform = true;
          if (b_A->data[qq - 1] < 0.0) {
            nrm = -nrm;
          }

          b_s_data[q] = nrm;
          if (std::abs(nrm) >= 1.0020841800044864E-292) {
            nrm = 1.0 / nrm;
            ns = (qq + nmq) - 1;
            for (k = qq; k <= ns; k++) {
              b_A->data[k - 1] *= nrm;
            }
          } else {
            ns = (qq + nmq) - 1;
            for (k = qq; k <= ns; k++) {
              b_A->data[k - 1] /= b_s_data[q];
            }
          }

          b_A->data[qq - 1]++;
          b_s_data[q] = -b_s_data[q];
        } else {
          b_s_data[q] = 0.0;
        }
      }

      for (k = qp1; k < 5; k++) {
        qjj = q + n * (k - 1);
        if (apply_transform) {
          nrm = -(xdotc(nmq, b_A, qq, b_A, qjj + 1) / b_A->data[q + b_A->size[0]
                  * q]);
          xaxpy(nmq, nrm, qq, b_A, qjj + 1);
        }

        e[k - 1] = b_A->data[qjj];
      }

      if (q + 1 <= nct) {
        for (ii = q + 1; ii <= n; ii++) {
          U->data[(ii + U->size[0] * q) - 1] = b_A->data[(ii + b_A->size[0] * q)
            - 1];
        }
      }

      if (q + 1 <= nrt) {
        nrm = b_xnrm2(3 - q, e, q + 2);
        if (nrm == 0.0) {
          e[q] = 0.0;
        } else {
          if (e[q + 1] < 0.0) {
            e[q] = -nrm;
          } else {
            e[q] = nrm;
          }

          nrm = e[q];
          if (std::abs(e[q]) >= 1.0020841800044864E-292) {
            nrm = 1.0 / e[q];
            for (k = qp1; k < 5; k++) {
              e[k - 1] *= nrm;
            }
          } else {
            for (k = qp1; k < 5; k++) {
              e[k - 1] /= nrm;
            }
          }

          e[q + 1]++;
          e[q] = -e[q];
          if (q + 2 <= n) {
            for (ii = qp1; ii <= n; ii++) {
              work->data[ii - 1] = 0.0;
            }

            for (k = qp1; k < 5; k++) {
              b_xaxpy(nmq - 1, e[k - 1], b_A, (q + n * (k - 1)) + 2, work, q + 2);
            }

            for (k = qp1; k < 5; k++) {
              b_xaxpy(nmq - 1, -e[k - 1] / e[q + 1], work, q + 2, b_A, (q + n *
                       (k - 1)) + 2);
            }
          }
        }

        for (ii = qp1; ii < 5; ii++) {
          V[(ii + (q << 2)) - 1] = e[ii - 1];
        }
      }
    }

    if (4 < A->size[0] + 1) {
      m = 3;
    } else {
      m = A->size[0];
    }

    if (nct < 4) {
      b_s_data[nct] = b_A->data[nct + b_A->size[0] * nct];
    }

    if (A->size[0] < m + 1) {
      b_s_data[m] = 0.0;
    }

    if (nrt + 1 < m + 1) {
      e[nrt] = b_A->data[nrt + b_A->size[0] * m];
    }

    e[m] = 0.0;
    if (nct + 1 <= A->size[0]) {
      for (k = nctp1; k <= n; k++) {
        for (ii = 0; ii < n; ii++) {
          U->data[ii + U->size[0] * (k - 1)] = 0.0;
        }

        U->data[(k + U->size[0] * (k - 1)) - 1] = 1.0;
      }
    }

    for (q = nct; q >= 1; q--) {
      qp1 = q + 1;
      ns = n - q;
      nctp1 = ns + 1;
      qq = (q + n * (q - 1)) - 1;
      if (b_s_data[q - 1] != 0.0) {
        for (k = qp1; k <= n; k++) {
          qjj = q + n * (k - 1);
          nrm = -(xdotc(nctp1, U, qq + 1, U, qjj) / U->data[qq]);
          xaxpy(ns + 1, nrm, qq + 1, U, qjj);
        }

        for (ii = q; ii <= n; ii++) {
          U->data[(ii + U->size[0] * (q - 1)) - 1] = -U->data[(ii + U->size[0] *
            (q - 1)) - 1];
        }

        U->data[qq]++;
        for (ii = 0; ii <= q - 2; ii++) {
          U->data[ii + U->size[0] * (q - 1)] = 0.0;
        }
      } else {
        for (ii = 0; ii < n; ii++) {
          U->data[ii + U->size[0] * (q - 1)] = 0.0;
        }

        U->data[qq] = 1.0;
      }
    }

    for (q = 3; q >= 0; q--) {
      if ((q + 1 <= nrt) && (e[q] != 0.0)) {
        qp1 = q + 2;
        ns = (q + (q << 2)) + 2;
        for (k = qp1; k < 5; k++) {
          nctp1 = (q + ((k - 1) << 2)) + 2;
          c_xaxpy(3 - q, -(b_xdotc(3 - q, V, ns, V, nctp1) / V[ns - 1]), ns, V,
                  nctp1);
        }
      }

      V[q << 2] = 0.0;
      V[1 + (q << 2)] = 0.0;
      V[2 + (q << 2)] = 0.0;
      V[3 + (q << 2)] = 0.0;
      V[q + (q << 2)] = 1.0;
    }

    for (q = 0; q <= m; q++) {
      if (b_s_data[q] != 0.0) {
        rt = std::abs(b_s_data[q]);
        nrm = b_s_data[q] / rt;
        b_s_data[q] = rt;
        if (q + 1 < m + 1) {
          e[q] /= nrm;
        }

        if (q + 1 <= n) {
          xscal(n, nrm, U, 1 + n * q);
        }
      }

      if ((q + 1 < m + 1) && (e[q] != 0.0)) {
        rt = std::abs(e[q]);
        nrm = rt / e[q];
        e[q] = rt;
        b_s_data[q + 1] *= nrm;
        ns = 1 + ((q + 1) << 2);
        i8 = ns + 3;
        for (k = ns; k <= i8; k++) {
          V[k - 1] *= nrm;
        }
      }
    }

    qjj = m;
    nmq = 0;
    snorm = 0.0;
    for (ii = 0; ii <= m; ii++) {
      nrm = std::abs(b_s_data[ii]);
      rt = std::abs(e[ii]);
      if ((nrm > rt) || rtIsNaN(rt)) {
        rt = nrm;
      }

      if ((!(snorm > rt)) && (!rtIsNaN(rt))) {
        snorm = rt;
      }
    }

    while ((m + 1 > 0) && (nmq < 75)) {
      ii = m;
      exitg1 = false;
      while (!(exitg1 || (ii == 0))) {
        nrm = std::abs(e[ii - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[ii - 1]) + std::
              abs(b_s_data[ii]))) || (nrm <= 1.0020841800044864E-292) || ((nmq >
              20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[ii - 1] = 0.0;
          exitg1 = true;
        } else {
          ii--;
        }
      }

      if (ii == m) {
        ns = 4;
      } else {
        nctp1 = m + 1;
        ns = m + 1;
        exitg1 = false;
        while ((!exitg1) && (ns >= ii)) {
          nctp1 = ns;
          if (ns == ii) {
            exitg1 = true;
          } else {
            nrm = 0.0;
            if (ns < m + 1) {
              nrm = std::abs(e[ns - 1]);
            }

            if (ns > ii + 1) {
              nrm += std::abs(e[ns - 2]);
            }

            rt = std::abs(b_s_data[ns - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) || (rt <=
                 1.0020841800044864E-292)) {
              b_s_data[ns - 1] = 0.0;
              exitg1 = true;
            } else {
              ns--;
            }
          }
        }

        if (nctp1 == ii) {
          ns = 3;
        } else if (nctp1 == m + 1) {
          ns = 1;
        } else {
          ns = 2;
          ii = nctp1;
        }
      }

      switch (ns) {
       case 1:
        rt = e[m - 1];
        e[m - 1] = 0.0;
        for (k = m; k >= ii + 1; k--) {
          xrotg(&b_s_data[k - 1], &rt, &sqds, &sm);
          if (k > ii + 1) {
            b = e[k - 2];
            rt = -sm * b;
            e[k - 2] = b * sqds;
          }

          xrot(V, 1 + ((k - 1) << 2), 1 + (m << 2), sqds, sm);
        }
        break;

       case 2:
        rt = e[ii - 1];
        e[ii - 1] = 0.0;
        for (k = ii + 1; k <= m + 1; k++) {
          xrotg(&b_s_data[k - 1], &rt, &sqds, &sm);
          b = e[k - 1];
          rt = -sm * b;
          e[k - 1] = b * sqds;
          b_xrot(n, U, 1 + n * (k - 1), 1 + n * (ii - 1), sqds, sm);
        }
        break;

       case 3:
        scale = std::abs(b_s_data[m]);
        nrm = b_s_data[m - 1];
        rt = std::abs(nrm);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        b = e[m - 1];
        rt = std::abs(b);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        rt = std::abs(b_s_data[ii]);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        rt = std::abs(e[ii]);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        sm = b_s_data[m] / scale;
        nrm /= scale;
        rt = b / scale;
        sqds = b_s_data[ii] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          rt = b * b + nrm;
          b_sqrt(&rt);
          if (b < 0.0) {
            rt = -rt;
          }

          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }

        rt += (sqds + sm) * (sqds - sm);
        nrm = sqds * (e[ii] / scale);
        for (k = ii + 1; k <= m; k++) {
          xrotg(&rt, &nrm, &sqds, &sm);
          if (k > ii + 1) {
            e[k - 2] = rt;
          }

          b = e[k - 1];
          nrm = b_s_data[k - 1];
          e[k - 1] = sqds * b - sm * nrm;
          rt = sm * b_s_data[k];
          b_s_data[k] *= sqds;
          xrot(V, 1 + ((k - 1) << 2), 1 + (k << 2), sqds, sm);
          b_s_data[k - 1] = sqds * nrm + sm * b;
          xrotg(&b_s_data[k - 1], &rt, &sqds, &sm);
          b = e[k - 1];
          rt = sqds * b + sm * b_s_data[k];
          b_s_data[k] = -sm * b + sqds * b_s_data[k];
          nrm = sm * e[k];
          e[k] *= sqds;
          if (k < n) {
            b_xrot(n, U, 1 + n * (k - 1), 1 + n * k, sqds, sm);
          }
        }

        e[m - 1] = rt;
        nmq++;
        break;

       default:
        if (b_s_data[ii] < 0.0) {
          b_s_data[ii] = -b_s_data[ii];
          ns = 1 + (ii << 2);
          i8 = ns + 3;
          for (k = ns; k <= i8; k++) {
            V[k - 1] = -V[k - 1];
          }
        }

        qp1 = ii + 1;
        while ((ii + 1 < qjj + 1) && (b_s_data[ii] < b_s_data[qp1])) {
          rt = b_s_data[ii];
          b_s_data[ii] = b_s_data[qp1];
          b_s_data[qp1] = rt;
          xswap(V, 1 + (ii << 2), 1 + ((ii + 1) << 2));
          if (ii + 1 < n) {
            b_xswap(n, U, 1 + n * ii, 1 + n * (ii + 1));
          }

          ii = qp1;
          qp1++;
        }

        nmq = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&work);
  emxFree_real_T(&b_A);
  s_size[0] = minnp;
  if (0 <= minnp - 1) {
    memcpy(&s_data[0], &b_s_data[0], (unsigned int)(minnp * (int)sizeof(double)));
  }
}

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                double s_data[]
//                int s_size[1]
//                double V[4]
// Return Type  : void
//
void d_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[], int
           s_size[1], double V[4])
{
  emxArray_real_T *b_A;
  int i38;
  int ns;
  int n;
  int minnp;
  double b_s_data[2];
  double e[2];
  int qjj;
  int nct;
  int q;
  int m;
  int qp1;
  int qq;
  int nmq_tmp;
  bool apply_transform;
  double nrm;
  int iter;
  double rt;
  double snorm;
  double r;
  bool exitg1;
  double f;
  double scale;
  double sm;
  double sqds;
  emxInit_real_T(&b_A, 2);
  i38 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = 2;
  emxEnsureCapacity_real_T(b_A, i38);
  ns = A->size[0] * A->size[1];
  for (i38 = 0; i38 < ns; i38++) {
    b_A->data[i38] = A->data[i38];
  }

  n = A->size[0];
  ns = A->size[0] + 1;
  if (ns >= 2) {
    ns = 2;
  }

  minnp = A->size[0];
  if (minnp >= 2) {
    minnp = 2;
  }

  if (0 <= ns - 1) {
    memset(&b_s_data[0], 0, (unsigned int)(ns * (int)sizeof(double)));
  }

  e[0] = 0.0;
  e[1] = 0.0;
  ns = A->size[0];
  qjj = A->size[0];
  i38 = U->size[0] * U->size[1];
  U->size[0] = ns;
  U->size[1] = qjj;
  emxEnsureCapacity_real_T(U, i38);
  ns *= qjj;
  for (i38 = 0; i38 < ns; i38++) {
    U->data[i38] = 0.0;
  }

  V[0] = 0.0;
  V[1] = 0.0;
  V[2] = 0.0;
  V[3] = 0.0;
  if (A->size[0] == 0) {
    V[0] = 1.0;
    V[3] = 1.0;
  } else {
    if (A->size[0] > 1) {
      nct = A->size[0] - 1;
    } else {
      nct = 0;
    }

    if (nct >= 2) {
      nct = 2;
    }

    ns = nct + 1;
    for (q = 0; q < nct; q++) {
      qp1 = q + 2;
      qq = (q + n * q) + 1;
      nmq_tmp = n - q;
      apply_transform = false;
      if (q + 1 <= nct) {
        nrm = xnrm2(nmq_tmp, b_A, qq);
        if (nrm > 0.0) {
          apply_transform = true;
          if (b_A->data[qq - 1] < 0.0) {
            nrm = -nrm;
          }

          b_s_data[q] = nrm;
          if (std::abs(nrm) >= 1.0020841800044864E-292) {
            nrm = 1.0 / nrm;
            i38 = (qq + nmq_tmp) - 1;
            for (qjj = qq; qjj <= i38; qjj++) {
              b_A->data[qjj - 1] *= nrm;
            }
          } else {
            i38 = (qq + nmq_tmp) - 1;
            for (qjj = qq; qjj <= i38; qjj++) {
              b_A->data[qjj - 1] /= b_s_data[q];
            }
          }

          b_A->data[qq - 1]++;
          b_s_data[q] = -b_s_data[q];
        } else {
          b_s_data[q] = 0.0;
        }
      }

      for (iter = qp1; iter < 3; iter++) {
        qjj = q + n;
        if (apply_transform) {
          nrm = -(xdotc(nmq_tmp, b_A, qq, b_A, qjj + 1) / b_A->data[q +
                  b_A->size[0] * q]);
          xaxpy(nmq_tmp, nrm, qq, b_A, qjj + 1);
        }

        e[1] = b_A->data[qjj];
      }

      if (q + 1 <= nct) {
        for (nmq_tmp = q + 1; nmq_tmp <= n; nmq_tmp++) {
          U->data[(nmq_tmp + U->size[0] * q) - 1] = b_A->data[(nmq_tmp +
            b_A->size[0] * q) - 1];
        }
      }
    }

    m = A->size[0] + 1;
    if (2 < m) {
      m = 2;
    }

    if (nct < 2) {
      b_s_data[nct] = b_A->data[nct + b_A->size[0] * nct];
    }

    if (A->size[0] < m) {
      b_s_data[m - 1] = 0.0;
    }

    if (1 < m) {
      e[0] = b_A->data[b_A->size[0]];
    }

    e[m - 1] = 0.0;
    if (nct + 1 <= A->size[0]) {
      for (iter = ns; iter <= n; iter++) {
        for (nmq_tmp = 0; nmq_tmp < n; nmq_tmp++) {
          U->data[nmq_tmp + U->size[0] * (iter - 1)] = 0.0;
        }

        U->data[(iter + U->size[0] * (iter - 1)) - 1] = 1.0;
      }
    }

    for (q = nct; q >= 1; q--) {
      qp1 = q + 1;
      nmq_tmp = n - q;
      ns = nmq_tmp + 1;
      qq = (q + n * (q - 1)) - 1;
      if (b_s_data[q - 1] != 0.0) {
        for (iter = qp1; iter <= n; iter++) {
          qjj = q + n * (iter - 1);
          nrm = -(xdotc(ns, U, qq + 1, U, qjj) / U->data[qq]);
          xaxpy(nmq_tmp + 1, nrm, qq + 1, U, qjj);
        }

        for (nmq_tmp = q; nmq_tmp <= n; nmq_tmp++) {
          U->data[(nmq_tmp + U->size[0] * (q - 1)) - 1] = -U->data[(nmq_tmp +
            U->size[0] * (q - 1)) - 1];
        }

        U->data[qq]++;
        for (nmq_tmp = 0; nmq_tmp <= q - 2; nmq_tmp++) {
          U->data[U->size[0] * (q - 1)] = 0.0;
        }
      } else {
        for (nmq_tmp = 0; nmq_tmp < n; nmq_tmp++) {
          U->data[nmq_tmp + U->size[0] * (q - 1)] = 0.0;
        }

        U->data[qq] = 1.0;
      }
    }

    V[2] = 0.0;
    V[3] = 0.0;
    V[3] = 1.0;
    V[0] = 0.0;
    V[1] = 0.0;
    V[0] = 1.0;
    nrm = e[0];
    for (q = 0; q < m; q++) {
      if (b_s_data[q] != 0.0) {
        rt = std::abs(b_s_data[q]);
        r = b_s_data[q] / rt;
        b_s_data[q] = rt;
        if (q + 1 < m) {
          nrm /= r;
        }

        if (q + 1 <= n) {
          xscal(n, r, U, 1 + n * q);
        }
      }

      if ((q + 1 < m) && (nrm != 0.0)) {
        rt = std::abs(nrm);
        r = rt / nrm;
        nrm = rt;
        b_s_data[1] *= r;
        V[2] *= r;
        V[3] *= r;
      }

      e[0] = nrm;
    }

    qp1 = m;
    iter = 0;
    snorm = 0.0;
    for (nmq_tmp = 0; nmq_tmp < m; nmq_tmp++) {
      nrm = std::abs(b_s_data[nmq_tmp]);
      r = std::abs(e[nmq_tmp]);
      if ((nrm > r) || rtIsNaN(r)) {
        r = nrm;
      }

      if ((!(snorm > r)) && (!rtIsNaN(r))) {
        snorm = r;
      }
    }

    while ((m > 0) && (iter < 75)) {
      nmq_tmp = m - 1;
      exitg1 = false;
      while (!(exitg1 || (nmq_tmp == 0))) {
        nrm = std::abs(e[0]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[0]) + std::abs
              (b_s_data[1]))) || (nrm <= 1.0020841800044864E-292) || ((iter > 20)
             && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[0] = 0.0;
          exitg1 = true;
        } else {
          nmq_tmp = 0;
        }
      }

      if (nmq_tmp == m - 1) {
        ns = 4;
      } else {
        qjj = m;
        ns = m;
        exitg1 = false;
        while ((!exitg1) && (ns >= nmq_tmp)) {
          qjj = ns;
          if (ns == nmq_tmp) {
            exitg1 = true;
          } else {
            nrm = 0.0;
            if (ns < m) {
              nrm = std::abs(e[0]);
            }

            if (ns > nmq_tmp + 1) {
              nrm += std::abs(e[0]);
            }

            r = std::abs(b_s_data[ns - 1]);
            if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
                 1.0020841800044864E-292)) {
              b_s_data[ns - 1] = 0.0;
              exitg1 = true;
            } else {
              ns--;
            }
          }
        }

        if (qjj == nmq_tmp) {
          ns = 3;
        } else if (qjj == m) {
          ns = 1;
        } else {
          ns = 2;
          nmq_tmp = qjj;
        }
      }

      switch (ns) {
       case 1:
        f = e[0];
        e[0] = 0.0;
        i38 = m - 1;
        for (qjj = i38; qjj >= nmq_tmp + 1; qjj--) {
          xrotg(&b_s_data[0], &f, &rt, &sm);
          c_xrot(V, 1 + ((m - 1) << 1), rt, sm);
        }
        break;

       case 2:
        f = e[nmq_tmp - 1];
        e[nmq_tmp - 1] = 0.0;
        for (qjj = nmq_tmp + 1; qjj <= m; qjj++) {
          xrotg(&b_s_data[qjj - 1], &f, &rt, &sm);
          nrm = e[qjj - 1];
          f = -sm * nrm;
          e[qjj - 1] = nrm * rt;
          b_xrot(n, U, 1 + n * (qjj - 1), 1 + n * (nmq_tmp - 1), rt, sm);
        }
        break;

       case 3:
        nrm = b_s_data[m - 1];
        scale = std::abs(nrm);
        r = std::abs(b_s_data[0]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(e[0]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(b_s_data[nmq_tmp]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(e[nmq_tmp]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        sm = nrm / scale;
        nrm = b_s_data[0] / scale;
        r = e[0] / scale;
        sqds = b_s_data[nmq_tmp] / scale;
        rt = ((nrm + sm) * (nrm - sm) + r * r) / 2.0;
        nrm = sm * r;
        nrm *= nrm;
        if ((rt != 0.0) || (nrm != 0.0)) {
          r = rt * rt + nrm;
          b_sqrt(&r);
          if (rt < 0.0) {
            r = -r;
          }

          r = nrm / (rt + r);
        } else {
          r = 0.0;
        }

        f = (sqds + sm) * (sqds - sm) + r;
        nrm = sqds * (e[nmq_tmp] / scale);
        for (qjj = nmq_tmp + 1; qjj < 2; qjj++) {
          xrotg(&f, &nrm, &rt, &sm);
          f = rt * b_s_data[0] + sm * e[0];
          nrm = rt * e[0] - sm * b_s_data[0];
          e[0] = nrm;
          r = sm * b_s_data[1];
          b_s_data[1] *= rt;
          c_xrot(V, 3, rt, sm);
          b_s_data[0] = f;
          xrotg(&b_s_data[0], &r, &rt, &sm);
          f = rt * nrm + sm * b_s_data[1];
          b_s_data[1] = -sm * nrm + rt * b_s_data[1];
          nrm = sm * e[1];
          e[1] *= rt;
          if (1 < n) {
            b_xrot(n, U, 1, 1 + n, rt, sm);
          }
        }

        e[0] = f;
        iter++;
        break;

       default:
        if (b_s_data[nmq_tmp] < 0.0) {
          b_s_data[nmq_tmp] = -b_s_data[nmq_tmp];
          ns = 1 + (nmq_tmp << 1);
          i38 = ns + 1;
          for (qjj = ns; qjj <= i38; qjj++) {
            V[qjj - 1] = -V[qjj - 1];
          }
        }

        while ((nmq_tmp + 1 < qp1) && (b_s_data[0] < b_s_data[1])) {
          rt = b_s_data[0];
          b_s_data[0] = b_s_data[1];
          b_s_data[1] = rt;
          c_xswap(V);
          if (1 < n) {
            b_xswap(n, U, 1, 1 + n);
          }

          nmq_tmp = 1;
        }

        iter = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&b_A);
  s_size[0] = minnp;
  if (0 <= minnp - 1) {
    memcpy(&s_data[0], &b_s_data[0], (unsigned int)(minnp * (int)sizeof(double)));
  }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
