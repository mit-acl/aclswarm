//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "svd1.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include <cmath>
#include <cstring>

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
  int i;
  int ns;
  int n;
  int minnp;
  double b_s_data[4];
  emxArray_real_T *work;
  double e[4];
  int nrt;
  int nct;
  int nctp1;
  int q;
  int qp1;
  int m;
  int qq;
  int nmq;
  bool apply_transform;
  double nrm;
  int jj;
  int qjj;
  int ii;
  double r;
  int ix;
  int iy;
  int k;
  double snorm;
  bool exitg1;
  double f;
  double scale;
  double sqds;
  double b;
  emxInit_real_T(&b_A, 2);
  i = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = 4;
  emxEnsureCapacity_real_T(b_A, i);
  ns = A->size[0] * A->size[1];
  for (i = 0; i < ns; i++) {
    b_A->data[i] = A->data[i];
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
    std::memset(&b_s_data[0], 0, ns * sizeof(double));
  }

  emxInit_real_T(&work, 1);
  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  e[3] = 0.0;
  i = work->size[0];
  work->size[0] = A->size[0];
  emxEnsureCapacity_real_T(work, i);
  ns = A->size[0];
  for (i = 0; i < ns; i++) {
    work->data[i] = 0.0;
  }

  i = U->size[0] * U->size[1];
  U->size[0] = A->size[0];
  U->size[1] = A->size[0];
  emxEnsureCapacity_real_T(U, i);
  ns = A->size[0] * A->size[0];
  for (i = 0; i < ns; i++) {
    U->data[i] = 0.0;
  }

  std::memset(&V[0], 0, 16U * sizeof(double));
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
      i = nct;
    } else {
      i = nrt;
    }

    for (q = 0; q < i; q++) {
      qp1 = q + 2;
      qq = (q + n * q) + 1;
      nmq = (n - q) - 1;
      apply_transform = false;
      if (q + 1 <= nct) {
        nrm = xnrm2(nmq + 1, b_A, qq);
        if (nrm > 0.0) {
          apply_transform = true;
          if (b_A->data[qq - 1] < 0.0) {
            r = -nrm;
            b_s_data[q] = -nrm;
          } else {
            r = nrm;
            b_s_data[q] = nrm;
          }

          if (std::abs(r) >= 1.0020841800044864E-292) {
            nrm = 1.0 / r;
            ns = qq + nmq;
            for (k = qq; k <= ns; k++) {
              b_A->data[k - 1] *= nrm;
            }
          } else {
            ns = qq + nmq;
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

      for (jj = qp1; jj < 5; jj++) {
        qjj = q + n * (jj - 1);
        if (apply_transform) {
          nrm = 0.0;
          if (nmq + 1 >= 1) {
            ix = qq;
            iy = qjj;
            for (k = 0; k <= nmq; k++) {
              nrm += b_A->data[ix - 1] * b_A->data[iy];
              ix++;
              iy++;
            }
          }

          nrm = -(nrm / b_A->data[q + b_A->size[0] * q]);
          xaxpy(nmq + 1, nrm, qq, b_A, qjj + 1);
        }

        e[jj - 1] = b_A->data[qjj];
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

            for (jj = qp1; jj < 5; jj++) {
              b_xaxpy(nmq, e[jj - 1], b_A, (q + n * (jj - 1)) + 2, work, q + 2);
            }

            for (jj = qp1; jj < 5; jj++) {
              b_xaxpy(nmq, -e[jj - 1] / e[q + 1], work, q + 2, b_A, (q + n * (jj
                        - 1)) + 2);
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
      for (jj = nctp1; jj <= n; jj++) {
        for (ii = 0; ii < n; ii++) {
          U->data[ii + U->size[0] * (jj - 1)] = 0.0;
        }

        U->data[(jj + U->size[0] * (jj - 1)) - 1] = 1.0;
      }
    }

    for (q = nct; q >= 1; q--) {
      qp1 = q + 1;
      ns = n - q;
      nmq = ns + 1;
      qq = (q + n * (q - 1)) - 1;
      if (b_s_data[q - 1] != 0.0) {
        for (jj = qp1; jj <= n; jj++) {
          qjj = q + n * (jj - 1);
          nrm = 0.0;
          if (nmq >= 1) {
            ix = qq;
            iy = qjj;
            for (k = 0; k <= ns; k++) {
              nrm += U->data[ix] * U->data[iy - 1];
              ix++;
              iy++;
            }
          }

          nrm = -(nrm / U->data[qq]);
          xaxpy(nmq, nrm, qq + 1, U, qjj);
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
        for (jj = qp1; jj < 5; jj++) {
          qjj = (q + ((jj - 1) << 2)) + 2;
          c_xaxpy(3 - q, -(xdotc(3 - q, V, ns, V, qjj) / V[ns - 1]), ns, V, qjj);
        }
      }

      ns = q << 2;
      V[ns] = 0.0;
      V[ns + 1] = 0.0;
      V[ns + 2] = 0.0;
      V[ns + 3] = 0.0;
      V[q + (q << 2)] = 1.0;
    }

    nct = m;
    nctp1 = 0;
    snorm = 0.0;
    for (q = 0; q <= m; q++) {
      if (b_s_data[q] != 0.0) {
        nrm = std::abs(b_s_data[q]);
        r = b_s_data[q] / nrm;
        b_s_data[q] = nrm;
        if (q + 1 < m + 1) {
          e[q] /= r;
        }

        if (q + 1 <= n) {
          ns = n * q;
          i = ns + n;
          for (k = ns + 1; k <= i; k++) {
            U->data[k - 1] *= r;
          }
        }
      }

      if ((q + 1 < m + 1) && (e[q] != 0.0)) {
        nrm = std::abs(e[q]);
        r = nrm / e[q];
        e[q] = nrm;
        b_s_data[q + 1] *= r;
        ns = (q + 1) << 2;
        i = ns + 4;
        for (k = ns + 1; k <= i; k++) {
          V[k - 1] *= r;
        }
      }

      nrm = std::abs(b_s_data[q]);
      r = std::abs(e[q]);
      if ((nrm > r) || rtIsNaN(r)) {
        r = nrm;
      }

      if ((!(snorm > r)) && (!rtIsNaN(r))) {
        snorm = r;
      }
    }

    while ((m + 1 > 0) && (nctp1 < 75)) {
      ii = m;
      exitg1 = false;
      while (!(exitg1 || (ii == 0))) {
        nrm = std::abs(e[ii - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[ii - 1]) + std::
              abs(b_s_data[ii]))) || (nrm <= 1.0020841800044864E-292) || ((nctp1
              > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[ii - 1] = 0.0;
          exitg1 = true;
        } else {
          ii--;
        }
      }

      if (ii == m) {
        ns = 4;
      } else {
        qjj = m + 1;
        ns = m + 1;
        exitg1 = false;
        while ((!exitg1) && (ns >= ii)) {
          qjj = ns;
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

        if (qjj == ii) {
          ns = 3;
        } else if (qjj == m + 1) {
          ns = 1;
        } else {
          ns = 2;
          ii = qjj;
        }
      }

      switch (ns) {
       case 1:
        f = e[m - 1];
        e[m - 1] = 0.0;
        for (k = m; k >= ii + 1; k--) {
          xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
          if (k > ii + 1) {
            b = e[k - 2];
            f = -scale * b;
            e[k - 2] = b * sqds;
          }

          xrot(V, ((k - 1) << 2) + 1, (m << 2) + 1, sqds, scale);
        }
        break;

       case 2:
        f = e[ii - 1];
        e[ii - 1] = 0.0;
        for (k = ii + 1; k <= m + 1; k++) {
          xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
          b = e[k - 1];
          f = -scale * b;
          e[k - 1] = b * sqds;
          if (n >= 1) {
            ix = n * (k - 1);
            iy = n * (ii - 1);
            for (ns = 0; ns < n; ns++) {
              nrm = sqds * U->data[ix] + scale * U->data[iy];
              U->data[iy] = sqds * U->data[iy] - scale * U->data[ix];
              U->data[ix] = nrm;
              iy++;
              ix++;
            }
          }
        }
        break;

       case 3:
        scale = std::abs(b_s_data[m]);
        nrm = b_s_data[m - 1];
        r = std::abs(nrm);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        b = e[m - 1];
        r = std::abs(b);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(b_s_data[ii]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(e[ii]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        f = b_s_data[m] / scale;
        nrm /= scale;
        r = b / scale;
        sqds = b_s_data[ii] / scale;
        b = ((nrm + f) * (nrm - f) + r * r) / 2.0;
        nrm = f * r;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          r = std::sqrt(b * b + nrm);
          if (b < 0.0) {
            r = -r;
          }

          r = nrm / (b + r);
        } else {
          r = 0.0;
        }

        f = (sqds + f) * (sqds - f) + r;
        r = sqds * (e[ii] / scale);
        for (k = ii + 1; k <= m; k++) {
          xrotg(&f, &r, &sqds, &scale);
          if (k > ii + 1) {
            e[k - 2] = f;
          }

          nrm = e[k - 1];
          b = b_s_data[k - 1];
          e[k - 1] = sqds * nrm - scale * b;
          r = scale * b_s_data[k];
          b_s_data[k] *= sqds;
          xrot(V, ((k - 1) << 2) + 1, (k << 2) + 1, sqds, scale);
          b_s_data[k - 1] = sqds * b + scale * nrm;
          xrotg(&b_s_data[k - 1], &r, &sqds, &scale);
          f = sqds * e[k - 1] + scale * b_s_data[k];
          b_s_data[k] = -scale * e[k - 1] + sqds * b_s_data[k];
          r = scale * e[k];
          e[k] *= sqds;
          if ((k < n) && (n >= 1)) {
            ix = n * (k - 1);
            iy = n * k;
            for (ns = 0; ns < n; ns++) {
              nrm = sqds * U->data[ix] + scale * U->data[iy];
              U->data[iy] = sqds * U->data[iy] - scale * U->data[ix];
              U->data[ix] = nrm;
              iy++;
              ix++;
            }
          }
        }

        e[m - 1] = f;
        nctp1++;
        break;

       default:
        if (b_s_data[ii] < 0.0) {
          b_s_data[ii] = -b_s_data[ii];
          ns = ii << 2;
          i = ns + 4;
          for (k = ns + 1; k <= i; k++) {
            V[k - 1] = -V[k - 1];
          }
        }

        qp1 = ii + 1;
        while ((ii + 1 < nct + 1) && (b_s_data[ii] < b_s_data[qp1])) {
          nrm = b_s_data[ii];
          b_s_data[ii] = b_s_data[qp1];
          b_s_data[qp1] = nrm;
          xswap(V, (ii << 2) + 1, ((ii + 1) << 2) + 1);
          if (ii + 1 < n) {
            ix = n * ii;
            iy = n * (ii + 1);
            for (k = 0; k < n; k++) {
              nrm = U->data[ix];
              U->data[ix] = U->data[iy];
              U->data[iy] = nrm;
              ix++;
              iy++;
            }
          }

          ii = qp1;
          qp1++;
        }

        nctp1 = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&work);
  emxFree_real_T(&b_A);
  s_size[0] = minnp;
  if (0 <= minnp - 1) {
    std::memcpy(&s_data[0], &b_s_data[0], minnp * sizeof(double));
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
  int qjj;
  int ns;
  int n;
  int minnp;
  double b_s_data[2];
  double e[2];
  int nct;
  int nctp1;
  int q;
  int m;
  int qp1;
  int qq;
  int nmq_tmp;
  bool apply_transform;
  double nrm;
  int jj;
  double rt;
  int ix;
  int iy;
  int k;
  double snorm;
  bool exitg1;
  double f;
  double scale;
  double b;
  double sm;
  double sqds;
  emxInit_real_T(&b_A, 2);
  qjj = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = 2;
  emxEnsureCapacity_real_T(b_A, qjj);
  ns = A->size[0] * A->size[1];
  for (qjj = 0; qjj < ns; qjj++) {
    b_A->data[qjj] = A->data[qjj];
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
    std::memset(&b_s_data[0], 0, ns * sizeof(double));
  }

  e[0] = 0.0;
  e[1] = 0.0;
  qjj = U->size[0] * U->size[1];
  U->size[0] = A->size[0];
  U->size[1] = A->size[0];
  emxEnsureCapacity_real_T(U, qjj);
  ns = A->size[0] * A->size[0];
  for (qjj = 0; qjj < ns; qjj++) {
    U->data[qjj] = 0.0;
  }

  V[1] = 0.0;
  V[2] = 0.0;
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

    nctp1 = nct + 1;
    for (q = 0; q < nct; q++) {
      qp1 = q + 2;
      qq = (q + n * q) + 1;
      nmq_tmp = n - q;
      ns = nmq_tmp - 1;
      apply_transform = false;
      if (q + 1 <= nct) {
        nrm = xnrm2(nmq_tmp, b_A, qq);
        if (nrm > 0.0) {
          apply_transform = true;
          if (b_A->data[qq - 1] < 0.0) {
            rt = -nrm;
            b_s_data[q] = -nrm;
          } else {
            rt = nrm;
            b_s_data[q] = nrm;
          }

          if (std::abs(rt) >= 1.0020841800044864E-292) {
            nrm = 1.0 / rt;
            qjj = qq + ns;
            for (k = qq; k <= qjj; k++) {
              b_A->data[k - 1] *= nrm;
            }
          } else {
            qjj = qq + ns;
            for (k = qq; k <= qjj; k++) {
              b_A->data[k - 1] /= b_s_data[q];
            }
          }

          b_A->data[qq - 1]++;
          b_s_data[q] = -b_s_data[q];
        } else {
          b_s_data[q] = 0.0;
        }
      }

      for (jj = qp1; jj < 3; jj++) {
        qjj = q + n;
        if (apply_transform) {
          nrm = 0.0;
          if (nmq_tmp >= 1) {
            ix = qq;
            iy = qjj;
            for (k = 0; k <= ns; k++) {
              nrm += b_A->data[ix - 1] * b_A->data[iy];
              ix++;
              iy++;
            }
          }

          nrm = -(nrm / b_A->data[q + b_A->size[0] * q]);
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
      for (jj = nctp1; jj <= n; jj++) {
        for (nmq_tmp = 0; nmq_tmp < n; nmq_tmp++) {
          U->data[nmq_tmp + U->size[0] * (jj - 1)] = 0.0;
        }

        U->data[(jj + U->size[0] * (jj - 1)) - 1] = 1.0;
      }
    }

    for (q = nct; q >= 1; q--) {
      qp1 = q + 1;
      nmq_tmp = n - q;
      ns = nmq_tmp + 1;
      qq = (q + n * (q - 1)) - 1;
      if (b_s_data[q - 1] != 0.0) {
        for (jj = qp1; jj <= n; jj++) {
          qjj = q + n * (jj - 1);
          nrm = 0.0;
          if (ns >= 1) {
            ix = qq;
            iy = qjj;
            for (k = 0; k <= nmq_tmp; k++) {
              nrm += U->data[ix] * U->data[iy - 1];
              ix++;
              iy++;
            }
          }

          nrm = -(nrm / U->data[qq]);
          xaxpy(ns, nrm, qq + 1, U, qjj);
        }

        for (nmq_tmp = q; nmq_tmp <= n; nmq_tmp++) {
          U->data[(nmq_tmp + U->size[0] * (q - 1)) - 1] = -U->data[(nmq_tmp +
            U->size[0] * (q - 1)) - 1];
        }

        U->data[qq]++;
        if (0 <= q - 2) {
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
    V[3] = 1.0;
    V[1] = 0.0;
    V[0] = 1.0;
    for (q = 0; q < m; q++) {
      if (b_s_data[q] != 0.0) {
        rt = std::abs(b_s_data[q]);
        nrm = b_s_data[q] / rt;
        b_s_data[q] = rt;
        if (q + 1 < m) {
          e[0] /= nrm;
        }

        if (q + 1 <= n) {
          ns = n * q;
          qjj = ns + n;
          for (k = ns + 1; k <= qjj; k++) {
            U->data[k - 1] *= nrm;
          }
        }
      }

      if ((q + 1 < m) && (e[0] != 0.0)) {
        rt = std::abs(e[0]);
        nrm = rt / e[0];
        e[0] = rt;
        b_s_data[1] *= nrm;
        V[2] *= nrm;
        V[3] *= nrm;
      }
    }

    qp1 = m;
    nctp1 = 0;
    snorm = 0.0;
    for (nmq_tmp = 0; nmq_tmp < m; nmq_tmp++) {
      nrm = std::abs(b_s_data[nmq_tmp]);
      rt = std::abs(e[nmq_tmp]);
      if ((nrm > rt) || rtIsNaN(rt)) {
        rt = nrm;
      }

      if ((!(snorm > rt)) && (!rtIsNaN(rt))) {
        snorm = rt;
      }
    }

    while ((m > 0) && (nctp1 < 75)) {
      nmq_tmp = m - 1;
      exitg1 = false;
      while (!(exitg1 || (nmq_tmp == 0))) {
        nrm = std::abs(e[0]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[0]) + std::abs
              (b_s_data[1]))) || (nrm <= 1.0020841800044864E-292) || ((nctp1 >
              20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
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
        qjj = m - 1;
        for (k = qjj; k >= nmq_tmp + 1; k--) {
          xrotg(&b_s_data[0], &f, &b, &sm);
          b_xrot(V, ((m - 1) << 1) + 1, b, sm);
        }
        break;

       case 2:
        f = e[nmq_tmp - 1];
        e[nmq_tmp - 1] = 0.0;
        for (k = nmq_tmp + 1; k <= m; k++) {
          xrotg(&b_s_data[k - 1], &f, &b, &sm);
          nrm = e[k - 1];
          f = -sm * nrm;
          e[k - 1] = nrm * b;
          if (n >= 1) {
            ix = n * (k - 1);
            iy = n * (nmq_tmp - 1);
            for (ns = 0; ns < n; ns++) {
              nrm = b * U->data[ix] + sm * U->data[iy];
              U->data[iy] = b * U->data[iy] - sm * U->data[ix];
              U->data[ix] = nrm;
              iy++;
              ix++;
            }
          }
        }
        break;

       case 3:
        nrm = b_s_data[m - 1];
        scale = std::abs(nrm);
        rt = std::abs(b_s_data[0]);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        rt = std::abs(e[0]);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        rt = std::abs(b_s_data[nmq_tmp]);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        rt = std::abs(e[nmq_tmp]);
        if ((!(scale > rt)) && (!rtIsNaN(rt))) {
          scale = rt;
        }

        sm = nrm / scale;
        nrm = b_s_data[0] / scale;
        rt = e[0] / scale;
        sqds = b_s_data[nmq_tmp] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          rt = std::sqrt(b * b + nrm);
          if (b < 0.0) {
            rt = -rt;
          }

          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }

        f = (sqds + sm) * (sqds - sm) + rt;
        rt = sqds * (e[nmq_tmp] / scale);
        for (k = nmq_tmp + 1; k < 2; k++) {
          xrotg(&f, &rt, &b, &sm);
          f = b * b_s_data[0] + sm * e[0];
          rt = b * e[0] - sm * b_s_data[0];
          e[0] = rt;
          nrm = sm * b_s_data[1];
          b_s_data[1] *= b;
          b_xrot(V, 3, b, sm);
          b_s_data[0] = f;
          xrotg(&b_s_data[0], &nrm, &b, &sm);
          f = b * rt + sm * b_s_data[1];
          b_s_data[1] = -sm * rt + b * b_s_data[1];
          rt = sm * e[1];
          e[1] *= b;
          if (1 < n) {
            ix = 0;
            iy = n;
            for (ns = 0; ns < n; ns++) {
              nrm = b * U->data[ix] + sm * U->data[iy];
              U->data[iy] = b * U->data[iy] - sm * U->data[ix];
              U->data[ix] = nrm;
              iy++;
              ix++;
            }
          }
        }

        e[0] = f;
        nctp1++;
        break;

       default:
        if (b_s_data[nmq_tmp] < 0.0) {
          b_s_data[nmq_tmp] = -b_s_data[nmq_tmp];
          ns = nmq_tmp << 1;
          qjj = ns + 2;
          for (k = ns + 1; k <= qjj; k++) {
            V[k - 1] = -V[k - 1];
          }
        }

        while ((nmq_tmp + 1 < qp1) && (b_s_data[0] < b_s_data[1])) {
          rt = b_s_data[0];
          b_s_data[0] = b_s_data[1];
          b_s_data[1] = rt;
          b_xswap(V);
          if (1 < n) {
            iy = n;
            ix = 0;
            for (k = 0; k < n; k++) {
              nrm = U->data[ix];
              U->data[ix] = U->data[iy];
              U->data[iy] = nrm;
              ix++;
              iy++;
            }
          }

          nmq_tmp = 1;
        }

        nctp1 = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&b_A);
  s_size[0] = minnp;
  if (0 <= minnp - 1) {
    std::memcpy(&s_data[0], &b_s_data[0], minnp * sizeof(double));
  }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
