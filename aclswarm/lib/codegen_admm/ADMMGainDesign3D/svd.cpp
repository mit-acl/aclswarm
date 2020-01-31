//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "svd.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "svd1.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                emxArray_real_T *S
//                double V[4]
// Return Type  : void
//
void c_svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *S,
           double V[4])
{
  int nx;
  bool p;
  int k;
  unsigned int unnamed_idx_0;
  double s_data[2];
  int s_size[1];
  double x;
  emxArray_real_T *r10;
  int i37;
  emxArray_real_T *U1;
  double V1[4];
  nx = A->size[0] << 1;
  p = true;
  for (k = 0; k < nx; k++) {
    if (p) {
      x = A->data[k];
      if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
        p = true;
      } else {
        p = false;
      }
    } else {
      p = false;
    }
  }

  if (p) {
    d_svd(A, U, s_data, s_size, V);
  } else {
    unnamed_idx_0 = (unsigned int)A->size[0];
    emxInit_real_T(&r10, 2);
    i37 = r10->size[0] * r10->size[1];
    r10->size[0] = (int)unnamed_idx_0;
    r10->size[1] = 2;
    emxEnsureCapacity_real_T(r10, i37);
    nx = (int)unnamed_idx_0 << 1;
    for (i37 = 0; i37 < nx; i37++) {
      r10->data[i37] = 0.0;
    }

    emxInit_real_T(&U1, 2);
    d_svd(r10, U1, s_data, s_size, V1);
    i37 = U->size[0] * U->size[1];
    U->size[0] = U1->size[0];
    U->size[1] = U1->size[1];
    emxEnsureCapacity_real_T(U, i37);
    nx = U1->size[0] * U1->size[1];
    emxFree_real_T(&r10);
    emxFree_real_T(&U1);
    for (i37 = 0; i37 < nx; i37++) {
      U->data[i37] = rtNaN;
    }

    nx = s_size[0];
    for (i37 = 0; i37 < nx; i37++) {
      s_data[i37] = rtNaN;
    }

    V[0] = rtNaN;
    V[1] = rtNaN;
    V[2] = rtNaN;
    V[3] = rtNaN;
  }

  i37 = S->size[0] * S->size[1];
  S->size[0] = U->size[1];
  S->size[1] = 2;
  emxEnsureCapacity_real_T(S, i37);
  nx = U->size[1] << 1;
  for (i37 = 0; i37 < nx; i37++) {
    S->data[i37] = 0.0;
  }

  i37 = s_size[0] - 1;
  for (k = 0; k <= i37; k++) {
    S->data[k + S->size[0] * k] = s_data[k];
  }
}

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                emxArray_real_T *S
//                double V[16]
// Return Type  : void
//
void svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *S,
         double V[16])
{
  int nx;
  bool p;
  int k;
  unsigned int unnamed_idx_0;
  double s_data[4];
  int s_size[1];
  double x;
  emxArray_real_T *r5;
  int i7;
  emxArray_real_T *U1;
  double V1[16];
  nx = A->size[0] << 2;
  p = true;
  for (k = 0; k < nx; k++) {
    if (p) {
      x = A->data[k];
      if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
        p = true;
      } else {
        p = false;
      }
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd(A, U, s_data, s_size, V);
  } else {
    unnamed_idx_0 = (unsigned int)A->size[0];
    emxInit_real_T(&r5, 2);
    i7 = r5->size[0] * r5->size[1];
    r5->size[0] = (int)unnamed_idx_0;
    r5->size[1] = 4;
    emxEnsureCapacity_real_T(r5, i7);
    nx = (int)unnamed_idx_0 << 2;
    for (i7 = 0; i7 < nx; i7++) {
      r5->data[i7] = 0.0;
    }

    emxInit_real_T(&U1, 2);
    b_svd(r5, U1, s_data, s_size, V1);
    i7 = U->size[0] * U->size[1];
    U->size[0] = U1->size[0];
    U->size[1] = U1->size[1];
    emxEnsureCapacity_real_T(U, i7);
    nx = U1->size[0] * U1->size[1];
    emxFree_real_T(&r5);
    emxFree_real_T(&U1);
    for (i7 = 0; i7 < nx; i7++) {
      U->data[i7] = rtNaN;
    }

    nx = s_size[0];
    for (i7 = 0; i7 < nx; i7++) {
      s_data[i7] = rtNaN;
    }

    for (i7 = 0; i7 < 16; i7++) {
      V[i7] = rtNaN;
    }
  }

  i7 = S->size[0] * S->size[1];
  S->size[0] = U->size[1];
  S->size[1] = 4;
  emxEnsureCapacity_real_T(S, i7);
  nx = U->size[1] << 2;
  for (i7 = 0; i7 < nx; i7++) {
    S->data[i7] = 0.0;
  }

  i7 = s_size[0] - 1;
  for (k = 0; k <= i7; k++) {
    S->data[k + S->size[0] * k] = s_data[k];
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
