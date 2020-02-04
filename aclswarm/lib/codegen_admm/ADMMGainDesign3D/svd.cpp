//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "svd.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"
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
  emxArray_real_T *r;
  double s_data[2];
  int s_size[1];
  unsigned int unnamed_idx_0;
  int i;
  emxArray_real_T *U1;
  double V1[4];
  nx = A->size[0] << 1;
  p = true;
  for (k = 0; k < nx; k++) {
    if ((!p) || (rtIsInf(A->data[k]) || rtIsNaN(A->data[k]))) {
      p = false;
    }
  }

  if (p) {
    d_svd(A, U, s_data, s_size, V);
  } else {
    emxInit_real_T(&r, 2);
    unnamed_idx_0 = static_cast<unsigned int>(A->size[0]);
    i = r->size[0] * r->size[1];
    r->size[0] = static_cast<int>(unnamed_idx_0);
    r->size[1] = 2;
    emxEnsureCapacity_real_T(r, i);
    nx = static_cast<int>(unnamed_idx_0) << 1;
    for (i = 0; i < nx; i++) {
      r->data[i] = 0.0;
    }

    emxInit_real_T(&U1, 2);
    d_svd(r, U1, s_data, s_size, V1);
    i = U->size[0] * U->size[1];
    U->size[0] = U1->size[0];
    U->size[1] = U1->size[1];
    emxEnsureCapacity_real_T(U, i);
    nx = U1->size[0] * U1->size[1];
    emxFree_real_T(&r);
    emxFree_real_T(&U1);
    for (i = 0; i < nx; i++) {
      U->data[i] = rtNaN;
    }

    nx = s_size[0];
    for (i = 0; i < nx; i++) {
      s_data[i] = rtNaN;
    }

    V[0] = rtNaN;
    V[1] = rtNaN;
    V[2] = rtNaN;
    V[3] = rtNaN;
  }

  i = S->size[0] * S->size[1];
  S->size[0] = U->size[1];
  S->size[1] = 2;
  emxEnsureCapacity_real_T(S, i);
  nx = U->size[1] << 1;
  for (i = 0; i < nx; i++) {
    S->data[i] = 0.0;
  }

  i = s_size[0] - 1;
  for (k = 0; k <= i; k++) {
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
  emxArray_real_T *r;
  double s_data[4];
  int s_size[1];
  unsigned int unnamed_idx_0;
  int i;
  emxArray_real_T *U1;
  double V1[16];
  nx = A->size[0] << 2;
  p = true;
  for (k = 0; k < nx; k++) {
    if ((!p) || (rtIsInf(A->data[k]) || rtIsNaN(A->data[k]))) {
      p = false;
    }
  }

  if (p) {
    b_svd(A, U, s_data, s_size, V);
  } else {
    emxInit_real_T(&r, 2);
    unnamed_idx_0 = static_cast<unsigned int>(A->size[0]);
    i = r->size[0] * r->size[1];
    r->size[0] = static_cast<int>(unnamed_idx_0);
    r->size[1] = 4;
    emxEnsureCapacity_real_T(r, i);
    nx = static_cast<int>(unnamed_idx_0) << 2;
    for (i = 0; i < nx; i++) {
      r->data[i] = 0.0;
    }

    emxInit_real_T(&U1, 2);
    b_svd(r, U1, s_data, s_size, V1);
    i = U->size[0] * U->size[1];
    U->size[0] = U1->size[0];
    U->size[1] = U1->size[1];
    emxEnsureCapacity_real_T(U, i);
    nx = U1->size[0] * U1->size[1];
    emxFree_real_T(&r);
    emxFree_real_T(&U1);
    for (i = 0; i < nx; i++) {
      U->data[i] = rtNaN;
    }

    nx = s_size[0];
    for (i = 0; i < nx; i++) {
      s_data[i] = rtNaN;
    }

    for (i = 0; i < 16; i++) {
      V[i] = rtNaN;
    }
  }

  i = S->size[0] * S->size[1];
  S->size[0] = U->size[1];
  S->size[1] = 4;
  emxEnsureCapacity_real_T(S, i);
  nx = U->size[1] << 2;
  for (i = 0; i < nx; i++) {
    S->data[i] = 0.0;
  }

  i = s_size[0] - 1;
  for (k = 0; k <= i; k++) {
    S->data[k + S->size[0] * k] = s_data[k];
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
