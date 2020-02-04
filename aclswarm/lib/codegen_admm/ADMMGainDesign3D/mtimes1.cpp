//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes1.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "mtimes1.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *C
// Return Type  : void
//
void b_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
              emxArray_real_T *C)
{
  int m;
  int inner;
  int n;
  int i;
  int j;
  int coffset;
  int boffset;
  int b_i;
  int k;
  int aoffset;
  double temp;
  m = A->size[0];
  inner = A->size[1];
  n = B->size[1];
  i = C->size[0] * C->size[1];
  C->size[0] = A->size[0];
  C->size[1] = B->size[1];
  emxEnsureCapacity_real_T(C, i);
  for (j = 0; j < n; j++) {
    coffset = j * m;
    boffset = j * inner;
    for (b_i = 0; b_i < m; b_i++) {
      C->data[coffset + b_i] = 0.0;
    }

    for (k = 0; k < inner; k++) {
      aoffset = k * m;
      temp = B->data[boffset + k];
      for (b_i = 0; b_i < m; b_i++) {
        i = coffset + b_i;
        C->data[i] += temp * A->data[aoffset + b_i];
      }
    }
  }
}

//
// Arguments    : const emxArray_creal_T *A
//                const emxArray_creal_T *B
//                emxArray_creal_T *C
// Return Type  : void
//
void mtimes(const emxArray_creal_T *A, const emxArray_creal_T *B,
            emxArray_creal_T *C)
{
  int m;
  int inner;
  int n;
  int i;
  int j;
  int coffset;
  int boffset;
  int b_i;
  int k;
  int aoffset;
  int temp_re_tmp;
  double temp_re;
  double temp_im;
  m = A->size[0];
  inner = A->size[1];
  n = B->size[1];
  i = C->size[0] * C->size[1];
  C->size[0] = A->size[0];
  C->size[1] = B->size[1];
  emxEnsureCapacity_creal_T(C, i);
  for (j = 0; j < n; j++) {
    coffset = j * m;
    boffset = j * inner;
    for (b_i = 0; b_i < m; b_i++) {
      i = coffset + b_i;
      C->data[i].re = 0.0;
      C->data[i].im = 0.0;
    }

    for (k = 0; k < inner; k++) {
      aoffset = k * m;
      temp_re_tmp = boffset + k;
      temp_re = B->data[temp_re_tmp].re;
      temp_im = B->data[temp_re_tmp].im;
      for (b_i = 0; b_i < m; b_i++) {
        temp_re_tmp = aoffset + b_i;
        i = coffset + b_i;
        C->data[i].re += temp_re * A->data[temp_re_tmp].re - temp_im * A->
          data[temp_re_tmp].im;
        C->data[i].im += temp_re * A->data[temp_re_tmp].im + temp_im * A->
          data[temp_re_tmp].re;
      }
    }
  }
}

//
// File trailer for mtimes1.cpp
//
// [EOF]
//
