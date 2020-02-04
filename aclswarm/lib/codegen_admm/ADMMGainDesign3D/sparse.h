//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sparse.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef SPARSE_H
#define SPARSE_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern bool b_sparse_full(const emxArray_boolean_T *this_d, const
  emxArray_int32_T *this_colidx);
extern void b_sparse_minus(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d, const
  emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int b_m,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m);
extern void b_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_n, emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T
  *s_rowidx, int *s_m);
extern void b_sparse_plus(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d, const
  emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int b_m, int b_n,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_n);
extern void b_sparse_times(const emxArray_real_T *b_d, const emxArray_int32_T
  *b_colidx, const emxArray_int32_T *b_rowidx, int b_m, int b_n,
  coder_internal_sparse *s);
extern double c_sparse_full(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx);
extern void c_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_n);
extern void sparse_abs(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, coder_internal_sparse_1 *
  s);
extern void sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
  *this_colidx, const emxArray_int32_T *this_rowidx, int this_m, int this_n,
  emxArray_real_T *y);
extern void sparse_lt(const emxArray_real_T *a_d, const emxArray_int32_T
                      *a_colidx, emxArray_boolean_T *s_d, emxArray_int32_T
                      *s_colidx, emxArray_int32_T *s_rowidx);
extern void sparse_minus(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d, const
  emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int b_m, int b_n,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_n);
extern void sparse_mldivide(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n, const
  emxArray_real_T *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T
  *b_rowidx, int b_m, coder_internal_sparse_1 *y);
extern void sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m);
extern void sparse_plus(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d, const
  emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int b_m,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m);
extern void sparse_rdivide(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, int a_n,
  coder_internal_sparse *s);
extern void sparse_spallocLike(int m, int nzmax, emxArray_real_T *s_d,
  emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx, int *s_m, int *s_maxnz);
extern void sparse_times(const emxArray_real_T *b_d, const emxArray_int32_T
  *b_colidx, const emxArray_int32_T *b_rowidx, int b_m, int b_n,
  coder_internal_sparse *s);
extern void sparse_transpose(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_n, coder_internal_sparse *y);

#endif

//
// File trailer for sparse.h
//
// [EOF]
//
