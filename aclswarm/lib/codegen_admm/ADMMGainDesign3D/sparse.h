//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sparse.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef SPARSE_H
#define SPARSE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T *
  this_colidx, const emxArray_int32_T *this_rowidx, int this_m, int this_n,
  emxArray_real_T *y);
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
extern void b_sparse_rdivide(const emxArray_real_T *a_d, const emxArray_int32_T *
  a_colidx, const emxArray_int32_T *a_rowidx, int a_m, int a_n,
  coder_internal_sparse *s);
extern void b_sparse_sparse(int m, int nzmaxval, emxArray_real_T *this_d,
  emxArray_int32_T *this_colidx, emxArray_int32_T *this_rowidx, int *this_m, int
  *this_maxnz);
extern bool c_sparse_full(const emxArray_boolean_T *this_d, const
  emxArray_int32_T *this_colidx);
extern void c_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_n);
extern double d_sparse_full(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx);
extern void permuteVector(const emxArray_int32_T *idx, emxArray_int32_T *y);
extern void sparse_abs(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, coder_internal_sparse_1 *
  s);
extern void sparse_copy(const emxArray_int32_T *this_colidx, const
  emxArray_int32_T *this_rowidx, int this_m, int this_n, coder_internal_sparse
  *t);
extern void sparse_full(const emxArray_real_T *this_d, const emxArray_int32_T
  *this_colidx, const emxArray_int32_T *this_rowidx, int this_m, emxArray_real_T
  *y);
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
extern void sparse_parenAssign(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_maxnz, const emxArray_real_T *rhs, coder_internal_sparse_1 *s);
extern void sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m, int *s_maxnz);
extern void sparse_plus(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, const emxArray_real_T *b_d, const
  emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int b_m,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_m);
extern void sparse_rdivide(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, int a_n,
  coder_internal_sparse *s);
extern void sparse_spallocLike(int m, emxArray_real_T *s_d, emxArray_int32_T
  *s_colidx, emxArray_int32_T *s_rowidx, int *s_m, int *s_maxnz);
extern void sparse_sparse(int m, int n, int nzmaxval, coder_internal_sparse
  *b_this);
extern void sparse_transpose(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_n, emxArray_real_T *y_d, emxArray_int32_T *y_colidx, emxArray_int32_T
  *y_rowidx, int *y_m, int *y_n);

#endif

//
// File trailer for sparse.h
//
// [EOF]
//
