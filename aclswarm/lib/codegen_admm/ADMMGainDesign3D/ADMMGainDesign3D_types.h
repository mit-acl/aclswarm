//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign3D_types.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef ADMMGAINDESIGN3D_TYPES_H
#define ADMMGAINDESIGN3D_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
typedef struct {
  int idx;
  int col;
  int row;
} b_struct_T;

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

typedef struct {
  emxArray_int32_T *f1;
} cell_wrap_3;

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

typedef struct {
  emxArray_real_T *d;
  emxArray_int32_T *colidx;
  emxArray_int32_T *rowidx;
  int m;
  int n;
} coder_internal_sparse;

typedef struct {
  emxArray_real_T *d;
  emxArray_int32_T *colidx;
  emxArray_int32_T *rowidx;
  int m;
  int maxnz;
} coder_internal_sparse_1;

struct emxArray_boolean_T
{
  bool *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

typedef struct {
  creal_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
} emxArray_creal_T;

struct emxArray_int8_T
{
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif

//
// File trailer for ADMMGainDesign3D_types.h
//
// [EOF]
//
