/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ADMMGainDesign3D_types.h
 *
 * Code generation for function 'ADMMGainDesign3D_types'
 *
 */

#ifndef ADMMGAINDESIGN3D_TYPES_H
#define ADMMGAINDESIGN3D_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#include "cs.h"

/* Type Definitions */
struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

struct cell_wrap_3
{
  emxArray_int32_T *f1;
};

struct emxArray_int8_T
{
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

struct coder_internal_sparse
{
  emxArray_real_T *d;
  emxArray_int32_T *colidx;
  emxArray_int32_T *rowidx;
  int m;
  int n;
};

struct emxArray_boolean_T
{
  bool *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

struct coder_internal_sparse_1
{
  emxArray_real_T *d;
  emxArray_int32_T *colidx;
  emxArray_int32_T *rowidx;
  int m;
  int maxnz;
};

struct emxArray_creal_T
{
  creal_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif

/* End of code generation (ADMMGainDesign3D_types.h) */
