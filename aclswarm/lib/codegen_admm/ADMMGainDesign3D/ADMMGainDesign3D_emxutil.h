//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign3D_emxutil.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//
#ifndef ADMMGAINDESIGN3D_EMXUTIL_H
#define ADMMGAINDESIGN3D_EMXUTIL_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void c_emxCopyStruct_coder_internal_(coder_internal_sparse *dst, const
  coder_internal_sparse *src);
extern void c_emxFreeStruct_coder_internal_(coder_internal_sparse *pStruct);
extern void c_emxInitStruct_coder_internal_(coder_internal_sparse *pStruct);
extern void d_emxCopyStruct_coder_internal_(coder_internal_sparse_1 *dst, const
  coder_internal_sparse_1 *src);
extern void d_emxFreeStruct_coder_internal_(coder_internal_sparse_1 *pStruct);
extern void d_emxInitStruct_coder_internal_(coder_internal_sparse_1 *pStruct);
extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_creal_T(emxArray_creal_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxFreeMatrix_cell_wrap_3(cell_wrap_3 pMatrix[2]);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_creal_T(emxArray_creal_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInitMatrix_cell_wrap_3(cell_wrap_3 pMatrix[2]);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_creal_T(emxArray_creal_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for ADMMGainDesign3D_emxutil.h
//
// [EOF]
//
