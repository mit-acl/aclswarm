//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ADMMGainDesign3D_emxutil.cpp
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//

// Include Files
#include "ADMMGainDesign3D_emxutil.h"
#include "ADMMGainDesign3D.h"
#include "rt_nonfinite.h"
#include <cstdlib>
#include <cstring>

// Function Declarations
static void emxFreeStruct_cell_wrap_3(cell_wrap_3 *pStruct);
static void emxInitStruct_cell_wrap_3(cell_wrap_3 *pStruct);

// Function Definitions

//
// Arguments    : cell_wrap_3 *pStruct
// Return Type  : void
//
static void emxFreeStruct_cell_wrap_3(cell_wrap_3 *pStruct)
{
  emxFree_int32_T(&pStruct->f1);
}

//
// Arguments    : cell_wrap_3 *pStruct
// Return Type  : void
//
static void emxInitStruct_cell_wrap_3(cell_wrap_3 *pStruct)
{
  emxInit_int32_T(&pStruct->f1, 1);
}

//
// Arguments    : coder_internal_sparse *pStruct
// Return Type  : void
//
void c_emxFreeStruct_coder_internal_(coder_internal_sparse *pStruct)
{
  emxFree_real_T(&pStruct->d);
  emxFree_int32_T(&pStruct->colidx);
  emxFree_int32_T(&pStruct->rowidx);
}

//
// Arguments    : coder_internal_sparse *pStruct
// Return Type  : void
//
void c_emxInitStruct_coder_internal_(coder_internal_sparse *pStruct)
{
  emxInit_real_T(&pStruct->d, 1);
  emxInit_int32_T(&pStruct->colidx, 1);
  emxInit_int32_T(&pStruct->rowidx, 1);
}

//
// Arguments    : coder_internal_sparse_1 *pStruct
// Return Type  : void
//
void d_emxFreeStruct_coder_internal_(coder_internal_sparse_1 *pStruct)
{
  emxFree_real_T(&pStruct->d);
  emxFree_int32_T(&pStruct->colidx);
  emxFree_int32_T(&pStruct->rowidx);
}

//
// Arguments    : coder_internal_sparse_1 *pStruct
// Return Type  : void
//
void d_emxInitStruct_coder_internal_(coder_internal_sparse_1 *pStruct)
{
  emxInit_real_T(&pStruct->d, 1);
  emxInit_int32_T(&pStruct->colidx, 1);
  emxInit_int32_T(&pStruct->rowidx, 1);
}

//
// Arguments    : emxArray_boolean_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = std::calloc(static_cast<unsigned int>(i), sizeof(bool));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(bool) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (bool *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : emxArray_creal_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_creal_T(emxArray_creal_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = std::calloc(static_cast<unsigned int>(i), sizeof(creal_T));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(creal_T) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (creal_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : emxArray_int32_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = std::calloc(static_cast<unsigned int>(i), sizeof(int));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(int) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : emxArray_int8_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = std::calloc(static_cast<unsigned int>(i), sizeof(signed char));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(signed char) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (signed char *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : emxArray_real_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = std::calloc(static_cast<unsigned int>(i), sizeof(double));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(double) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : cell_wrap_3 pMatrix[2]
// Return Type  : void
//
void emxFreeMatrix_cell_wrap_3(cell_wrap_3 pMatrix[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    emxFreeStruct_cell_wrap_3(&pMatrix[i]);
  }
}

//
// Arguments    : emxArray_boolean_T **pEmxArray
// Return Type  : void
//
void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (bool *)NULL) && (*pEmxArray)->canFreeData) {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

//
// Arguments    : emxArray_creal_T **pEmxArray
// Return Type  : void
//
void emxFree_creal_T(emxArray_creal_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_creal_T *)NULL) {
    if (((*pEmxArray)->data != (creal_T *)NULL) && (*pEmxArray)->canFreeData) {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_creal_T *)NULL;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
// Return Type  : void
//
void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

//
// Arguments    : emxArray_int8_T **pEmxArray
// Return Type  : void
//
void emxFree_int8_T(emxArray_int8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int8_T *)NULL) {
    if (((*pEmxArray)->data != (signed char *)NULL) && (*pEmxArray)->canFreeData)
    {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_int8_T *)NULL;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
// Return Type  : void
//
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

//
// Arguments    : cell_wrap_3 pMatrix[2]
// Return Type  : void
//
void emxInitMatrix_cell_wrap_3(cell_wrap_3 pMatrix[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    emxInitStruct_cell_wrap_3(&pMatrix[i]);
  }
}

//
// Arguments    : emxArray_boolean_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)std::malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (bool *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)std::malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_creal_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_creal_T(emxArray_creal_T **pEmxArray, int numDimensions)
{
  emxArray_creal_T *emxArray;
  int i;
  *pEmxArray = (emxArray_creal_T *)std::malloc(sizeof(emxArray_creal_T));
  emxArray = *pEmxArray;
  emxArray->data = (creal_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)std::malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)std::malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)std::malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_int8_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions)
{
  emxArray_int8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int8_T *)std::malloc(sizeof(emxArray_int8_T));
  emxArray = *pEmxArray;
  emxArray->data = (signed char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)std::malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)std::malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)std::malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// File trailer for ADMMGainDesign3D_emxutil.cpp
//
// [EOF]
//
