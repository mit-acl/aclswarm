//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef SVD_H
#define SVD_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void c_svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *
                  S, double V[4]);
extern void svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *S,
                double V[16]);

#endif

//
// File trailer for svd.h
//
// [EOF]
//
