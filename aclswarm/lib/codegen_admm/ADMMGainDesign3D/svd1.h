//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.h
//
// MATLAB Coder version            : 4.3
// C/C++ source code generated on  : 02-Feb-2020 11:20:18
//
#ifndef SVD1_H
#define SVD1_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

// Function Declarations
extern void b_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[],
                  int s_size[1], double V[16]);
extern void d_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[],
                  int s_size[1], double V[4]);

#endif

//
// File trailer for svd1.h
//
// [EOF]
//
