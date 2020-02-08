/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * binOp.h
 *
 * Code generation for function 'binOp'
 *
 */

#ifndef BINOP_H
#define BINOP_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void allocEqsizeBinop(const emxArray_int32_T *a_colidx, const
  emxArray_int32_T *b_colidx, int sn, int sm, coder_internal_sparse *s);

#endif

/* End of code generation (binOp.h) */
