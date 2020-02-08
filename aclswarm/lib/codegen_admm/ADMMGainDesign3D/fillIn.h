/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fillIn.h
 *
 * Code generation for function 'fillIn'
 *
 */

#ifndef FILLIN_H
#define FILLIN_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_sparse_fillIn(coder_internal_sparse *b_this);
extern void c_sparse_fillIn(coder_internal_sparse_1 *b_this);
extern void sparse_fillIn(coder_internal_sparse *b_this);

#endif

/* End of code generation (fillIn.h) */
