/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * find.h
 *
 * Code generation for function 'find'
 *
 */

#ifndef FIND_H
#define FIND_H

/* Include files */
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ADMMGainDesign3D_types.h"

/* Function Declarations */
extern void b_eml_find(const emxArray_boolean_T *x, emxArray_int32_T *i);
extern void eml_find(const emxArray_real_T *x, emxArray_int32_T *i,
                     emxArray_int32_T *j);

#endif

/* End of code generation (find.h) */
