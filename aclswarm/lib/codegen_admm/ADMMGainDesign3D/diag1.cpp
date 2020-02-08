/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diag1.cpp
 *
 * Code generation for function 'diag1'
 *
 */

/* Include files */
#include "diag1.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_diag(const emxArray_boolean_T *v, emxArray_boolean_T *d)
{
  int nv;
  int j;
  int loop_ub;
  nv = v->size[0];
  j = d->size[0] * d->size[1];
  d->size[0] = v->size[0];
  d->size[1] = v->size[0];
  emxEnsureCapacity_boolean_T(d, j);
  loop_ub = v->size[0] * v->size[0];
  for (j = 0; j < loop_ub; j++) {
    d->data[j] = false;
  }

  for (j = 0; j < nv; j++) {
    d->data[j + d->size[0] * j] = v->data[j];
  }
}

void c_diag(const emxArray_creal_T *v, emxArray_creal_T *d)
{
  int u0;
  int u1;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    u1 = d->size[0];
    d->size[0] = 1;
    emxEnsureCapacity_creal_T(d, u1);
    d->data[0] = v->data[0];
  } else {
    u0 = v->size[0];
    u1 = v->size[1];
    if (u0 < u1) {
      u1 = u0;
    }

    if (0 < v->size[1]) {
      u0 = u1;
    } else {
      u0 = 0;
    }

    u1 = d->size[0];
    d->size[0] = u0;
    emxEnsureCapacity_creal_T(d, u1);
    u1 = u0 - 1;
    for (u0 = 0; u0 <= u1; u0++) {
      d->data[u0] = v->data[u0 + v->size[0] * u0];
    }
  }
}

void d_diag(const emxArray_creal_T *v, emxArray_creal_T *d)
{
  int nv;
  int j;
  int loop_ub;
  nv = v->size[0];
  j = d->size[0] * d->size[1];
  d->size[0] = v->size[0];
  d->size[1] = v->size[0];
  emxEnsureCapacity_creal_T(d, j);
  loop_ub = v->size[0] * v->size[0];
  for (j = 0; j < loop_ub; j++) {
    d->data[j].re = 0.0;
    d->data[j].im = 0.0;
  }

  for (j = 0; j < nv; j++) {
    d->data[j + d->size[0] * j] = v->data[j];
  }
}

void diag(const emxArray_boolean_T *v, emxArray_boolean_T *d)
{
  int u0;
  int u1;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    u1 = d->size[0];
    d->size[0] = 1;
    emxEnsureCapacity_boolean_T(d, u1);
    d->data[0] = v->data[0];
  } else {
    u0 = v->size[0];
    u1 = v->size[1];
    if (u0 < u1) {
      u1 = u0;
    }

    if (0 < v->size[1]) {
      u0 = u1;
    } else {
      u0 = 0;
    }

    u1 = d->size[0];
    d->size[0] = u0;
    emxEnsureCapacity_boolean_T(d, u1);
    u1 = u0 - 1;
    for (u0 = 0; u0 <= u1; u0++) {
      d->data[u0] = v->data[u0 + v->size[0] * u0];
    }
  }
}

/* End of code generation (diag1.cpp) */
