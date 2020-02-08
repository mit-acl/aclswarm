/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mtimes.cpp
 *
 * Code generation for function 'mtimes'
 *
 */

/* Include files */
#include "mtimes.h"
#include "ADMMGainDesign3D.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "fillIn.h"
#include "introsort.h"
#include "rt_nonfinite.h"
#include "sparse.h"

/* Function Definitions */
void b_sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
                     *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, const
                     emxArray_real_T *b_d, const emxArray_int32_T *b_colidx,
                     const emxArray_int32_T *b_rowidx, coder_internal_sparse_1
                     *c)
{
  emxArray_int32_T *ccolidx;
  int i;
  int pb;
  emxArray_int32_T *flag;
  int cnnz;
  int bcidx;
  int aend;
  int i1;
  emxArray_real_T *wd;
  bool needSort;
  int paend;
  int pa;
  double bd;
  emxInit_int32_T(&ccolidx, 1);
  i = ccolidx->size[0];
  ccolidx->size[0] = b_colidx->size[0];
  emxEnsureCapacity_int32_T(ccolidx, i);
  pb = b_colidx->size[0];
  for (i = 0; i < pb; i++) {
    ccolidx->data[i] = 0;
  }

  emxInit_int32_T(&flag, 1);
  i = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i);
  for (i = 0; i < a_m; i++) {
    flag->data[i] = 0;
  }

  cnnz = 0;
  bcidx = b_colidx->data[0];
  ccolidx->data[0] = 1;
  while ((bcidx < b_colidx->data[1]) && (cnnz <= a_m)) {
    pb = b_rowidx->data[bcidx - 1];
    aend = a_colidx->data[pb] - 1;
    i = a_colidx->data[pb - 1];
    for (pb = i; pb <= aend; pb++) {
      i1 = a_rowidx->data[pb - 1] - 1;
      if (flag->data[i1] != 1) {
        flag->data[i1] = 1;
        cnnz++;
      }
    }

    bcidx++;
  }

  ccolidx->data[1] = cnnz + 1;
  sparse_spallocLike(a_m, cnnz, c->d, c->colidx, c->rowidx, &c->m, &c->maxnz);
  i = c->colidx->size[0];
  c->colidx->size[0] = ccolidx->size[0];
  emxEnsureCapacity_int32_T(c->colidx, i);
  pb = ccolidx->size[0];
  for (i = 0; i < pb; i++) {
    c->colidx->data[i] = ccolidx->data[i];
  }

  emxInit_real_T(&wd, 1);
  i = wd->size[0];
  wd->size[0] = a_m;
  emxEnsureCapacity_real_T(wd, i);
  i = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i);
  for (i = 0; i < a_m; i++) {
    flag->data[i] = 0;
  }

  cnnz = -1;
  needSort = false;
  if (b_colidx->data[1] - 1 != 0) {
    if (b_colidx->data[1] - 1 == 1) {
      paend = a_colidx->data[b_rowidx->data[0]] - 1;
      i = a_colidx->data[b_rowidx->data[0] - 1];
      for (pa = i; pa <= paend; pa++) {
        cnnz++;
        i1 = a_rowidx->data[pa - 1];
        c->rowidx->data[cnnz] = i1;
        wd->data[i1 - 1] = a_d->data[pa - 1] * b_d->data[0];
      }
    } else {
      paend = a_colidx->data[b_rowidx->data[0]] - 1;
      i = a_colidx->data[b_rowidx->data[0] - 1];
      for (pa = i; pa <= paend; pa++) {
        cnnz++;
        pb = a_rowidx->data[pa - 1];
        bcidx = pb - 1;
        flag->data[bcidx] = cnnz + 1;
        c->rowidx->data[cnnz] = pb;
        wd->data[bcidx] = a_d->data[pa - 1] * b_d->data[0];
      }

      for (pb = 1; pb + 1 < b_colidx->data[1]; pb++) {
        bd = b_d->data[pb];
        paend = a_colidx->data[b_rowidx->data[pb]] - 1;
        i = a_colidx->data[b_rowidx->data[pb] - 1];
        for (pa = i; pa <= paend; pa++) {
          i1 = a_rowidx->data[pa - 1];
          aend = i1 - 1;
          if (flag->data[aend] < 1) {
            cnnz++;
            flag->data[aend] = cnnz + 1;
            c->rowidx->data[cnnz] = i1;
            wd->data[aend] = a_d->data[pa - 1] * bd;
            needSort = true;
          } else {
            wd->data[aend] += a_d->data[pa - 1] * bd;
          }
        }
      }
    }
  }

  emxFree_int32_T(&flag);
  pb = ccolidx->data[1] - 1;
  bcidx = ccolidx->data[0];
  if (needSort) {
    b_introsort(c->rowidx, ccolidx->data[0], ccolidx->data[1] - 1);
  }

  emxFree_int32_T(&ccolidx);
  for (aend = bcidx; aend <= pb; aend++) {
    c->d->data[aend - 1] = wd->data[c->rowidx->data[aend - 1] - 1];
  }

  emxFree_real_T(&wd);
  c_sparse_fillIn(c);
}

void sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                   const emxArray_int32_T *a_rowidx, int a_m, const
                   emxArray_real_T *b_d, const emxArray_int32_T *b_colidx, const
                   emxArray_int32_T *b_rowidx, int b_n, coder_internal_sparse *c)
{
  emxArray_int32_T *ccolidx;
  int i;
  int numalloc;
  emxArray_int32_T *flag;
  int cnnz;
  int j;
  int exitg1;
  int bcidx;
  int cstart;
  int cmax;
  int aend;
  int i1;
  emxArray_real_T *wd;
  int pb;
  bool needSort;
  int pcstart;
  double bd;
  emxInit_int32_T(&ccolidx, 1);
  i = ccolidx->size[0];
  ccolidx->size[0] = b_colidx->size[0];
  emxEnsureCapacity_int32_T(ccolidx, i);
  numalloc = b_colidx->size[0];
  for (i = 0; i < numalloc; i++) {
    ccolidx->data[i] = 0;
  }

  emxInit_int32_T(&flag, 1);
  i = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i);
  for (i = 0; i < a_m; i++) {
    flag->data[i] = 0;
  }

  cnnz = 0;
  j = 0;
  do {
    exitg1 = 0;
    if (j <= b_n - 1) {
      bcidx = b_colidx->data[j];
      cstart = cnnz;
      cmax = cnnz + a_m;
      ccolidx->data[j] = cnnz + 1;
      while ((bcidx < b_colidx->data[j + 1]) && (cnnz <= cmax)) {
        numalloc = b_rowidx->data[bcidx - 1];
        aend = a_colidx->data[numalloc] - 1;
        i = a_colidx->data[numalloc - 1];
        for (numalloc = i; numalloc <= aend; numalloc++) {
          i1 = a_rowidx->data[numalloc - 1] - 1;
          if (flag->data[i1] != j + 1) {
            flag->data[i1] = j + 1;
            cnnz++;
          }
        }

        bcidx++;
      }

      if (cnnz < cstart) {
        exitg1 = 1;
      } else {
        j++;
      }
    } else {
      ccolidx->data[b_n] = cnnz + 1;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  c->m = a_m;
  c->n = b_n;
  if (cnnz >= 1) {
    numalloc = cnnz;
  } else {
    numalloc = 1;
  }

  i = c->d->size[0];
  c->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(c->d, i);
  for (i = 0; i < numalloc; i++) {
    c->d->data[i] = 0.0;
  }

  i = c->colidx->size[0];
  c->colidx->size[0] = b_n + 1;
  emxEnsureCapacity_int32_T(c->colidx, i);
  c->colidx->data[0] = 1;
  i = c->rowidx->size[0];
  c->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(c->rowidx, i);
  for (i = 0; i < numalloc; i++) {
    c->rowidx->data[i] = 0;
  }

  for (numalloc = 0; numalloc < b_n; numalloc++) {
    c->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(c);
  i = c->colidx->size[0];
  c->colidx->size[0] = ccolidx->size[0];
  emxEnsureCapacity_int32_T(c->colidx, i);
  numalloc = ccolidx->size[0];
  for (i = 0; i < numalloc; i++) {
    c->colidx->data[i] = ccolidx->data[i];
  }

  emxInit_real_T(&wd, 1);
  i = wd->size[0];
  wd->size[0] = a_m;
  emxEnsureCapacity_real_T(wd, i);
  i = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i);
  for (i = 0; i < a_m; i++) {
    flag->data[i] = 0;
  }

  pb = 0;
  cnnz = -1;
  for (j = 0; j < b_n; j++) {
    aend = j + 1;
    needSort = false;
    pcstart = cnnz + 2;
    numalloc = (b_colidx->data[aend] - pb) - 1;
    if (numalloc != 0) {
      if (numalloc == 1) {
        cstart = a_colidx->data[b_rowidx->data[pb]] - 1;
        i = a_colidx->data[b_rowidx->data[pb] - 1];
        for (cmax = i; cmax <= cstart; cmax++) {
          cnnz++;
          i1 = a_rowidx->data[cmax - 1];
          c->rowidx->data[cnnz] = i1;
          wd->data[i1 - 1] = a_d->data[cmax - 1] * b_d->data[pb];
        }

        pb++;
      } else {
        cstart = a_colidx->data[b_rowidx->data[pb]] - 1;
        i = a_colidx->data[b_rowidx->data[pb] - 1];
        for (cmax = i; cmax <= cstart; cmax++) {
          cnnz++;
          numalloc = a_rowidx->data[cmax - 1];
          bcidx = numalloc - 1;
          flag->data[bcidx] = cnnz + 1;
          c->rowidx->data[cnnz] = numalloc;
          wd->data[bcidx] = a_d->data[cmax - 1] * b_d->data[pb];
        }

        for (pb++; pb + 1 < b_colidx->data[aend]; pb++) {
          bd = b_d->data[pb];
          cstart = a_colidx->data[b_rowidx->data[pb]] - 1;
          i = a_colidx->data[b_rowidx->data[pb] - 1];
          for (cmax = i; cmax <= cstart; cmax++) {
            i1 = a_rowidx->data[cmax - 1];
            numalloc = i1 - 1;
            if (flag->data[numalloc] < pcstart) {
              cnnz++;
              flag->data[numalloc] = cnnz + 1;
              c->rowidx->data[cnnz] = i1;
              wd->data[numalloc] = a_d->data[cmax - 1] * bd;
              needSort = true;
            } else {
              wd->data[numalloc] += a_d->data[cmax - 1] * bd;
            }
          }
        }
      }
    }

    bcidx = ccolidx->data[aend] - 1;
    pcstart = ccolidx->data[aend - 1];
    if (needSort) {
      b_introsort(c->rowidx, ccolidx->data[aend - 1], ccolidx->data[aend] - 1);
    }

    for (numalloc = pcstart; numalloc <= bcidx; numalloc++) {
      c->d->data[numalloc - 1] = wd->data[c->rowidx->data[numalloc - 1] - 1];
    }
  }

  emxFree_int32_T(&flag);
  emxFree_int32_T(&ccolidx);
  emxFree_real_T(&wd);
  b_sparse_fillIn(c);
}

/* End of code generation (mtimes.cpp) */
