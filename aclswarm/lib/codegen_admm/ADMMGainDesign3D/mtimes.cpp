//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "mtimes.h"
#include "fillIn.h"
#include "ADMMGainDesign3D_emxutil.h"
#include "introsort.h"
#include "sparse.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                int a_m
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                coder_internal_sparse_1 *c
// Return Type  : void
//
void b_sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
                     *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, const
                     emxArray_real_T *b_d, const emxArray_int32_T *b_colidx,
                     const emxArray_int32_T *b_rowidx, coder_internal_sparse_1
                     *c)
{
  emxArray_int32_T *ccolidx;
  unsigned int unnamed_idx_0;
  int i17;
  int bcidx;
  emxArray_int32_T *flag;
  int cnnz;
  int paend;
  int pa;
  emxArray_real_T *wd;
  bool needSort;
  double bd;
  emxInit_int32_T(&ccolidx, 1);
  unnamed_idx_0 = (unsigned int)b_colidx->size[0];
  i17 = ccolidx->size[0];
  ccolidx->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity_int32_T(ccolidx, i17);
  bcidx = (int)unnamed_idx_0;
  for (i17 = 0; i17 < bcidx; i17++) {
    ccolidx->data[i17] = 0;
  }

  emxInit_int32_T(&flag, 1);
  i17 = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i17);
  for (i17 = 0; i17 < a_m; i17++) {
    flag->data[i17] = 0;
  }

  cnnz = 0;
  bcidx = b_colidx->data[0];
  ccolidx->data[0] = 1;
  while ((bcidx < b_colidx->data[1]) && (cnnz <= a_m)) {
    paend = a_colidx->data[b_rowidx->data[bcidx - 1]] - 1;
    i17 = a_colidx->data[b_rowidx->data[bcidx - 1] - 1];
    for (pa = i17; pa <= paend; pa++) {
      if (flag->data[a_rowidx->data[pa - 1] - 1] != 1) {
        flag->data[a_rowidx->data[pa - 1] - 1] = 1;
        cnnz++;
      }
    }

    bcidx++;
  }

  ccolidx->data[1] = cnnz + 1;
  b_sparse_sparse(a_m, cnnz, c->d, c->colidx, c->rowidx, &c->m, &c->maxnz);
  i17 = c->colidx->size[0];
  c->colidx->size[0] = ccolidx->size[0];
  emxEnsureCapacity_int32_T(c->colidx, i17);
  bcidx = ccolidx->size[0];
  for (i17 = 0; i17 < bcidx; i17++) {
    c->colidx->data[i17] = ccolidx->data[i17];
  }

  emxInit_real_T(&wd, 1);
  i17 = wd->size[0];
  wd->size[0] = a_m;
  emxEnsureCapacity_real_T(wd, i17);
  i17 = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i17);
  for (i17 = 0; i17 < a_m; i17++) {
    flag->data[i17] = 0;
  }

  cnnz = -1;
  needSort = false;
  if (b_colidx->data[1] - 1 != 0) {
    if (b_colidx->data[1] - 1 == 1) {
      paend = a_colidx->data[b_rowidx->data[0]] - 1;
      i17 = a_colidx->data[b_rowidx->data[0] - 1];
      for (pa = i17; pa <= paend; pa++) {
        cnnz++;
        c->rowidx->data[cnnz] = a_rowidx->data[pa - 1];
        wd->data[a_rowidx->data[pa - 1] - 1] = a_d->data[pa - 1] * b_d->data[0];
      }
    } else {
      paend = a_colidx->data[b_rowidx->data[0]] - 1;
      i17 = a_colidx->data[b_rowidx->data[0] - 1];
      for (pa = i17; pa <= paend; pa++) {
        cnnz++;
        flag->data[a_rowidx->data[pa - 1] - 1] = cnnz + 1;
        c->rowidx->data[cnnz] = a_rowidx->data[pa - 1];
        wd->data[a_rowidx->data[pa - 1] - 1] = a_d->data[pa - 1] * b_d->data[0];
      }

      for (bcidx = 1; bcidx + 1 < b_colidx->data[1]; bcidx++) {
        bd = b_d->data[bcidx];
        paend = a_colidx->data[b_rowidx->data[bcidx]] - 1;
        i17 = a_colidx->data[b_rowidx->data[bcidx] - 1];
        for (pa = i17; pa <= paend; pa++) {
          if (flag->data[a_rowidx->data[pa - 1] - 1] < 1) {
            cnnz++;
            flag->data[a_rowidx->data[pa - 1] - 1] = cnnz + 1;
            c->rowidx->data[cnnz] = a_rowidx->data[pa - 1];
            wd->data[a_rowidx->data[pa - 1] - 1] = a_d->data[pa - 1] * bd;
            needSort = true;
          } else {
            wd->data[a_rowidx->data[pa - 1] - 1] += a_d->data[pa - 1] * bd;
          }
        }
      }
    }
  }

  emxFree_int32_T(&flag);
  bcidx = ccolidx->data[1] - 1;
  paend = ccolidx->data[0];
  if (needSort) {
    b_introsort(c->rowidx, ccolidx->data[0], ccolidx->data[1] - 1);
  }

  emxFree_int32_T(&ccolidx);
  for (pa = paend; pa <= bcidx; pa++) {
    c->d->data[pa - 1] = wd->data[c->rowidx->data[pa - 1] - 1];
  }

  emxFree_real_T(&wd);
  c_sparse_fillIn(c);
}

//
// Arguments    : const emxArray_real_T *a_d
//                const emxArray_int32_T *a_colidx
//                const emxArray_int32_T *a_rowidx
//                int a_m
//                const emxArray_real_T *b_d
//                const emxArray_int32_T *b_colidx
//                const emxArray_int32_T *b_rowidx
//                int b_n
//                coder_internal_sparse *c
// Return Type  : void
//
void sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T *a_colidx,
                   const emxArray_int32_T *a_rowidx, int a_m, const
                   emxArray_real_T *b_d, const emxArray_int32_T *b_colidx, const
                   emxArray_int32_T *b_rowidx, int b_n, coder_internal_sparse *c)
{
  emxArray_int32_T *ccolidx;
  unsigned int unnamed_idx_0;
  int i14;
  int bcidx;
  emxArray_int32_T *flag;
  int cnnz;
  int j;
  int exitg1;
  int cstart;
  int cmax;
  int pb;
  int pcstart;
  emxArray_real_T *wd;
  bool needSort;
  double bd;
  emxInit_int32_T(&ccolidx, 1);
  unnamed_idx_0 = (unsigned int)b_colidx->size[0];
  i14 = ccolidx->size[0];
  ccolidx->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity_int32_T(ccolidx, i14);
  bcidx = (int)unnamed_idx_0;
  for (i14 = 0; i14 < bcidx; i14++) {
    ccolidx->data[i14] = 0;
  }

  emxInit_int32_T(&flag, 1);
  i14 = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i14);
  for (i14 = 0; i14 < a_m; i14++) {
    flag->data[i14] = 0;
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
        pb = a_colidx->data[b_rowidx->data[bcidx - 1]] - 1;
        i14 = a_colidx->data[b_rowidx->data[bcidx - 1] - 1];
        for (pcstart = i14; pcstart <= pb; pcstart++) {
          if (flag->data[a_rowidx->data[pcstart - 1] - 1] != j + 1) {
            flag->data[a_rowidx->data[pcstart - 1] - 1] = j + 1;
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

  sparse_sparse(a_m, b_n, cnnz, c);
  i14 = c->colidx->size[0];
  c->colidx->size[0] = ccolidx->size[0];
  emxEnsureCapacity_int32_T(c->colidx, i14);
  bcidx = ccolidx->size[0];
  for (i14 = 0; i14 < bcidx; i14++) {
    c->colidx->data[i14] = ccolidx->data[i14];
  }

  emxInit_real_T(&wd, 1);
  i14 = wd->size[0];
  wd->size[0] = a_m;
  emxEnsureCapacity_real_T(wd, i14);
  i14 = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i14);
  for (i14 = 0; i14 < a_m; i14++) {
    flag->data[i14] = 0;
  }

  pb = 0;
  cnnz = -1;
  for (j = 0; j < b_n; j++) {
    cmax = 1 + j;
    needSort = false;
    pcstart = cnnz + 2;
    bcidx = (b_colidx->data[cmax] - pb) - 1;
    if (bcidx != 0) {
      if (bcidx == 1) {
        bcidx = a_colidx->data[b_rowidx->data[pb]] - 1;
        i14 = a_colidx->data[b_rowidx->data[pb] - 1];
        for (cstart = i14; cstart <= bcidx; cstart++) {
          cnnz++;
          c->rowidx->data[cnnz] = a_rowidx->data[cstart - 1];
          wd->data[a_rowidx->data[cstart - 1] - 1] = a_d->data[cstart - 1] *
            b_d->data[pb];
        }

        pb++;
      } else {
        bcidx = a_colidx->data[b_rowidx->data[pb]] - 1;
        i14 = a_colidx->data[b_rowidx->data[pb] - 1];
        for (cstart = i14; cstart <= bcidx; cstart++) {
          cnnz++;
          flag->data[a_rowidx->data[cstart - 1] - 1] = cnnz + 1;
          c->rowidx->data[cnnz] = a_rowidx->data[cstart - 1];
          wd->data[a_rowidx->data[cstart - 1] - 1] = a_d->data[cstart - 1] *
            b_d->data[pb];
        }

        for (pb++; pb + 1 < b_colidx->data[cmax]; pb++) {
          bd = b_d->data[pb];
          bcidx = a_colidx->data[b_rowidx->data[pb]] - 1;
          i14 = a_colidx->data[b_rowidx->data[pb] - 1];
          for (cstart = i14; cstart <= bcidx; cstart++) {
            if (flag->data[a_rowidx->data[cstart - 1] - 1] < pcstart) {
              cnnz++;
              flag->data[a_rowidx->data[cstart - 1] - 1] = cnnz + 1;
              c->rowidx->data[cnnz] = a_rowidx->data[cstart - 1];
              wd->data[a_rowidx->data[cstart - 1] - 1] = a_d->data[cstart - 1] *
                bd;
              needSort = true;
            } else {
              wd->data[a_rowidx->data[cstart - 1] - 1] += a_d->data[cstart - 1] *
                bd;
            }
          }
        }
      }
    }

    cstart = ccolidx->data[cmax] - 1;
    pcstart = ccolidx->data[cmax - 1];
    if (needSort) {
      b_introsort(c->rowidx, ccolidx->data[cmax - 1], ccolidx->data[cmax] - 1);
    }

    for (bcidx = pcstart; bcidx <= cstart; bcidx++) {
      c->d->data[bcidx - 1] = wd->data[c->rowidx->data[bcidx - 1] - 1];
    }
  }

  emxFree_int32_T(&flag);
  emxFree_int32_T(&ccolidx);
  emxFree_real_T(&wd);
  b_sparse_fillIn(c);
}

//
// File trailer for mtimes.cpp
//
// [EOF]
//
