//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlascl.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 28-Jan-2020 15:30:30
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ADMMGainDesign3D.h"
#include "xzlascl.h"
#include "ADMMGainDesign3D_emxutil.h"

// Function Definitions

//
// Arguments    : double cfrom
//                double cto
//                emxArray_creal_T *A
// Return Type  : void
//
void xzlascl(double cfrom, double cto, emxArray_creal_T *A)
{
  double cfromc;
  double ctoc;
  bool notdone;
  double cfrom1;
  double cto1;
  double a;
  int i57;
  int loop_ub;
  int b_loop_ub;
  int i58;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      a = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      a = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      a = ctoc / cfromc;
      notdone = false;
    }

    i57 = A->size[0] * A->size[1];
    emxEnsureCapacity_creal_T(A, i57);
    loop_ub = A->size[1];
    for (i57 = 0; i57 < loop_ub; i57++) {
      b_loop_ub = A->size[0];
      for (i58 = 0; i58 < b_loop_ub; i58++) {
        A->data[i58 + A->size[0] * i57].re *= a;
        A->data[i58 + A->size[0] * i57].im *= a;
      }
    }
  }
}

//
// File trailer for xzlascl.cpp
//
// [EOF]
//
