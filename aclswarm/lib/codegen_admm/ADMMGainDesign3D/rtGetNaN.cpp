//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//  File: rtGetNaN.cpp
//
//  MATLAB Coder version            : 4.3
//  C/C++ source code generated on  : 02-Feb-2020 11:20:18


//
//  Abstract:
//        MATLAB for code generation function to initialize non-finite, NaN

#include "rtGetNaN.h"

// Function: rtGetNaN ======================================================================
//  Abstract:
//  Initialize rtNaN needed by the generated code.
//  NaN is initialized as non-signaling. Assumes IEEE.

real_T rtGetNaN(void)
{
  return rtNaN;
}

// Function: rtGetNaNF =====================================================================
//  Abstract:
//  Initialize rtNaNF needed by the generated code.
//  NaN is initialized as non-signaling. Assumes IEEE.

real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

//
//  File trailer for rtGetNaN.cpp
//
//  [EOF]

