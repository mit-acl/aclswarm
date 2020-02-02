#ifdef CS_COMPLEX
#undef CS_COMPLEX
#endif
#include "cs.h"
#include <math.h>
/**
 * Microsoft Visual Stuido does not support standard C99 complex.
 * But it has limited complex support(_DComplex).
 * Add several wrapper functions for operators that Visual Studio complex does not support.
 **/

/**
 * Divide two CS_ENTRYs
 */ 
CS_ENTRY cs_divcc(CS_ENTRY lhs, CS_ENTRY rhs) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    double r,i;
    if (fabs(creal(rhs)) > fabs(cimag(rhs))) {
    double bim = cimag(rhs) / creal(rhs);
    double d = creal(rhs) + bim * cimag(rhs);
    r = (creal(lhs) + bim * cimag(lhs)) / d;
    i = (cimag(lhs) - bim * creal(lhs)) / d;
    } else {
    double bim = creal(rhs) / cimag(rhs);
    double d = cimag(rhs) + bim * creal(rhs);
    r = (bim * creal(lhs) + cimag(lhs)) / d;
    i = (bim * cimag(lhs) - creal(lhs)) / d;
    }
    return _DCOMPLEX_(r,i);
#else
    return lhs / rhs;
#endif
}

/**
 * Multiply two CS_ENTRYs
 */ 
CS_ENTRY cs_mulcc(CS_ENTRY lhs, CS_ENTRY rhs) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    double r = creal(lhs)*creal(rhs) - cimag(lhs)*cimag(rhs);
    double i = cimag(lhs)*creal(rhs) + creal(lhs)*cimag(rhs);
    return _DCOMPLEX_(r, i);
#else
    return lhs * rhs;
#endif
}

/**
 * Subtract two CS_ENTRYs
 */ 
CS_ENTRY cs_subcc(CS_ENTRY lhs, CS_ENTRY rhs) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    return _DCOMPLEX_(creal(lhs) - creal(rhs), cimag(lhs) - cimag(rhs));
#else
    return lhs - rhs;
#endif
}

/**
 * Add two CS_ENTRYs
 */ 
CS_ENTRY cs_pluscc(CS_ENTRY lhs, CS_ENTRY rhs) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    return _DCOMPLEX_(creal(lhs) + creal(rhs), cimag(lhs) + cimag(rhs));
#else
    return lhs + rhs;
#endif
}

/**
 * Create a CS_ENTRY form double
 */ 
CS_ENTRY cs_createFromDouble(double aVal) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    return _DCOMPLEX_(aVal, 0);
#else
    return aVal;
#endif
}

/**
 * Determine two CS_ETNRYs are equal
 */ 
int cs_equalcc(CS_ENTRY lhs, CS_ENTRY rhs) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    return ((creal(lhs) == creal(rhs)) && (cimag(lhs) == cimag(rhs))) ?  1 : 0;
#else
    return (lhs == rhs) ? 1 : 0;
#endif
}

/**
 * sqrt of CS_ENTRY
 * This function by design is doing real only sqrt.
 * If need to do complex, use csqrt.
 */ 
CS_ENTRY cs_sqrt(CS_ENTRY in) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    return _DCOMPLEX_(sqrt(creal(in)), 0);
#else
    return sqrt(in);
#endif
}

/**
 * Change sign of CS_ENTRY
 */ 
CS_ENTRY cs_flipSign(CS_ENTRY in) {
#if !defined(__cplusplus) && defined(_MSC_VER) && defined(CS_COMPLEX)
    return _DCOMPLEX_(-creal(in), -cimag(in));
#else
    return -in;
#endif
}

/**
 * Multiply a double with CS_ENTRY and results a double
 */ 
double cs_muldc(double lhs, CS_ENTRY rhs) {
    return lhs * CS_REAL(rhs);
}