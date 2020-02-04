#ifdef CS_COMPLEX
#if !defined(__cplusplus) && defined(_MSC_VER)
#define CS_ENTRY_CREATE(a,b) _DCOMPLEX_(a,b)
#else
#define CS_ENTRY_CREATE(a,b) CS_ENTRY(a,b)
#endif
#endif

#include "cs.h"
/* load a triplet matrix from a file */
cs* cs_load(FILE* f) {
    double i, j; /* use double for integers to avoid csi conflicts */
    double x;
#ifdef CS_COMPLEX
    double xi;
#endif
    cs* T;
    if (!f) {
        return (NULL);
    }                              /* check inputs */
    T = cs_spalloc(0, 0, 1, 1, 1); /* allocate result */
#ifdef CS_COMPLEX
    while (fscanf(f, "%lg %lg %lg %lg\n", &i, &j, &x, &xi) == 4)
#else
    while (fscanf(f, "%lg %lg %lg\n", &i, &j, &x) == 3)
#endif
    {
#ifdef CS_COMPLEX
#if  defined(__cplusplus) || defined(_MSC_VER)
        if (!cs_entry(T, (CS_INT)i, (CS_INT)j, CS_ENTRY_CREATE(x,  xi))) {
#else
        if (!cs_entry(T, (CS_INT)i, (CS_INT)j, x + xi * I)) {
#endif
#else
        if (!cs_entry(T, (CS_INT)i, (CS_INT)j, x)) {
#endif
            return (cs_spfree(T));
        }
    }
    return (T);
}
