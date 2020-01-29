#ifdef CS_COMPLEX
#undef CS_COMPLEX
#endif
#include "cs.h"
static CS_INT cs_nonzero(CS_INT i, CS_INT j, CS_ENTRY aij, void* other) {
    (void)i;
    (void)j;
    (void)other;
    return cs_equalcc(aij, cs_createFromDouble(0)) ? 0 : 1;
}
CS_INT cs_dropzeros(cs* A) {
    return (cs_fkeep(A, &cs_nonzero, NULL)); /* keep all nonzero entries */
}
