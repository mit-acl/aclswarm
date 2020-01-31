#include "cs.h"
/* create a Householder reflection [v,beta,s]=house(x), overwrite x with v,
 * where (I-beta*v*v')*x = s*e1 and e1 = [1 0 ... 0]'.
 * Note that this CXSparse version is different than CSparse.  See Higham,
 * Accuracy & Stability of Num Algorithms, 2nd ed, 2002, page 357. */
CS_ENTRY cs_house(CS_ENTRY* x, double* beta, CS_INT n) {
    CS_ENTRY s = cs_createFromDouble(0);
    CS_INT i;
    if (!x || !beta) {
        return cs_createFromDouble(-1);
    } /* check inputs */
    /* s = norm(x) */
    for (i = 0; i < n; i++) {
        s = cs_pluscc(s, cs_mulcc(x[i], CS_CONJ(x[i])));
    }
    s = cs_sqrt(s);
    if (cs_equalcc(s, cs_createFromDouble(0))) {
        (*beta) = 0;
        x[0] = cs_createFromDouble(1);
    } else {
        /* s = sign(x[0]) * norm (x) ; */
        if (cs_equalcc(x[0], cs_createFromDouble(0)) == 0) {
            s = cs_mulcc(s, cs_divcc(x[0], cs_createFromDouble(CS_ABS(x[0]))));
        }
        x[0] = cs_pluscc(x[0], s);
        (*beta) = 1. / CS_REAL(cs_mulcc(CS_CONJ(s), x[0]));
    }
    return cs_flipSign(s);
}
