#ifdef CS_COMPLEX
#undef CS_COMPLEX
#endif
#include "cs.h"
/* apply the ith Householder vector to x */
CS_INT cs_happly(const cs* V, CS_INT i, double beta, CS_ENTRY* x) {
    CS_INT p, *Vp, *Vi;
    CS_ENTRY *Vx, tau = cs_createFromDouble(0);
    if (!CS_CSC(V) || !x) {
        return (0);
    } /* check inputs */
    Vp = V->p;
    Vi = V->i;
    Vx = V->x;
    for (p = Vp[i]; p < Vp[i + 1]; p++) { /* tau = v'*x */
        tau = cs_pluscc(tau, cs_mulcc(CS_CONJ(Vx[p]), x[Vi[p]]));
    }
    tau = cs_mulcc(tau, cs_createFromDouble(beta));                          /* tau = beta*(v'*x) */
    for (p = Vp[i]; p < Vp[i + 1]; p++) { /* x = x - v*tau */
        x[Vi[p]] = cs_subcc(x[Vi[p]], cs_mulcc(Vx[p], tau));
    }
    return (1);
}
