#include "cs.h"
/* sparse Cholesky update/downdate, L*L' + sigma*w*w' (sigma = +1 or -1) */
CS_INT cs_updown(cs* L, CS_INT sigma, const cs* C, const CS_INT* parent) {
    CS_INT n, p, f, j, *Lp, *Li, *Cp, *Ci;
    CS_ENTRY *Lx, *Cx, alpha, gamma, w1, w2, *w, tmp;
    double beta = 1, beta2 = 1, delta;
#ifdef CS_COMPLEX
    CS_ENTRY phase;
#endif
    if (!CS_CSC(L) || !CS_CSC(C) || !parent) {
        return (0);
    } /* check inputs */
    Lp = L->p;
    Li = L->i;
    Lx = L->x;
    n = L->n;
    Cp = C->p;
    Ci = C->i;
    Cx = C->x;
    if ((p = Cp[0]) >= Cp[1]) {
        return (1);
    }                                   /* return if C empty */
    w = (CS_ENTRY*)cs_malloc(n, sizeof(CS_ENTRY)); /* get workspace */
    if (!w) {
        return (0);
    } /* out of memory */
    f = Ci[p];
    for (; p < Cp[1]; p++) {
        f = CS_MIN(f, Ci[p]);
    } /* f = min (find (C)) */
    for (j = f; j != -1; j = parent[j]) {
        w[j] = cs_createFromDouble(0);
    } /* clear workspace w */
    for (p = Cp[0]; p < Cp[1]; p++) {
        w[Ci[p]] = Cx[p];
    }                                     /* w = C */
    for (j = f; j != -1; j = parent[j]) { /* walk path f up to root */
        p = Lp[j];
        alpha = cs_divcc(w[j], Lx[p]); /* alpha = w(j) / L(j,j) */
        beta2 = beta * beta + cs_muldc(sigma, cs_mulcc(alpha, CS_CONJ(alpha)));
        if (beta2 <= 0) {
            break;
        } /* not positive definite */
        beta2 = sqrt(beta2);
        delta = (sigma > 0) ? (beta / beta2) : (beta2 / beta);
        gamma = cs_divcc(cs_mulcc(cs_createFromDouble(sigma), CS_CONJ(alpha)), cs_createFromDouble(beta2 * beta));
        tmp = ((sigma > 0) ? (cs_mulcc(gamma, w[j])) : cs_createFromDouble(0));
        Lx[p] = cs_pluscc(cs_mulcc(cs_createFromDouble(delta), Lx[p]), tmp);
        beta = beta2;
#ifdef CS_COMPLEX
        phase = cs_divcc(cs_createFromDouble(CS_ABS(Lx[p])), Lx[p]); /* phase = abs(L(j,j))/L(j,j)*/
        Lx[p] = cs_mulcc(Lx[p], phase);                /* L(j,j) = L(j,j) * phase */
#endif
        for (p++; p < Lp[j + 1]; p++) {
            w1 = w[Li[p]];
            w[Li[p]] = w2 = cs_subcc(w1, cs_mulcc(alpha, Lx[p]));
            Lx[p] = cs_pluscc(cs_mulcc(cs_createFromDouble(delta), Lx[p]), cs_mulcc(gamma, ((sigma > 0) ? w1 : w2)));
#ifdef CS_COMPLEX
            Lx[p] = cs_mulcc(Lx[p], phase); /* L(i,j) = L(i,j) * phase */
#endif
        }
    }
    cs_free(w);
    return (beta2 > 0);
}
