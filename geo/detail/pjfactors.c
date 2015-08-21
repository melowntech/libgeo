#include <stdio.h>
#include <string.h>

#include <projects.h>

#include "./pjfactors.h"

int geo_detail_pj_factors(double x, double y, void *pj
                          , struct geo_detail_pj_factors *factors)
{
    LP lp = { x, y };

    struct FACTORS fac;
    memset(&fac, 0, sizeof(fac));
    if (pj_factors(lp, pj, .0, &fac)) {
        return 1;
    }

    printf("%f, %f\n", fac.h, fac.k);

    factors->meridionalScale = fac.h;
    factors->parallelScale = fac.k;
    factors->angularDistortion = fac.omega;
    factors->thetaPrime = fac.thetap;
    factors->convergence = fac.conv;
    factors->arealScaleFactor = fac.s;
    factors->minScaleError = fac.a;
    factors->maxScaleError = fac.b;

    factors->lambdaDx = fac.der.x_l;
    factors->phiDx = fac.der.x_p;
    factors->lambdaDy = fac.der.y_l;
    factors->phiDy = fac.der.y_p;

    return 0;
}
