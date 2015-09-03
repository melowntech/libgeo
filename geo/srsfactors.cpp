#include <stdexcept>

#include <proj_api.h>

#include "dbglog/dbglog.hpp"

#include "./srsfactors.hpp"

#if PJ_VERSION < 480
#  include "./detail/pjfactors-4.7.h"
#else
#  include "./detail/pjfactors-4.8.h"
#endif

namespace geo {

SrsFactors::SrsFactors(const SrsDefinition &def)
    : proj_(def), srcProj_(proj_.rev())
{}

SrsFactors::SrsFactors(const SrsDefinition &def, const SrsDefinition &src)
    : proj_(def), srcProj_(src, true)
{}

namespace {

int getPjFactors(double x, double y, void *pj
                 , SrsFactors::Factors &factors)
{
    projLP lp = { x, y };

    struct FACTORS fac;
    memset(&fac, 0, sizeof(fac));
    if (pj_factors(lp, pj, .0, &fac)) {
        return 1;
    }

    factors.meridionalScale = fac.h;
    factors.parallelScale = fac.k;
    factors.angularDistortion = fac.omega;
    factors.thetaPrime = fac.thetap;
    factors.convergence = fac.conv;
    factors.arealScaleFactor = fac.s;
    factors.minScaleError = fac.a;
    factors.maxScaleError = fac.b;

    factors.lambdaDx = fac.der.x_l;
    factors.phiDx = fac.der.x_p;
    factors.lambdaDy = fac.der.y_l;
    factors.phiDy = fac.der.y_p;

    return 0;
}

} // namespace

SrsFactors::Factors SrsFactors::operator()(const math::Point2 &p) const
{
    // obtain lat/lon from p
    auto pp(srcProj_(p, false));

    Factors f;

    if (getPjFactors(pp(0), pp(1), proj_.proj_.get(), f)) {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to get SRS factors for coordinates " << p << ": "
            << ::pj_strerrno(::pj_errno);
    }

    return f;
}

} // namespace geo
