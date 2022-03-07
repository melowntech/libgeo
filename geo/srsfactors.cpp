/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdexcept>

#include <proj.h>

#if PROJ_VERSION_NUMBER < 60000
#  include "detail/pjfactors-4.8.h"
#endif

#include "dbglog/dbglog.hpp"

#include "srsfactors.hpp"

namespace geo {

SrsFactors::SrsFactors(const SrsDefinition &def)
    : proj_(def), srcProj_(proj_.rev())
{}

SrsFactors::SrsFactors(const SrsDefinition &def, const SrsDefinition &src)
    : proj_(def), srcProj_(src, true)
{}

#if PROJ_VERSION_NUMBER < 60000

namespace {
int getPjFactors(double x, double y, void *pj
                 , SrsFactors::Factors &factors)
{
    projLP lp = { x, y };

    struct FACTORS fac;
    memset(&fac, 0, sizeof(fac));
    if (pj_factors(lp, static_cast<projPJ>(pj), .0, &fac)) {
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
            << proj_.error();
    }

    return f;
}

#else // proj 6+

namespace {

int getPjFactors(double x, double y, PJ *pj
                 , SrsFactors::Factors &factors)
{

    const auto fac(::proj_factors(pj, { x, y }));

    factors.meridionalScale = fac.meridional_scale;
    factors.parallelScale = fac.parallel_scale;
    factors.angularDistortion = fac.angular_distortion;
    factors.thetaPrime = fac.meridian_parallel_angle;
    factors.convergence = fac.meridian_convergence;
    factors.arealScaleFactor = fac.areal_scale;
    factors.minScaleError = fac.tissot_semimajor;
    factors.maxScaleError = fac.tissot_semiminor;

    factors.lambdaDx = fac.dx_dlam;
    factors.phiDx = fac.dx_dphi;
    factors.lambdaDy = fac.dy_dlam;
    factors.phiDy = fac.dy_dphi;

    return 0;
}

} // namespace

SrsFactors::Factors SrsFactors::operator()(const math::Point2 &p) const
{
    // obtain lat/lon from p
    auto pp(srcProj_(p, false));

    Factors f;

    auto *pj(static_cast<PJ*>(proj_.proj_.get()));

    if (getPjFactors(pp(0), pp(1), pj, f)) {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to get SRS factors for coordinates " << p << ": "
            << proj_.error();
    }

    return f;
}

#endif

} // namespace geo
