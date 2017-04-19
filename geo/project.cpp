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

#include <proj_api.h>

#include "dbglog/dbglog.hpp"

#include "./project.hpp"

namespace geo {

namespace {

std::shared_ptr<void> initProj(const SrsDefinition &def)
{
    return { pj_init_plus(def.as(SrsDefinition::Type::proj4).srs.c_str())
            , [](void *ptr) { if (ptr) pj_free(ptr); } };
}

} // namespace

Projection::Projection(const SrsDefinition &def, bool inverse)
    : proj_(initProj(def)), inverse_(inverse)
{
    if (!proj_) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot initialize projection for (" << def.srs << ")";
    }

    LOG(info1) << "Projection(" << def.srs << ")";
}

namespace {
    constexpr double DEG2RAD(M_PI / 180.);
    constexpr double RAD2DEG(180. / M_PI);
}

math::Point2 Projection::operator()(const math::Point2 &p, bool deg) const
{
    if (inverse_) {
        auto res(pj_inv({ p(0), p(1) }, proj_.get()));
        if (deg) {
            return { res.u * RAD2DEG, res.v * RAD2DEG };
        }

        return { res.u, res.v };
    }

    if (deg) {
        auto res(pj_fwd({ p(0) * DEG2RAD, p(1) * DEG2RAD}, proj_.get()));
        return { res.u, res.v };
    }

    auto res(pj_fwd({ p(0), p(1)}, proj_.get()));
    return { res.u, res.v };
}

math::Point3 Projection::operator()(const math::Point3 &p, bool deg) const
{
    auto xy(operator()(math::Point2(p(0), p(1)), deg));
    return { xy(0), xy(1), p(2) };
}

} // namespace geo
