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
#include "verticaladjuster.hpp"

namespace geo {

namespace {

boost::optional<SrsFactors> create(bool apply, const SrsDefinition &srs)
{
    if (srs.type == SrsDefinition::Type::enu) return boost::none;
    if (!apply) return boost::none;
    return SrsFactors(srs);
}

boost::optional<SrsFactors> create(bool apply, const SrsDefinition &srs
                                   , const SrsDefinition &srcSrs)
{
    if (srs.type == SrsDefinition::Type::enu) return boost::none;
    if (!apply) return boost::none;
    return SrsFactors(srs, srcSrs);
}

}

VerticalAdjuster::VerticalAdjuster(const SrsDefinition &srs, bool inverse)
    : sf_(create(true, srs)), inverse_(inverse)
{
}

VerticalAdjuster::VerticalAdjuster(const SrsDefinition &srs
                                   , const SrsDefinition &srcSrs
                                   , bool inverse)
    : sf_(create(true, srs, srcSrs)), inverse_(inverse)
{
}

VerticalAdjuster::VerticalAdjuster(bool apply, const SrsDefinition &srs
                                   , bool inverse)
    : sf_(create(apply, srs)), inverse_(inverse)
{
}

VerticalAdjuster::VerticalAdjuster(bool apply, const SrsDefinition &srs
                                   , const SrsDefinition &srcSrs
                                   , bool inverse)
    : sf_(create(apply, srs, srcSrs)), inverse_(inverse)
{
}

math::Point3 VerticalAdjuster::operator()(math::Point3 p, bool inverse) const
{
    if (sf_) {
        auto scale((*sf_)(p).meridionalScale);
        if (inverse_) { inverse = !inverse; }
        if (inverse) {
            p(2) /= scale;
        } else {
            p(2) *= scale;
        }
    }
    return p;
}

math::Point4 VerticalAdjuster::operator()(const math::Point4 &p, bool inverse)
    const
{
    const auto pp(operator()
                  (math::Point3(p(0) / p(3), p(1) / p(3), p(2) / p(3))
                   , inverse));
    return math::Point4(pp(0), pp(1), pp(2), 1.0);
}

} // namespace geo
