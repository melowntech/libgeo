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

#include <GeographicLib/Geoid.hpp>

#include "dbglog/dbglog.hpp"

#include "heightconvertor.hpp"
#include "detail/srs.hpp"

namespace geo {

namespace {

std::shared_ptr<void> initGeoid(const boost::filesystem::path &geoidDir)
{
    return std::make_shared<GeographicLib::Geoid>
        ("egm96-5", geoidDir.string());
}

} // namespace

HeightConvertor::HeightConvertor(const SrsDefinition &srs
                                 , const boost::filesystem::path &geoidDir
                                 , Direction direction)
    : proj_(srs, true), geoid_(initGeoid(geoidDir))
    , toGeoid_(direction == Direction::toGeoid)
{
    LOG(info1) << "Geoid initialized";
}

math::Point3 HeightConvertor::operator()(const math::Point3 &p) const
{
    auto latlon(proj_(p));
    return {p(0), p(1)
            , std::static_pointer_cast<GeographicLib::Geoid>(geoid_)
            ->ConvertHeight
            (latlon(1), latlon(0), p(2)
             , (toGeoid_ ? GeographicLib::Geoid::ELLIPSOIDTOGEOID
                : GeographicLib::Geoid::GEOIDTOELLIPSOID)) };
}

} // namespace geo
