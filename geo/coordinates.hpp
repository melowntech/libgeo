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
#ifndef geo_coordinates_hpp_included_
#define geo_coordinates_hpp_included_

#include "math/geometry_core.hpp"
#include "math/transform.hpp"
#include "math/math.hpp"

namespace geo {

inline math::Matrix4 geo2normalized(const math::Extents2& extents)
{
    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));

    double scale = std::min(
        2.0 / (extents.ur[0] - extents.ll[0]),
        2.0 / (extents.ur[1] - extents.ll[1]));

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;
    trafo(0, 3) = -midpoint[0] * scale;
    trafo(1, 3) = -midpoint[1] * scale;

    return trafo;
}

inline math::Matrix4 normalized2geo(const math::Extents2& extents)
{
    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));

    double scale = std::max(
        (extents.ur[0] - extents.ll[0]) / 2.0,
        (extents.ur[1] - extents.ll[1]) / 2.0);

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;
    trafo(0, 3) = midpoint[0];
    trafo(1, 3) = midpoint[1];

    return trafo;
}

inline math::Matrix4 geo2normalized(const math::Extents3& extents)
{
    const math::Point3 midpoint(0.5 * (extents.ur + extents.ll));

    double scale = std::min( std::min(
        2.0 / (extents.ur[0] - extents.ll[0]),
        2.0 / (extents.ur[1] - extents.ll[1])),
        2.0 / (extents.ur[2] - extents.ll[2]));

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;
    trafo(0, 3) = -midpoint[0] * scale;
    trafo(1, 3) = -midpoint[1] * scale;
    trafo(2, 3) = -midpoint[2] * scale;

    return trafo;
}

inline math::Matrix4 normalized2geo(const math::Extents3& extents)
{
    const math::Point3 midpoint(0.5 * (extents.ur + extents.ll));

    double scale = std::max( std::max(
        (extents.ur[0] - extents.ll[0]) / 2.0,
        (extents.ur[1] - extents.ll[1]) / 2.0),
        (extents.ur[2] - extents.ll[2]) / 2.0);

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;
    trafo(0, 3) = midpoint[0];
    trafo(1, 3) = midpoint[1];
    trafo(2, 3) = midpoint[2];

    return trafo;
}

inline math::Matrix4 local2normalized(const math::Extents2& extents)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    double scale = std::min(
        2.0 / (extents.ur[0] - extents.ll[0]),
        2.0 / (extents.ur[1] - extents.ll[1]));

    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;

    return trafo;
}

inline math::Matrix4 geo2local(const math::Extents2& extents)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));
    trafo(0, 3) = -midpoint[0];
    trafo(1, 3) = -midpoint[1];

    return trafo;
}

inline math::Matrix4 local2geo(const math::Extents2& extents)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));
    trafo(0, 3) = midpoint[0];
    trafo(1, 3) = midpoint[1];

    return trafo;
}

inline math::Matrix4 raster2geo(const math::Extents2 &extents
                               , const math::Size2f &pxSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // scale
    trafo(0, 0) = pxSize.width;
    trafo(1, 1) = -pxSize.height;

    // translate
    trafo(0, 3) = extents.ll(0) + pxSize.width / 2.0;
    trafo(1, 3) = extents.ur(1) - pxSize.height / 2.0;

    return trafo;
}

inline math::Matrix4 geo2raster(const math::Extents2 &extents
                                , const math::Size2f &pxSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    trafo(0, 0) = 1 / pxSize.width;
    trafo(1, 1) = -1 / pxSize.height;

    trafo(0, 3) = (-extents.ll(0) - pxSize.width / 2.0) / pxSize.width;
    trafo(1, 3) = (extents.ur(1) - pxSize.height / 2.0) / pxSize.height;

    return trafo;
}

// Origin-based coordinate systems

inline math::Matrix4 geo2local(const math::Point2 &origin)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    trafo(0, 3) = -origin[0];
    trafo(1, 3) = -origin[1];

    return trafo;
}

inline math::Matrix4 local2geo(const math::Point2 &origin)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    trafo(0, 3) = origin[0];
    trafo(1, 3) = origin[1];

    return trafo;
}

} // namespace geo

#endif // geo_geo_hpp_included_
