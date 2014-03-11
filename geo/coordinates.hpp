#ifndef geo_coordinates_hpp_included_
#define geo_coordinates_hpp_included_

#include "math/geometry_core.hpp"
#include "math/transform.hpp"

namespace geo {

namespace ublas = boost::numeric::ublas;


inline math::Matrix4 geo2normalized(const math::Extents2& extents)
{
    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));

    double scale = std::min(
        2.0 / (extents.ur[0] - extents.ll[0]),
        2.0 / (extents.ur[1] - extents.ll[1]));

    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
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

    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;
    trafo(0, 3) = midpoint[0];
    trafo(1, 3) = midpoint[1];

    return trafo;
}

inline math::Matrix4 local2normalized(const math::Extents2& extents)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    double scale = std::min(
        2.0 / (extents.ur[0] - extents.ll[0]),
        2.0 / (extents.ur[1] - extents.ll[1]));

    trafo(0, 0) = trafo(1, 1) = trafo(2, 2) = scale;

    return trafo;
}

inline math::Matrix4 geo2local(const math::Extents2& extents)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));
    trafo(0, 3) = -midpoint[0];
    trafo(1, 3) = -midpoint[1];

    return trafo;
}

inline math::Matrix4 local2geo(const math::Extents2& extents)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    const math::Point2 midpoint(0.5 * (extents.ur + extents.ll));
    trafo(0, 3) = midpoint[0];
    trafo(1, 3) = midpoint[1];

    return trafo;
}

} // namespace vadstena

#endif // geo_geo_hpp_included_
