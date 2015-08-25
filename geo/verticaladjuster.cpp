#include "verticaladjuster.hpp"

namespace geo {

math::Point3 VerticalAdjuster::operator()(math::Point3 p) const
{
    p(2) *= sf_(p).meridionalScale;
    return p;
}

} // namespace geo
