#include "verticaladjuster.hpp"

namespace geo {

namespace {

boost::optional<SrsFactors> create(bool apply, const SrsDefinition &srs)
{
    if (!apply) return boost::none;
    return SrsFactors(srs);
}

boost::optional<SrsFactors> create(bool apply, const SrsDefinition &srs
                                   , const SrsDefinition &srcSrs)
{
    if (!apply) return boost::none;
    return SrsFactors(srs, srcSrs);
}

}

VerticalAdjuster::VerticalAdjuster(bool apply, const SrsDefinition &srs)
    : sf_(create(apply, srs))
{
}

VerticalAdjuster::VerticalAdjuster(bool apply, const SrsDefinition &srs
                                   , const SrsDefinition &srcSrs)
    : sf_(create(apply, srs, srcSrs))
{
}

math::Point3 VerticalAdjuster::operator()(math::Point3 p) const
{
    if (sf_) {
        p(2) *= (*sf_)(p).meridionalScale;
    }
    return p;
}

} // namespace geo
