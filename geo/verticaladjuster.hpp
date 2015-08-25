#ifndef geo_verticaladjuster_hpp_included_
#define geo_verticaladjuster_hpp_included_

#include "math/geometry_core.hpp"

#include "./srsfactors.hpp"

namespace geo {

class VerticalAdjuster {
public:
    VerticalAdjuster(const SrsDefinition &srs) : sf_(srs) {}
    VerticalAdjuster(const SrsDefinition &srs, const SrsDefinition &srcSrs)
        : sf_(srs, srcSrs)
    {}
    VerticalAdjuster(const SrsFactors &factors) : sf_(factors) {}

    math::Point3 operator()(math::Point3 p) const;

private:
    SrsFactors sf_;
};

} // namespace geo

#endif // geo_verticaladjuster_hpp_included_
