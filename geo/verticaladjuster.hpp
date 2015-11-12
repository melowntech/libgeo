#ifndef geo_verticaladjuster_hpp_included_
#define geo_verticaladjuster_hpp_included_

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "math/geometry_core.hpp"

#include "./srsfactors.hpp"

namespace geo {

class VerticalAdjuster {
public:
    VerticalAdjuster() {}
    VerticalAdjuster(const SrsDefinition &srs) : sf_(boost::in_place(srs)) {}
    VerticalAdjuster(const SrsDefinition &srs, const SrsDefinition &srcSrs)
        : sf_(boost::in_place(srs, srcSrs))
    {}

    VerticalAdjuster(bool apply, const SrsDefinition &srs);
    VerticalAdjuster(bool apply, const SrsDefinition &srs
                     , const SrsDefinition &srcSrs);

    VerticalAdjuster(const SrsFactors &factors)
        : sf_(boost::in_place(factors)) {}

    /** Apply/unapply vertical adjustment.
     */
    math::Point3 operator()(math::Point3 p, bool inverse = false) const;

private:
    boost::optional<SrsFactors> sf_;
};

} // namespace geo

#endif // geo_verticaladjuster_hpp_included_
