#ifndef geo_dem_hpp_included_
#define geo_dem_hpp_included_

#include <string>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"

namespace geo {

struct DemCloud {
    math::Points3 pc;
    SrsDefinition projectionReference;

    operator math::Points3&() { return pc; }
    operator const math::Points3&() const { return pc; }
};

DemCloud loadDem(const boost::filesystem::path &path
                 , boost::optional<math::Extents2> extents = boost::none
                 , boost::optional<SrsDefinition> dstSrs = boost::none);

} // namespace geo

#endif // geo_dem_hpp_included_
