#ifndef geo_dem_hpp_included_
#define geo_dem_hpp_included_

#include <string>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

namespace geo {

struct DemCloud {
    math::Points3 pc;
    std::string projectionReference;

    operator math::Points3&() { return pc; }
    operator const math::Points3&() const { return pc; }
};

DemCloud loadDem(const boost::filesystem::path &path);

DemCloud loadDem(const boost::filesystem::path &path
                 , const math::Extents2 &extents);

} // namespace geo

#endif // geo_dem_hpp_included_
