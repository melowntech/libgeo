#ifndef scene_init_dem_hpp_included_
#define scene_init_dem_hpp_included_

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

namespace geo {

math::Points3 loadDem(const boost::filesystem::path &path);

math::Points3 loadDem(const boost::filesystem::path &path
                      , const math::Extents2 &extents);

} // namespace geo

#endif // scene_init_dem_hpp_included_

