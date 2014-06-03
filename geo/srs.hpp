#ifndef geo_srs_hpp_included_
#define geo_srs_hpp_included_

#include <ogr_spatialref.h>

#include "./srsdef.hpp"

namespace geo {

/** Returns spatial reference as OGRSpatialReference
 */
OGRSpatialReference asOgrSrs(const SrsDefinition &def);

} // namespace geo

#endif // geo_srs_hpp_included_
