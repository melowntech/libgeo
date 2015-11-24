#ifndef geo_srs_hpp_included_
#define geo_srs_hpp_included_

#include <ogr_spatialref.h>

#include "./srsdef.hpp"

namespace geo {

/** Returns spatial reference as OGRSpatialReference
 */
OGRSpatialReference asOgrSr(const SrsDefinition &def);

/** Merges horizontal SRS from first parameter with vertival SRS from second
 * parameter and returns result.
 *
 * Both parameters must be either projected or geographic projections.
 *
 *  \param dst SRS to be modified
 *  \param src source SRS
 */
OGRSpatialReference merge(const OGRSpatialReference &horiz
                          , const OGRSpatialReference &vert);

} // namespace geo

#endif // geo_srs_hpp_included_
