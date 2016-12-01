#ifndef geo_detail_srs_hpp_included_
#define geo_detail_srs_hpp_included_

#include <ogr_spatialref.h>
#include <GeographicLib/LocalCartesian.hpp>

#include "../srsdef.hpp"

namespace geo { namespace detail {

void import(OGRSpatialReference &sr, const SrsDefinition &def);

void import(GeographicLib::LocalCartesian &lc, const SrsDefinition &def);

} } // namespace geo::detail

#endif // geo_detail_srs_hpp_included_
