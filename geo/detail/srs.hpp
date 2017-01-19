#ifndef geo_detail_srs_hpp_included_
#define geo_detail_srs_hpp_included_

#include <ogr_spatialref.h>
#include <GeographicLib/LocalCartesian.hpp>

#include "../srsdef.hpp"
#include "../enu.hpp"

namespace geo { namespace detail {

void import(OGRSpatialReference &sr, const SrsDefinition &def);

void import(Enu &enu, const SrsDefinition &def);

} } // namespace geo::detail

#endif // geo_detail_srs_hpp_included_
