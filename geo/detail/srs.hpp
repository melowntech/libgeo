#ifndef geo_detail_srs_hpp_included_
#define geo_detail_srs_hpp_included_

#include <string>
#include <vector>

#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"

#include "../srsdef.hpp"

namespace geo { namespace detail {

inline void import(OGRSpatialReference &sr, const SrsDefinition &def)
{
    switch (def.type) {
    case SrsDefinition::Type::proj4: {
        auto err(sr.importFromProj4(def.srs.c_str()));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing proj definition: <" << err << ">.";
        }
        break;
    }

    case SrsDefinition::Type::wkt: {
        // convert NUL-terminated string to char vector
        std::vector<char> tmp(def.srs.c_str()
                              , def.srs.c_str() + def.srs.size() + 1);
        char *data(tmp.data());
        auto err(sr.importFromWkt(&data));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing wkt definition: <" << err << ">.";
        }
        break;
    }
    }
}


} } // namespace geo::detail

#endif // geo_detail_srs_hpp_included_
