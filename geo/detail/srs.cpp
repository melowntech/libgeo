#include <string>
#include <vector>

#include "boost/lexical_cast.hpp"

#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"

#include "./srs.hpp"

namespace geo { namespace detail {

void import(OGRSpatialReference &sr, const SrsDefinition &def)
{
    switch (def.type) {
    case SrsDefinition::Type::proj4: {
        auto err(sr.importFromProj4(def.srs.c_str()));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing proj definition: <" << err << "> (input = "
                << def.srs << ").";
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
                << "Error parsing wkt definition: <" << err << "> (input = "
                << def.srs << ").";
        }
        break;
    }

    case SrsDefinition::Type::epsg: {
        try {
            auto err(sr.importFromEPSG(boost::lexical_cast<int>(def.srs)));
            if (err != OGRERR_NONE) {
                LOGTHROW(err1, std::runtime_error)
                    << "Error parsing EPSG definition: <"
                    << err << "> (input = "
                    << def.srs << ").";
            }
        } catch (const boost::bad_lexical_cast &e) {
                LOGTHROW(err1, std::runtime_error)
                    << "Error parsing EPSG definition: not a number (input = "
                    << def.srs << ").";
        }
        break;
    }
    }
}


} } // namespace geo::detail
