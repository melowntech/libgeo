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

    case SrsDefinition::Type::enu:
        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS is not supported by ORG library.";
    }
}

void import(GeographicLib::LocalCartesian &lc, const SrsDefinition &def)
{
    if (!def.is(SrsDefinition::Type::enu)) {
        LOGTHROW(err1, std::runtime_error)
            << "Only ENU SRS is not supported by GeographicLib library.";
    }

    std::istringstream is(def.srs);
    GeographicLib::Math::real lat, lon, h;

    is >> utility::expect('[')
       >> lon >> utility::expect(',') >> lat >> utility::expect(',') >> h
       >> utility::expect(']');

    auto comma(utility::match(','));
    is >> comma;
    if (!comma.matched) {
        if (!is) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing ENU definition (input = " << def.srs << ").";
        }

        if (!is.eof()) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing ENU definition (input = " << def.srs << ").";
        }

        // OK, default WGS84 speroid
        lc = GeographicLib::LocalCartesian(lat, lon, h);
        return;
    }

    // parse sphereoid
    GeographicLib::Math::real major, flattening;
    is >> utility::expect('[') >> major >> utility::expect(',')
       >> flattening >> utility::expect(']');

    if (!is) {
        LOGTHROW(err1, std::runtime_error)
            << "Error parsing ENU definition (input = " << def.srs <<").";
    }

    lc = GeographicLib::LocalCartesian
        (lat, lon, h, GeographicLib::Geocentric(major, 1.0 / flattening));
}


} } // namespace geo::detail
