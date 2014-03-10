#include <stdexcept>
#include <vector>

#include <ogr_spatialref.h>
#include <cpl_conv.h>

#include "dbglog/dbglog.hpp"

#include "./srsdef.hpp"
#include "./detail/srs.hpp"

namespace geo {

namespace {

std::string wkt2Proj(const SrsDefinition &def)
{
    OGRSpatialReference sr;
    detail::import(sr, def);

    char *out(nullptr);
    auto err(sr.exportToProj4(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error converting wkt to proj definition: <"
            << err << ">.";
    }
    std::string projDef(out);
    ::CPLFree(out);

    return projDef;
}

std::string proj2Wkt(const SrsDefinition &def)
{
    OGRSpatialReference sr;
    detail::import(sr, def);

    char *out(nullptr);
    auto err(sr.exportToWkt(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error converting proj to wkt definition: <"
            << err << ">.";
    }
    std::string wktDef(out);
    ::CPLFree(out);

    return wktDef;
}

} // namespace

SrsDefinition SrsDefinition::as(Type dstType) const
{
    if (type == dstType) {
        return *this;
    }

    switch (dstType) {
    case SrsDefinition::Type::proj4:
        return { wkt2Proj(*this), dstType };

    case SrsDefinition::Type::wkt:
        return { proj2Wkt(*this), dstType };
    }

    // never reached
    return *this;
}

} // namespace geo
