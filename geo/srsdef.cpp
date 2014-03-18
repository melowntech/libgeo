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

bool areSame(const SrsDefinition &def1, const SrsDefinition &def2
             , SrsEquivalence type)
{
    OGRSpatialReference sr1;
    detail::import(sr1, def1);

    OGRSpatialReference sr2;
    detail::import(sr2, def2);

    switch (type) {
    case SrsEquivalence::both:
        return sr1.IsSame(&sr2);
    case SrsEquivalence::geographic:
        return sr1.IsSameGeogCS(&sr2);
    case SrsEquivalence::vertical:
        return sr1.IsSameVertCS(&sr2);
    }
    return false;
}

} // namespace geo
