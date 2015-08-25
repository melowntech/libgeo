#include <stdexcept>
#include <vector>

#include <boost/lexical_cast.hpp>

#include <ogr_spatialref.h>
#include <cpl_conv.h>
#include <sstream>

#include "dbglog/dbglog.hpp"

#include "./srsdef.hpp"
#include "./srs.hpp"
#include "./detail/srs.hpp"

namespace geo {

namespace {

std::string def2Proj(const SrsDefinition &def)
{
    OGRSpatialReference sr;
    detail::import(sr, def);

    char *out(nullptr);
    auto err(sr.exportToProj4(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error converting " << def.type << " to proj definition: <"
            << err << ">.";
    }
    std::string projDef(out);
    ::CPLFree(out);

    return projDef;
}

std::string def2Wkt(const SrsDefinition &def)
{
    OGRSpatialReference sr;
    detail::import(sr, def);

    char *out(nullptr);
    auto err(sr.exportToWkt(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error converting " << def.type << " to wkt definition: <"
            << err << ">.";
    }
    std::string wktDef(out);
    ::CPLFree(out);

    return wktDef;
}

} // namespace

SrsDefinition::SrsDefinition(int epsg)
    : srs(boost::lexical_cast<std::string>(epsg))
    , type(Type::epsg)
{}

SrsDefinition SrsDefinition::longlat() {
    
    return SrsDefinition(
        "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs", Type::proj4 );
}

SrsDefinition SrsDefinition::utm(uint zone, bool isNorth) {
    
    std::ostringstream ostr;
    
    ostr 
        << "+proj=utm +zone=" << zone << " " 
        << ( isNorth ? "+north " : "+south " ) 
        << "+ellps=WGS84 +datum=WGS84 +no_defs";
    
    LOG(debug) <<  ostr.str();   
        
    return SrsDefinition( ostr.str(), Type::proj4 );
}

SrsDefinition SrsDefinition::utmFromLonglat(const math::Point2 & longlat ) {
    
    return utm( 
        ::floor( (longlat[0] + 180) / 6) + 1,
        longlat[1] >= 0.0 ); 
}


SrsDefinition SrsDefinition::as(Type dstType) const
{
    if (type == dstType) {
        return *this;
    }

    switch (dstType) {
    case SrsDefinition::Type::proj4:
        return { def2Proj(*this), dstType };

    case SrsDefinition::Type::wkt:
        return { def2Wkt(*this), dstType };

    case SrsDefinition::Type::epsg:
        LOGTHROW(err1, std::runtime_error)
            << "EPSG reference cannot be constructed from "
            << type << " representation.";
    }

    // never reached
    return *this;
}

::OGRSpatialReference SrsDefinition::reference() const
{
    ::OGRSpatialReference sr;
    detail::import(sr, *this);
    return sr;
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

OGRSpatialReference asOgrSr(const SrsDefinition &def)
{
    OGRSpatialReference sr;
    detail::import(sr, def);
    return sr;
}

} // namespace geo
