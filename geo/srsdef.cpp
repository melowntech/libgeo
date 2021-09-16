/**
 * Copyright (c) 2017-2021 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdexcept>
#include <vector>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem/path.hpp>

#include <ogr_spatialref.h>
#include <cpl_conv.h>

#include <GeographicLib/Geocentric.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"

#include "srsdef.hpp"
#include "srs.hpp"
#include "enu.hpp"
#include "detail/srs.hpp"
#include "csconvertor.hpp"

namespace geo {

namespace {

#if GDAL_VERSION_NUM >= 3000000
#  define GEO_TRADITIONAL_GIS_ORDER(sr) \
    sr.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER)
#else
#  define GEO_TRADITIONAL_GIS_ORDER(sr)
#endif

inline std::string srs2Wkt(const OGRSpatialReference &sr)
{
    char *out(nullptr);
    auto err(sr.exportToWkt(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error exporting SRS into wkt definition: <"
            << err << ">.";
    }
    std::string wktDef(out);
    ::CPLFree(out);

    return wktDef;
}

inline std::string srs2Proj(const OGRSpatialReference &sr)
{
    char *out(nullptr);
    auto err(sr.exportToProj4(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error exporting SRS into proj definition: <"
            << err << ">.";
    }
    std::string projDef(out);
    ::CPLFree(out);

    return projDef;
}

inline std::string def2Proj(const SrsDefinition &def)
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

inline std::string def2Wkt(const SrsDefinition &def)
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

SrsDefinition::SrsDefinition(int epsg1, int epsg2)
    : srs(str(boost::format("%s+%s") % epsg1 % epsg2))
    , type(Type::epsg)
{}

SrsDefinition SrsDefinition::longlat() {

    return SrsDefinition(
        "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs", Type::proj4 );
}

SrsDefinition SrsDefinition::utm(unsigned int zone, bool isNorth) {

    std::ostringstream ostr;

    ostr
        << "+proj=utm +zone=" << zone << " "
        << ( isNorth ? "+north " : "+south " )
        << "+ellps=WGS84 +datum=WGS84 +no_defs";

    LOG(debug) <<  ostr.str();

    return SrsDefinition( ostr.str(), Type::proj4 );
}

SrsDefinition SrsDefinition::utmFromLonglat(const math::Point2 & longlat ) {

    return utm((unsigned int)(std::floor((longlat[0] + 180) / 6) + 1)
               , longlat[1] >= 0.0);
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
        break;

    case SrsDefinition::Type::enu:
        LOGTHROW(err1, std::runtime_error)
            << "ENU reference cannot be constructed from "
            << type << " representation.";
        break;
    }

    // never reached
    return *this;
}

bool SrsDefinition::convertibleTo(Type dstType) const {
    if (type == dstType) { return true; }
    switch (dstType) {
    case SrsDefinition::Type::proj4:
        return (type != SrsDefinition::Type::enu);

    case SrsDefinition::Type::wkt:
        return (type != SrsDefinition::Type::enu);

    case SrsDefinition::Type::epsg:
        return false;

    case SrsDefinition::Type::enu:
        return false;
    }

    // never reached
    return false;
}

SrsDefinition SrsDefinition::fromReference(const OGRSpatialReference &src
                                           , Type type)
{
    switch (type) {
    case SrsDefinition::Type::proj4: return { srs2Proj(src), type };
    case SrsDefinition::Type::wkt: return { srs2Wkt(src), type };

    case SrsDefinition::Type::epsg:
        LOGTHROW(err1, std::runtime_error)
            << "OGRSpatialReference cannot be exported into EPSG "
            "representation.";
        throw;

    case SrsDefinition::Type::enu:
        LOGTHROW(err1, std::runtime_error)
            << "OGRSpatialReference cannot be exported into ENU "
            "representation.";
        throw;
    }
    throw;
}

SrsDefinition SrsDefinition::fromEnu(const Enu &src)
{
    SrsDefinition dst;
    dst.type = Type::enu;
    dst.srs = boost::lexical_cast<std::string>(src);
    return dst;
}

::OGRSpatialReference SrsDefinition::reference() const
{
    ::OGRSpatialReference sr;
    GEO_TRADITIONAL_GIS_ORDER(sr);
    detail::import(sr, *this);

    return sr;
}

Enu SrsDefinition::enu() const
{
    Enu enu;
    detail::import(enu, *this);
    return enu;
}

namespace {

bool wgs84Datum(const ::OGRSpatialReference &ref)
{
    return (!strcmp(ref.GetAuthorityName("SPHEROID"), "EPSG")
            && !strcmp(ref.GetAuthorityCode("SPHEROID"), "7030"));
}

void outputSpheroid(std::ostream &os, const ::OGRSpatialReference &ref)
{
    if (wgs84Datum(ref)) {
        os << " +datum=WGS84";
    } else {
        os << " +a=" << ref.GetSemiMajor()
           << " +b=" << ref.GetSemiMinor();
    }
}

using ToWgs84 = std::array<double, 7>;

boost::optional<ToWgs84> getToWgs84(const ::OGRSpatialReference &ref)
{
    ToWgs84 towgs84;
    ref.GetTOWGS84(towgs84.data(), towgs84.size());

    if (std::size_t(std::count(towgs84.begin(), towgs84.end(), 0.0)) == towgs84.size()) {
        // all zeroes -> no params needed
        return boost::none;
    }

    return towgs84;
}

void outputToWgs84(std::ostream &os, const ToWgs84 &towgs84)
{
    os << " +towgs84=" << towgs84[0] << "," << towgs84[1]
       << "," << towgs84[2] << "," << towgs84[3] << "," << towgs84[4]
       << "," << towgs84[5] << "," << towgs84[6];
}

void outputToWgs84(std::ostream &os, const ::OGRSpatialReference &ref)
{
    if (const auto towgs84 = getToWgs84(ref)) {
        outputToWgs84(os, *towgs84);
    }
}

} // namespace

SrsDefinition geographic(const SrsDefinition &srs)
{
    ::OGRSpatialReference ref(srs.reference());

    if (ref.IsGeographic()) { return srs; }

    ::OGRSpatialReference ret;
    if (ret.CopyGeogCSFrom(&ref) == OGRERR_NONE) {
        // we can get geographic system from provided srs
        return SrsDefinition::fromReference(ret);
    }

    if (!ref.IsGeocentric()) {
        LOGTHROW(err2, std::runtime_error)
            << "Unsupported SRS type.";
    }

    std::ostringstream os;
    os << std::setprecision(12);
    os << "+proj=longlat +units=m +no_defs";

    outputSpheroid(os, ref);
    outputToWgs84(os, ref);

    return { os.str() };
}

SrsDefinition geocentric(const SrsDefinition &srs)
{
    ::OGRSpatialReference ref(srs.reference());

    if (ref.IsGeocentric()) { return srs; }

    std::ostringstream os;
    os << std::setprecision(12);
    os << "+proj=geocent +units=m +no_defs";

    outputSpheroid(os, ref);
    outputToWgs84(os, ref);

    return { os.str() };
}

SrsDefinition tmerc(const SrsDefinition &refSrs
                    , const math::Point2d &origin
                    , double scaleFactor, const math::Point2 &falseOrigin
                    , const boost::optional<std::string> &geoid)
{
    auto gcs(geographic(refSrs));

    const auto gcsOrigin(CsConvertor(refSrs, gcs)(origin));

    const auto gcsRef(gcs.reference());

    // construct tmerc
    ::OGRSpatialReference tm;
    if (OGRERR_NONE != tm.CopyGeogCSFrom(&gcsRef)) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot copy GeoCS from SRS.";
    }

    if (OGRERR_NONE != tm.SetTM(gcsOrigin(1), gcsOrigin(0)
                                , scaleFactor
                                , falseOrigin(0)
                                , falseOrigin(1)))
    {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot set tmerc.";
    }

    if (geoid) { return SrsDefinition::fromReference(setGeoid(tm, *geoid)); }
    return SrsDefinition::fromReference(tm);
}

math::Point3 ellipsoid(const SrsDefinition &srs)
{
    auto ref(srs.reference());
    const auto major = ref.GetSemiMajor();
    const auto minor = ref.GetSemiMinor();
    return math::Point3(major, major, minor);
}

bool areSame(const SrsDefinition &def1, const SrsDefinition &def2
             , SrsEquivalence type)
{
    if (def1.is(SrsDefinition::Type::enu)
        && def2.is(SrsDefinition::Type::enu))
    {
        return (def1.srs == def2.srs);
    }

    if (def1.is(SrsDefinition::Type::enu)
        || def2.is(SrsDefinition::Type::enu))
    {
        // ENU vs anything else
        return false;
    }

    auto sr1(def1.reference());
    auto sr2(def2.reference());

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
    GEO_TRADITIONAL_GIS_ORDER(sr);
    detail::import(sr, def);
    return sr;
}

OGRSpatialReference merge(const OGRSpatialReference &horiz
                          , const OGRSpatialReference &vert)
{
    if (!(horiz.IsProjected() || horiz.IsGeographic())) {
        LOGTHROW(err1, std::runtime_error)
            << "SRS Merge: 'horizontal' SRS is not projected nor "
            "geographic coordinate system";
    }

    if (!(vert.IsProjected()
          || vert.IsGeographic()
           || vert.IsVertical()))
    {
        LOGTHROW(err1, std::runtime_error)
            << "SRS Merge: 'vertical' SRS is not projected nor "
            "geographic coordinate system";
    }

    OGRSpatialReference h(horiz);
    h.StripVertical();

    if (!vert.IsVertical()) {
        // vert SRS has no vertical component -> copy only horizontal part
        return h;
    }

    // OK, there is vertical part in the vert SRS

    // find VERT_CS node
    OGRSpatialReference v;
    v.SetRoot(vert.GetRoot()->GetNode("VERT_CS")->Clone());

    auto name(str(boost::format("%s + %s Vertical Datum")
                  % h.GetRoot()->GetChild(0)->GetValue()
                  % v.GetRoot()->GetChild(0)->GetValue()));

    OGRSpatialReference out;
    GEO_TRADITIONAL_GIS_ORDER(out);
    out.SetCompoundCS(name.c_str(), &h, &v);

    // done
    return out;
}

SrsDefinition merge(const SrsDefinition &horiz, const SrsDefinition &vert)
{
    return SrsDefinition::fromReference
        (merge(horiz.reference(), vert.reference()));
}

::OGRSpatialReference setGeoid(const ::OGRSpatialReference &srs
                               , const std::string &geoid)
{
    if (!(srs.IsProjected() || srs.IsGeographic())) {
        LOGTHROW(err1, std::runtime_error)
            << "SRS set geoid: SRS is neither projected nor "
            "geographic coordinate system";
    }

    std::string wkt("VERT_CS[\"geoid height\","
                    "VERT_DATUM[\"geoid\",2005,EXTENSION[\"PROJ4_GRIDS\""
                    ",\"" + geoid + "\"]],UNIT[\"metre\",1]]");
    std::vector<char> tmp(wkt.c_str(), wkt.c_str() + wkt.size() + 1);
    ::OGRSpatialReference vert;
#if GDAL_VERSION_NUM >= 2030000
    auto err(vert.importFromWkt(tmp.data()));
#else
    char *data(tmp.data());
    auto err(vert.importFromWkt(&data));
#endif

    if (err != OGRERR_NONE) {
        LOGTHROW(err1, std::runtime_error)
            << "Error parsing wkt definition: <" << err << "> (input = "
            << wkt << ").";
    }

    ::OGRSpatialReference out;
    GEO_TRADITIONAL_GIS_ORDER(out);
    out.SetCompoundCS("", &srs, &vert);
    return out;
}

SrsDefinition setGeoid(const SrsDefinition &srs, const std::string &geoid)
{
    return SrsDefinition::fromReference(setGeoid(srs.reference(), geoid));
}

SrsDefinition setGeoid(const SrsDefinition &srs
                       , const boost::optional<std::string> &geoidGrid)
{
    if (geoidGrid) { return setGeoid(srs, *geoidGrid); }
    return srs;
}

SrsDefinition SrsDefinition::fromString(std::string value)
{
    namespace ba = boost::algorithm;
    ba::trim(value);
    if (value.empty()) { return {}; }

    if (value.front() == '+') {
        // proj string
        return SrsDefinition(value, SrsDefinition::Type::proj4);
    } else if (ba::istarts_with(value, "epsg:")) {
        // epsg
        return SrsDefinition(value.substr(5), SrsDefinition::Type::epsg);
    } else if (ba::istarts_with(value, "enu")) {
        // enu
        return SrsDefinition(value, SrsDefinition::Type::enu);
    }

    return SrsDefinition(value, SrsDefinition::Type::wkt);
}

std::string SrsDefinition::toString() const
{
    return boost::lexical_cast<std::string>(*this);
}

bool isProjected(const SrsDefinition &srs)
{
    if (srs.type == SrsDefinition::Type::enu) { return false; }
    return srs.reference().IsProjected();
}

bool isGeographic(const SrsDefinition &srs)
{
    if (srs.type == SrsDefinition::Type::enu) { return false; }
    return srs.reference().IsGeographic();
}

boost::optional<Periodicity> isPeriodic(const SrsDefinition &srs)
{
    if (srs.type == SrsDefinition::Type::enu) { return boost::none; }

    const auto ref(srs.reference());

    // geographic? OK
    if (ref.IsGeographic()) {
        // simple
        return Periodicity(Periodicity::Type::x, -180.0, +180.0);
    }

    if (!ref.IsProjected()) { return boost::none; }

    // OK, projected

    // extract projection, a bit tricky since it is simpler to get from proj
    // string

    const auto proj(srs.as(SrsDefinition::Type::proj4));

    const auto contains([&proj](const char *what)
    {
        return proj.srs.find(what) != std::string::npos;
    });

    const auto xCylinder([&]() -> boost::optional<Periodicity>
    {
        const auto ref(srs.reference());

        // valid values span from -pi*semi-major-axis to +pi*semi-major-axis but
        // we have to offset this range by false easting
        const auto falseEasting(ref.GetProjParm(SRS_PP_FALSE_EASTING));

        const auto limit(M_PI * ref.GetSemiMajor());
        return Periodicity(Periodicity::Type::x, -limit - falseEasting
                           , +limit - falseEasting);
    });

    // TODO: check for proper northing and some possible deviation from
    // "perfect" world
    if (contains("+proj=eqc")) { return xCylinder();}
    if (contains("+proj=merc")) { return xCylinder(); }

    return boost::none;
}

SrsDefinition setAngularUnit(const SrsDefinition &srs, AngularUnit unit)
{
    auto ref(srs.reference());

    switch (unit) {
    case AngularUnit::radian:
        ref.SetAngularUnits(SRS_UA_RADIAN, 1.0);
        break;
    case AngularUnit::degree:
        ref.SetAngularUnits(SRS_UA_DEGREE, 0.0174532925199433);
        break;
    }

    return SrsDefinition::fromReference(ref, SrsDefinition::Type::wkt);
}

double linearUnit(const SrsDefinition &srs, bool convertAngluar)
{
    auto ref(srs.reference());

    if (!ref.IsGeographic() || !convertAngluar) {
        return ref.GetLinearUnits(nullptr);
    }

    // geographic and we have to convert angular unit to linear

    // angular unit in radians times semi-major axis
    return ref.GetAngularUnits(nullptr) * ref.GetSemiMajor();
}

SrsDefinition asc2gtx(const SrsDefinition &srs)
{
    namespace fs = boost::filesystem;
    namespace ba = boost::algorithm;
    using namespace std::string_literals;

    struct Process {
        Process(OGR_SRSNode *node) { run(node);  }

        void run(OGR_SRSNode *node) {
            if (!node) { return; }
            using namespace std::string_literals;

            const auto *value(node->GetValue());
            if (value == "VERT_DATUM"s) {
                vertDatum(node);
                return;
            }

            for (int i(0), e(node->GetChildCount()); i != e; ++i) {
                auto child(node->GetChild(i));
                run(child);
            }
        }

        void vertDatum(OGR_SRSNode *node) {
            for (int i(0), e(node->GetChildCount()); i != e; ++i) {
                auto child(node->GetChild(i));
                const std::string value(child->GetValue());
                if (ba::iends_with(value, ".asc")
                    || ba::iends_with(value, ".bin"))
                {
                    const auto grid
                        (fs::path(value).replace_extension(".gtx")
                         .filename().string());

                    child->SetValue("EXTENSION");
                    child->ClearChildren();

                    child->AddChild(OGR_SRSNode("PROJ4_GRIDS").Clone());
                    child->AddChild(OGR_SRSNode(grid.c_str()).Clone());
                }
            }
        }
    };

    auto ref(srs.reference());
    Process(ref.GetRoot());

    return geo::SrsDefinition::fromReference
        (ref, geo::SrsDefinition::Type::wkt);
}

} // namespace geo
