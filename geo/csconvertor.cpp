#include <stdexcept>

#include <ogr_spatialref.h>
#include <cpl_error.h>

#include "dbglog/dbglog.hpp"

#include "./csconvertor.hpp"
#include "./detail/srs.hpp"

namespace geo {

namespace {

namespace ublas = boost::numeric::ublas;

typedef boost::optional<std::string> OptName;

std::string asName(const OGRSpatialReference &ref
                   , const OptName &name = boost::none)
{
    if (name) { return *name; }
    return SrsDefinition::fromReference(ref).srs;
}

std::shared_ptr<void>
initTransImpl(const OGRSpatialReference &from, const OptName &fromName
              , const OGRSpatialReference &to, const OptName &toName)
{
    std::shared_ptr< ::OGRCoordinateTransformation>
        trans(::OGRCreateCoordinateTransformation
              (const_cast<OGRSpatialReference*>(&from)
               , const_cast<OGRSpatialReference*>(&to)));

    if (!trans) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot initialize coordinate system transformation ("
            << asName(from, fromName) <<  " ->"
            << asName(to, toName) << "): <"
            << ::CPLGetLastErrorMsg() << ">.";
    }
    return trans;
}

const OGRSpatialReference& asReference(const OGRSpatialReference &ref)
{
    return ref;
}

OptName optName(const OGRSpatialReference&)
{
    return boost::none;
}

OGRSpatialReference asReference(const SrsDefinition &def)
{
    return def.reference();
}

OptName optName(const SrsDefinition &def)
{
    return def.as(SrsDefinition::Type::proj4).srs;
}

template <typename S1, typename S2>
std::shared_ptr<void> initTrans(const S1 &from, const S2 &to)
{
    return initTransImpl(asReference(from), optName(from)
                         , asReference(to), optName(to));
}

inline OGRCoordinateTransformation& trans(const std::shared_ptr<void> &t)
{
    return *std::static_pointer_cast< ::OGRCoordinateTransformation>(t);
}

} // namespace

CsConvertor::CsConvertor(const SrsDefinition &from, const SrsDefinition &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << from.srs << " -> " << to.srs << ")";
}

CsConvertor::CsConvertor(const OGRSpatialReference &from
                         , const OGRSpatialReference &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << asName(from) << " -> " << asName(to) << ")";
}

CsConvertor::CsConvertor(const SrsDefinition &from
                         , const OGRSpatialReference &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << from.srs << " -> " << asName(to) << ")";
}

CsConvertor::CsConvertor(const OGRSpatialReference &from
                         , const SrsDefinition &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << asName(from) << " -> " << to.srs << ")";
}

math::Point2 CsConvertor::operator()(const math::Point2 &p) const
{
    double x(p(0)), y(p(1)), z(0);
    if (!(trans(trans_).Transform(1, &x, &y, &z))) {
        LOGTHROW(err1, std::runtime_error)
            << std::fixed
            << "Cannot convert point " << p << " between coordinate systems: <"
            << ::CPLGetLastErrorMsg() << ">.";
    }
    return { x, y };
}

math::Point3 CsConvertor::operator()(const math::Point3 &p) const
{
    double x(p(0)), y(p(1)), z(p(2));
    if (!(trans(trans_).Transform(1, &x, &y, &z))) {
        LOGTHROW(err1, std::runtime_error)
            << std::fixed
            << "Cannot convert point " << p << " between coordinate systems: <"
            << ::CPLGetLastErrorMsg() << ">.";
    }
    return { x, y, z };
}

CsConvertor CsConvertor::inverse() const
{
    return CsConvertor(*trans(trans_).GetTargetCS()
                       , *trans(trans_).GetSourceCS());
}

bool CsConvertor::isProjected() const
{
    return trans(trans_).GetTargetCS()->IsProjected();
}

bool CsConvertor::areSrsEqual() const
{
    auto &tr(trans(trans_));
    char *srcName;
    char *dstName;
    auto srcUnit(tr.GetSourceCS()->GetLinearUnits(&srcName));
    auto dstUnit(tr.GetTargetCS()->GetLinearUnits(&dstName));

    LOG(info1)
        << "SRS Units: " << srcUnit << "/<" << srcName << "> -> "
        << dstUnit << "/<" << dstName << ">.";

    return !std::strcmp(srcName, dstName);
}

} // namespace geo
