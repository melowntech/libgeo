#include <stdexcept>

#include <ogr_spatialref.h>
#include <cpl_error.h>

#include "dbglog/dbglog.hpp"

#include "./csconvertor.hpp"
#include "./detail/srs.hpp"

namespace geo {

namespace {

namespace ublas = boost::numeric::ublas;

std::shared_ptr<void> initTrans(const SrsDefinition &from
                                , const SrsDefinition &to)
{
    OGRSpatialReference srFrom;
    detail::import(srFrom, from);
    OGRSpatialReference srTo;
    detail::import(srTo, to);

    std::shared_ptr< ::OGRCoordinateTransformation>
        trans(::OGRCreateCoordinateTransformation(&srFrom, &srTo));

    if (!trans) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot initialize coordinate system transformation ("
            << from.srs <<  " ->" << to.srs << "): <"
            << ::CPLGetLastErrorMsg() << ">.";
    }
    return trans;
}

inline OGRCoordinateTransformation& trans(const std::shared_ptr<void> &t)
{
    return *std::static_pointer_cast< ::OGRCoordinateTransformation>(t);
}

} // namespace

CsConvertor::CsConvertor(const SrsDefinition &from, const SrsDefinition &to)
    : from_(from), to_(to), trans_(initTrans(from, to))
    , srcMetricScale_(trans(trans_).GetSourceCS()->GetLinearUnits())
    , dstMetricScale_(trans(trans_).GetTargetCS()->GetLinearUnits())
{
    LOG(info1) << "Coordinate system transformation ("
               << from.srs <<  " ->" << to.srs << "); "
               << "Scales to metric system: ("
               << srcMetricScale_ << ", " << dstMetricScale_ << ").";
}

math::Point2 CsConvertor::operator()(const math::Point2 &p) const
{
    double x(p(0)), y(p(1));
    if (!(trans(trans_).Transform(1, &x, &y))) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot convert point between coordinate systems: <"
            << ::CPLGetLastErrorMsg() << ">.";
    }
    return { x, y };
}

math::Point3 CsConvertor::operator()(const math::Point3 &p) const
{
    auto xy(operator()(math::Point2(p(0), p(1))));
    return { xy(0), xy(1), p(2) };
}

CsConvertor CsConvertor::inverse() const
{
    return CsConvertor(to_, from_);
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

double CsConvertor::dilation(const math::Point2 &point) const
{
    // one kilemeter
    const double checkDistance(1000.);

    // checkpoint (one kilometer from point)
    const math::Point2 checkPoint(point(0) + checkDistance / srcMetricScale_
                                  , point(1));

    // transform point to destination SRS
    auto xPoint(operator()(point));
    auto xCheckPoint(operator()(checkPoint));

    // calculate dilation
    auto dilation((ublas::norm_2(xCheckPoint - xPoint) * dstMetricScale_)
                  / checkDistance);

    LOG(info1) << "Dilation for " << point << " is " << dilation << ".";

    return dilation;
}

} // namespace geo
