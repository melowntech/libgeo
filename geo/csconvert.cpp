#include <stdexcept>

#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"

#include "./csconvert.hpp"
#include "./detail/srs.hpp"

namespace geo {

namespace {

std::shared_ptr<void> initTrans(const SrsDefinition &from
                                , const SrsDefinition &to)
{
    OGRSpatialReference srFrom;
    detail::import(srFrom, from);
    OGRSpatialReference srTo;
    detail::import(srTo, to);

    return std::shared_ptr<OGRCoordinateTransformation>
        (OGRCreateCoordinateTransformation(&srFrom, &srTo));
}

} // namespace

CsConvertor::CsConvertor(const SrsDefinition &from, const SrsDefinition &to)
    : from_(from), to_(to), trans_(initTrans(from, to))
{
    if (!trans_) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot initialize coordinate system transformation ("
            << from.srs <<  " ->" << to.srs << ")";
    }
    LOG(info1) << "Coordinate system transformation ("
               << from.srs <<  " ->" << to.srs << ")";
}

math::Point2 CsConvertor::operator()(const math::Point2 &p) const
{
    double x(p(0)), y(p(1));
    if (!(std::static_pointer_cast<OGRCoordinateTransformation>(trans_)
          ->Transform(1, &x, &y)))
    {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot convert point between coordinate systems.";
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

} // namespace geo
