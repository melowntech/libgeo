#include <stdexcept>

#include <boost/lexical_cast.hpp>

#include <proj_api.h>
#include <ogr_spatialref.h>
#include <cpl_error.h>

#include "dbglog/dbglog.hpp"

#include "./csconvertor.hpp"
#include "./detail/srs.hpp"

namespace geo {

class CsConvertor::Impl : boost::noncopyable
{
public:
    typedef std::shared_ptr<Impl> pointer;
    virtual ~Impl() {}

    virtual math::Point2 convert(const math::Point2 &p) const = 0;
    virtual math::Point3 convert(const math::Point3 &p) const = 0;
    virtual bool isProjected() const { return true; }
    virtual bool areSrsEqual() const = 0;

    virtual pointer inverse() const = 0;
};

namespace {

namespace ublas = boost::numeric::ublas;

typedef boost::optional<std::string> OptName;

std::string asName(const OGRSpatialReference &ref
                   , const OptName &name = boost::none)
{
    if (name) { return *name; }
    return boost::lexical_cast<std::string>(SrsDefinition::fromReference(ref));
}

std::string asName(const Enu &enu, const OptName &name = boost::none)
{
    if (name) { return *name; }
    return boost::lexical_cast<std::string>(SrsDefinition::fromEnu(enu));
}

std::unique_ptr< ::OGRCoordinateTransformation>
initOgr(const OGRSpatialReference &from, const OptName &fromName
        , const OGRSpatialReference &to, const OptName &toName)
{
    std::unique_ptr< ::OGRCoordinateTransformation>
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

class OgrImpl : public CsConvertor::Impl
{
public:
    OgrImpl(const SrsDefinition &from, const SrsDefinition &to)
        : trans_(initOgr(from.reference(), from.srs
                         , to.reference(), to.srs))
    {}

    OgrImpl(const OGRSpatialReference &from, const OGRSpatialReference &to)
        : trans_(initOgr(from, boost::none
                         , to, boost::none))
    {}

    OgrImpl(const SrsDefinition &from, const OGRSpatialReference &to)
        : trans_(initOgr(from.reference(), from.srs
                         , to, boost::none))
    {}

    OgrImpl(const OGRSpatialReference &from, const SrsDefinition &to)
        : trans_(initOgr(from, boost::none
                         , to.reference(), to.srs))
    {}

    virtual math::Point2 convert(const math::Point2 &p) const {
        double x(p(0)), y(p(1));
        if (!(trans_->Transform(1, &x, &y))) {
            LOGTHROW(err1, ProjectionError)
                << "Cannot convert point between coordinate systems: <"
                << ::CPLGetLastErrorMsg() << ">.";
        }
        return { x, y };
    }

    virtual math::Point3 convert(const math::Point3 &p) const {
        double x(p(0)), y(p(1)), z(p(2));
        if (!(trans_->Transform(1, &x, &y, &z))) {
            LOGTHROW(err1, ProjectionError)
                << "Cannot convert point between coordinate systems: <"
                << ::CPLGetLastErrorMsg() << ">.";
        }
        return { x, y, z };
    }

    virtual bool isProjected() const {
        return trans_->GetTargetCS()->IsProjected();
    }

    virtual pointer inverse() const {
        return std::make_shared<OgrImpl>
            (*trans_->GetTargetCS(), *trans_->GetSourceCS());
    }

    virtual bool areSrsEqual() const {
        char *srcName;
        char *dstName;
        auto srcUnit(trans_->GetSourceCS()->GetLinearUnits(&srcName));
        auto dstUnit(trans_->GetTargetCS()->GetLinearUnits(&dstName));

        LOG(info1)
            << "SRS Units: " << srcUnit << "/<" << srcName << "> -> "
            << dstUnit << "/<" << dstName << ">.";

        return !std::strcmp(srcName, dstName);
    }

private:
    std::unique_ptr< ::OGRCoordinateTransformation> trans_;
};

namespace {
static struct Initializer {
    Initializer() {
        ::pj_acquire_lock();
        ::pj_release_lock();
    }
} initializer;
}

std::unique_ptr< ::OGRCoordinateTransformation>
initOgr2Enu(const OGRSpatialReference &from, const OptName &fromName
            , const Enu &enu, bool inverse)
{
    ::OGRSpatialReference ll;
    if (!enu.spheroid) {
        // WGS84
        ll = *OGRSpatialReference::GetWGS84SRS();
    } else {
        // another ellipsoid
        // TODO: use some better names
        if (OGRERR_NONE
            != ll.SetGeogCS("lonlat", "lonlat", "sphereoid"
                            , enu.spheroid->a
                            , enu.spheroid->f1()))
        {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot initialize coordinate system transformation ("
                << asName(from, fromName) <<  " -> latlon): <"
                << ::CPLGetLastErrorMsg() << ">.";
        }
    }

    // apply toWGS84 params
    switch (enu.towgs84.size()) {
    case 0:
        // force proper vertical datum
        ll.SetTOWGS84(0.0, 0.0, 0.0);
        break;

    case 3:
        ll.SetTOWGS84(enu.towgs84[0], enu.towgs84[1], enu.towgs84[2]);
        break;
    case 7:
        ll.SetTOWGS84(enu.towgs84[0], enu.towgs84[1], enu.towgs84[2]
                      , enu.towgs84[3], enu.towgs84[4], enu.towgs84[5]
                      , enu.towgs84[6]);
        break;

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Cannot initialize coordinate system transformation for ENU:"
            " towgs84 must have either 3 or 7 elements.";
    }

    std::unique_ptr< ::OGRCoordinateTransformation>
        trans(inverse
              ? ::OGRCreateCoordinateTransformation
              (&ll, const_cast<OGRSpatialReference*>(&from))
              : ::OGRCreateCoordinateTransformation
              (const_cast<OGRSpatialReference*>(&from), &ll));

    if (!trans) {
        if (inverse) {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot initialize coordinate system transformation ("
                << asName(from, fromName) <<  " -> latlon): <"
                << ::CPLGetLastErrorMsg() << ">.";
        } else {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot initialize coordinate system transformation "
                "(latlon -> "
                << asName(from, fromName) <<  "): <"
                << ::CPLGetLastErrorMsg() << ">.";
        }
    }

    return trans;
}

GeographicLib::LocalCartesian initLc(const Enu &enu)
{
    if (!enu.spheroid) {
        return GeographicLib::LocalCartesian(enu.lat0, enu.lon0, enu.h0);
    }

    return GeographicLib::LocalCartesian
        (enu.lat0, enu.lon0, enu.h0
         , GeographicLib::Geocentric(enu.spheroid->a
                                     , enu.spheroid->f()));
}

class Ogr2EnuImpl : public CsConvertor::Impl
{
public:
    Ogr2EnuImpl(const SrsDefinition &from, const SrsDefinition &to
                , bool inverse)
        : enu_(to.enu())
        , lc_(initLc(enu_))
        , trans_(initOgr2Enu(from.reference(), from.srs, enu_, inverse))
        , inverse_(inverse)
    {}

    Ogr2EnuImpl(const OGRSpatialReference &from, const SrsDefinition &to
                , bool inverse)
        : enu_(to.enu())
        , lc_(initLc(enu_))
        , trans_(initOgr2Enu(from, boost::none, enu_, inverse))
        , inverse_(inverse)
    {}

    Ogr2EnuImpl(const SrsDefinition &from, const Enu &to
                , bool inverse)
        : enu_(to)
        , lc_(initLc(enu_))
        , trans_(initOgr2Enu(from.reference(), from.srs, enu_, inverse))
        , inverse_(inverse)
    {}

    virtual math::Point2 convert(const math::Point2 &p) const {
        return convert(math::Point3(p(0), p(1), 0.0));
    }

    virtual math::Point3 convert(const math::Point3 &p) const {
        if (inverse_) {
            // ENU > lonlat
            GeographicLib::Math::real lat, lon, z;
            lc_.Reverse(p(0), p(1), p(2), lat, lon, z);

            // lonlat -> srs
            double xx(lon), yy(lat), zz(z);
            if (!(trans_->Transform(1, &xx, &yy, &zz))) {
                LOGTHROW(err1, std::runtime_error)
                    << "Cannot convert point between coordinate systems: <"
                    << ::CPLGetLastErrorMsg() << ">.";
            }
            return { xx, yy, zz };
        }

        // SRS -> lonlat
        double x(p(0)), y(p(1)), z(p(2));
        if (!(trans_->Transform(1, &x, &y, &z))) {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot convert point between coordinate systems: <"
                << ::CPLGetLastErrorMsg() << ">.";
        }

        // lonlat -> ENU
        GeographicLib::Math::real xx, yy, zz;
        lc_.Forward(y, x, z, xx, yy, zz);

        // done
        return { double(xx), double(yy), double(zz) };
    }

    virtual bool isProjected() const {
        return trans_->GetTargetCS()->IsProjected();
    }

    virtual bool areSrsEqual() const {
        // TODO: implement me
        return false;
    }

    virtual pointer inverse() const {
        return  std::make_shared<Ogr2EnuImpl>
            (*trans_->GetTargetCS(), *trans_->GetSourceCS()
             , enu_, !inverse_);
    }

    Ogr2EnuImpl(const OGRSpatialReference &src
                , const OGRSpatialReference &dst
                , const Enu &enu
                , bool inverse)
        : enu_(enu), trans_(::OGRCreateCoordinateTransformation
                            (const_cast<OGRSpatialReference*>(&src)
                             , const_cast<OGRSpatialReference*>(&dst)))
        , inverse_(inverse)
    {
        if (!trans_) {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot initialize coordinate system transformation ("
                << asName(src) <<  " -> " << asName(dst) << "): <"
                << ::CPLGetLastErrorMsg() << ">.";
        }
    }

private:
    Enu enu_;
    GeographicLib::LocalCartesian lc_;
    std::unique_ptr< ::OGRCoordinateTransformation> trans_;
    bool inverse_;
};

std::shared_ptr<CsConvertor::Impl>
initTrans(const SrsDefinition &from, const SrsDefinition &to)
{
    if (from.is(SrsDefinition::Type::enu) || to.is(SrsDefinition::Type::enu)) {
        if (from.type == to.type) {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot convert between two ENU system so far.";
        }

        if (from.is(SrsDefinition::Type::enu)) {
            return std::make_shared<Ogr2EnuImpl>(to, from, true);
        }

        return std::make_shared<Ogr2EnuImpl>(from, to, false);
    }

    return std::make_shared<OgrImpl>(from, to);
}

std::shared_ptr<CsConvertor::Impl>
initTrans(const OGRSpatialReference& from, const OGRSpatialReference &to)
{
    return std::make_shared<OgrImpl>(from, to);
}

std::shared_ptr<CsConvertor::Impl>
initTrans(const SrsDefinition &from, const OGRSpatialReference &to)
{
    if (from.is(SrsDefinition::Type::enu)) {
        return std::make_shared<Ogr2EnuImpl>(to, from, true);
    }

    return std::make_shared<OgrImpl>(from, to);
}

std::shared_ptr<CsConvertor::Impl>
initTrans(const OGRSpatialReference &from, const SrsDefinition &to)
{
    if (to.is(SrsDefinition::Type::enu)) {
        return std::make_shared<Ogr2EnuImpl>(from, to, false);
    }

    return std::make_shared<OgrImpl>(from, to);
}

std::shared_ptr<CsConvertor::Impl>
initTrans(const SrsDefinition &from, const Enu &to)
{
    return std::make_shared<Ogr2EnuImpl>(from, to, false);
}

} // namespace

CsConvertor::CsConvertor(const SrsDefinition &from, const SrsDefinition &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << from << " -> " << to << ").";
}

CsConvertor::CsConvertor(const OGRSpatialReference &from
                         , const OGRSpatialReference &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << asName(from) << " -> " << asName(to) << ").";
}

CsConvertor::CsConvertor(const SrsDefinition &from
                         , const OGRSpatialReference &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << from << " -> " << asName(to) << ").";
}

CsConvertor::CsConvertor(const OGRSpatialReference &from
                         , const SrsDefinition &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << asName(from) << " -> " << to << ").";
}

CsConvertor::CsConvertor(const SrsDefinition &from, const Enu &to)
    : trans_(initTrans(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << from << " -> " << asName(to) << ").";
}

CsConvertor::CsConvertor(const std::shared_ptr<Impl> &trans)
    : trans_(trans)
{}

math::Point2 CsConvertor::operator()(const math::Point2 &p) const
{
    return trans_->convert(p);
}

math::Point3 CsConvertor::operator()(const math::Point3 &p) const
{
    return trans_->convert(p);
}

CsConvertor CsConvertor::inverse() const
{
    return CsConvertor(trans_->inverse());
}

bool CsConvertor::isProjected() const
{
    return trans_->isProjected();
}

bool CsConvertor::areSrsEqual() const
{
    return trans_->areSrsEqual();
}

} // namespace geo
