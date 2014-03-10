#include <stdexcept>

#include <GeographicLib/Geoid.hpp>

#include "dbglog/dbglog.hpp"

#include "./heightconvertor.hpp"
#include "./detail/srs.hpp"

namespace geo {

namespace {

std::shared_ptr<void> initGeoid(const boost::filesystem::path &geoidDir)
{
    return std::make_shared<GeographicLib::Geoid>
        ("egm96-5", geoidDir.string());
}

} // namespace

HeightConvertor::HeightConvertor(const SrsDefinition &srs
                                 , const boost::filesystem::path &geoidDir
                                 , Direction direction)
    : proj_(srs, true), geoid_(initGeoid(geoidDir))
    , toGeoid_(direction == Direction::toGeoid)
{
    LOG(info1) << "Geoid initialized";
}

math::Point3 HeightConvertor::operator()(const math::Point3 &p) const
{
    auto latlon(proj_(p));
    return {p(0), p(1)
            , std::static_pointer_cast<GeographicLib::Geoid>(geoid_)
            ->ConvertHeight
            (latlon(1), latlon(0), p(2)
             , (toGeoid_ ? GeographicLib::Geoid::ELLIPSOIDTOGEOID
                : GeographicLib::Geoid::GEOIDTOELLIPSOID)) };
}

} // namespace geo
