#ifndef geo_heightconvertor_hpp_included_
#define geo_heightconvertor_hpp_included_

#include <string>
#include <memory>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"
#include "./project.hpp"

namespace geo {

class HeightConvertor {
public:
    enum class Direction {
        toEllipsoid, toGeoid
    };

    /** Construct height convertor.
     */
    HeightConvertor(const SrsDefinition &srs
                    , const boost::filesystem::path &geoidDir
                    , Direction direction = Direction::toGeoid);

    math::Point3 operator()(const math::Point3 &p) const;

private:
    Projection proj_;
    std::shared_ptr<void> geoid_;
    bool toGeoid_;
};

} // namespace geo

#endif // geo_heightconvertor_hpp_included_
