#ifndef geo_project_hpp_included_
#define geo_project_hpp_included_

#include <string>
#include <memory>

#include "math/geometry_core.hpp"

namespace geo {

class Projection {
public:
    Projection(const std::string &def, bool inverse = false);

    math::Point2 operator()(const math::Point2 &p) const;

    Projection rev() const { return { proj_, !inverse_ }; };

private:
    Projection(const std::shared_ptr<void> proj, bool inverse)
        : proj_(proj), inverse_(inverse)
    {}

    std::shared_ptr<void> proj_;
    bool inverse_;
};

} // namespace geo

#endif // geo_project_hpp_included_
