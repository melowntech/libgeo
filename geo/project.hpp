#ifndef geo_project_hpp_included_
#define geo_project_hpp_included_

#include <string>
#include <memory>
#include <stdexcept>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"
#include "./srsfactorsfwd.hpp"

namespace geo {

struct ProjectionError : public std::runtime_error {
    ProjectionError(const std::string &msg) : std::runtime_error(msg) {}
};

class Projection {
public:
    Projection(const SrsDefinition &def, bool inverse = false);

    math::Point2 operator()(const math::Point2 &p, bool deg = true) const;

    math::Point3 operator()(const math::Point3 &p, bool deg = true) const;

    Projection rev() const { return { proj_, !inverse_ }; };

    friend class SrsFactors;

private:
    Projection(const std::shared_ptr<void> proj, bool inverse)
        : proj_(proj), inverse_(inverse)
    {}

    std::shared_ptr<void> proj_;
    bool inverse_;
};

} // namespace geo

#endif // geo_project_hpp_included_
