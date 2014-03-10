#ifndef geo_csconvert_hpp_included_
#define geo_csconvert_hpp_included_

#include <string>
#include <memory>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"

namespace geo {

class CsConvertor {
public:
    CsConvertor(const SrsDefinition &from, const SrsDefinition &to);

    math::Point2 operator()(const math::Point2 &p) const;

    math::Point3 operator()(const math::Point3 &p) const;

    CsConvertor inverse() const;

private:
    const SrsDefinition from_;
    const SrsDefinition to_;
    std::shared_ptr<void> trans_;
};

} // namespace geo

#endif // geo_csconvert_hpp_included_
