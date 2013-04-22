#include <proj_api.h>

#include "dbglog/dbglog.hpp"

#include "project.hpp"

namespace geo {

Projection::Projection(const std::string &def, bool inverse)
    : proj_(pj_init_plus(def.c_str())
            , [](void *ptr) {if (ptr) pj_free(ptr); })
    , inverse_(inverse)
{
    LOG(info4) << "def: " << def;
}

namespace {
    constexpr double DEG2RAD(M_PI / 180.);
}

math::Point2 Projection::operator()(const math::Point2 &p) const
{
    auto res(inverse_
             ? pj_inv({ p(0), p(1) }, proj_.get())
             : pj_fwd({ p(0) * DEG2RAD, p(1) * DEG2RAD}, proj_.get()));

    return { res.u, res.v };
}

} // namespace geo
