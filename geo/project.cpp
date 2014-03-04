#include <stdexcept>
#include <vector>

#include <proj_api.h>
#include <ogr_spatialref.h>
#include <cpl_conv.h>

#include "dbglog/dbglog.hpp"

#include "project.hpp"

namespace geo {

namespace {

std::string wkt2Proj(const std::string &def)
{
    OGRSpatialReference sr;

    {
        // convert NUL-terminated string to char vector
        std::vector<char> tmp(def.c_str(), def.c_str() + def.size() + 1);
        char *data(tmp.data());
        auto err(sr.importFromWkt(&data));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing wkt definition: <" << err << ">.";
        }
    }

    char *out(nullptr);

    auto err(sr.exportToProj4(&out));
    if (err != OGRERR_NONE) {
        ::CPLFree(out);
        LOGTHROW(err1, std::runtime_error)
            << "Error converting wkt to proj definition: <"
            << err << ">.";
    }
    std::string projDef(out);
    ::CPLFree(out);

    // TODO: implement me
    return projDef;
}

std::shared_ptr<void> initProj(const Projection::Definition &def)
{
    std::string projDef;
    switch (def.type) {
    case Projection::Definition::Type::proj:
        projDef = def.def;
        break;

    case Projection::Definition::Type::wkt:
        projDef = wkt2Proj(def.def);
        break;
    }

    return { pj_init_plus(projDef.c_str())
            , [](void *ptr) { if (ptr) pj_free(ptr); } };
}

} // namespace

Projection::Projection(const Definition &def, bool inverse)
    : proj_(initProj(def)), inverse_(inverse)
{
    LOG(info1) << "Projection(" << def.def << ")";
}

namespace {
    constexpr double DEG2RAD(M_PI / 180.);
    constexpr double RAD2DEG(180. / M_PI);
}

math::Point2 Projection::operator()(const math::Point2 &p) const
{
    if (inverse_) {
        auto res(pj_inv({ p(0), p(1) }, proj_.get()));
        return { res.u * RAD2DEG, res.v * RAD2DEG };
    }

    auto res(pj_fwd({ p(0) * DEG2RAD, p(1) * DEG2RAD}, proj_.get()));
    return { res.u, res.v };
}

math::Point3 Projection::operator()(const math::Point3 &p) const
{
    auto xy(operator()(math::Point2(p(0), p(1))));
    return { xy(0), xy(1), p(2) };
}

} // namespace geo
