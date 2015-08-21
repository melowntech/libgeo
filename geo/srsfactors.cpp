#include <stdexcept>

#include <proj_api.h>

#include "dbglog/dbglog.hpp"

#include "./srsfactors.hpp"

namespace geo {

SrsFactors::SrsFactors(const SrsDefinition &def)
    : proj_(def), srcProj_(proj_.rev())
{}

SrsFactors::SrsFactors(const SrsDefinition &def, const SrsDefinition &src)
    : proj_(def), srcProj_(src, true)
{}

SrsFactors::Factors SrsFactors::operator()(const math::Point2 &p) const
{
    // obtain lat/lon from p
    auto pp(srcProj_(p, false));

    Factors f;

    if (::geo_detail_pj_factors(pp(0), pp(1), proj_.proj_.get(), &f)) {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to get SRS factors for coordinates " << p << ": "
            << ::pj_strerrno(::pj_errno);
    }

    return f;
}

} // namespace geo
