#ifndef geo_srsdactors_hpp_included_
#define geo_srsdactors_hpp_included_

#include <string>
#include <memory>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"
#include "./project.hpp"
#include "./detail/pjfactors.h"

namespace geo {

class SrsFactors {
public:
    /** Creates SRS factors interfaces; all queries are expected to be in given
     *  SRS.
     */
    SrsFactors(const SrsDefinition &def);

    /** Creates SRS factors interfaces; all queries are expected to be in given
     *  src SRS.
     */
    SrsFactors(const SrsDefinition &def
               , const SrsDefinition &src);

    typedef struct ::geo_detail_pj_factors Factors;

    Factors operator()(const math::Point2 &p) const;

    Factors operator()(const math::Point3 &p) const {
        return operator()(math::Point2(p(0), p(1)));
    }

private:
    Projection proj_;
    Projection srcProj_;
};

} // namespace geo

#endif // geo_srsdactors_hpp_included_
