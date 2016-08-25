#ifndef geo_csconvert_hpp_included_
#define geo_csconvert_hpp_included_

#include <string>
#include <memory>

#include "math/geometry_core.hpp"

#include "./project.hpp"
#include "./srsdef.hpp"

namespace geo {

class CsConvertor {
public:
    CsConvertor(const SrsDefinition &from, const SrsDefinition &to);
    CsConvertor(const OGRSpatialReference &from
                , const OGRSpatialReference &to);
    CsConvertor(const SrsDefinition &from, const OGRSpatialReference &to);
    CsConvertor(const OGRSpatialReference &from, const SrsDefinition &to);

    math::Point2 operator()(const math::Point2 &p) const;
    math::Point3 operator()(const math::Point3 &p) const;

    // return extents containing original extents
    math::Extents2 operator()(const math::Extents2 &p) const;
    math::Extents3 operator()( const math::Extents3 &p ) const;

    CsConvertor inverse() const;

    /** Is destination SRS projected?
     */
    bool isProjected() const;

    /** Are source and destination SRSs equal in linear units?
     */
    bool areSrsEqual() const;

private:
    std::shared_ptr<void> trans_;
};

// inline method implementation

inline math::Extents2 CsConvertor::operator()(const math::Extents2 &e) const
{
    math::Extents2 res(operator()( ll(e) ));
    update(res, operator()( ul(e) ));
    update(res, operator()( ur(e) ));
    update(res, operator()( lr(e) ));

    return res;
}

inline math::Extents3 CsConvertor::operator()( const math::Extents3 &e) const
{
    math::Extents3 res(operator()( bll(e) ));
    update(res, operator()( bul(e) ));
    update(res, operator()( bur(e) ));
    update(res, operator()( blr(e) ));

    update(res, operator()( tll(e) ));
    update(res, operator()( tul(e) ));
    update(res, operator()( tur(e) ));
    update(res, operator()( tlr(e) ));

    return res;
}

} // namespace geo

#endif // geo_csconvert_hpp_included_
