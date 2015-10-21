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

    // return extents containing original extents
    math::Extents2 operator()(const math::Extents2 &p) const;
    math::Extents3 operator()( const math::Extents3 &p
                             , bool adjVertical ) const;

    CsConvertor inverse() const;

    double dilation(const math::Point2 &point) const;
    double dilation(const math::Point3 &point) const;

    /** Adjust vertical coordinate by lenght dilation.
     */
    math::Point3 adjustVertical(math::Point3 point) const;

    /** Is destination SRS projected?
     */
    bool isProjected() const;

    /** Are source and destination SRSs equal in linear units?
     */
    bool areSrsEqual() const;

private:
    SrsDefinition from_;
    SrsDefinition to_;
    std::shared_ptr<void> trans_;
    double srcMetricScale_;
    double dstMetricScale_;
};

// inline method implementation

inline double CsConvertor::dilation(const math::Point3 &point) const
{
    return dilation(math::Point2(point(0), point(1)));
}

inline math::Point3 CsConvertor::adjustVertical(math::Point3 point) const
{
    point(2) *= dilation(point);
    return point;
}

inline math::Extents2 CsConvertor::operator()(const math::Extents2 &e) const
{
    math::Extents2 res(operator()( ll(e) ));
    update(res, operator()( ul(e) ));
    update(res, operator()( ur(e) ));
    update(res, operator()( lr(e) ));

    return res;
}

inline math::Extents3 CsConvertor::operator()( const math::Extents3 &e
                                             , bool adjVertical ) const
{
    auto adj([&](const math::Point3 &p) -> math::Point3 {
        if (adjVertical) { return adjustVertical(p); }
        return p;
    });

    math::Extents3 res(operator()( adj(bll(e)) ));
    update(res, operator()( adj(bul(e)) ));
    update(res, operator()( adj(bur(e)) ));
    update(res, operator()( adj(blr(e)) ));

    update(res, operator()( adj(tll(e)) ));
    update(res, operator()( adj(tul(e)) ));
    update(res, operator()( adj(tur(e)) ));
    update(res, operator()( adj(tlr(e)) ));

    return res;
}

} // namespace geo

#endif // geo_csconvert_hpp_included_
