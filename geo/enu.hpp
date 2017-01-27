#ifndef geo_enu_hpp_included_
#define geo_enu_hpp_included_

#include <vector>
#include <iostream>

#include <boost/optional.hpp>

#include "math/geometry_core.hpp"

namespace geo {

struct Enu {
    struct Spheroid {
        double a;
        double b;

        Spheroid(double a = .0, double b = .0) : a(a), b(b) {}

        double f() const { return (a - b) / a; }
        double f1() const {
            return (a == b) ? .0 : (a / (a - b));
        }
    };

    double lat0;
    double lon0;
    double h0;
    boost::optional<Spheroid> spheroid;
    std::vector<double> towgs84;

    Enu() : lat0(), lon0(), h0() {}

    Enu(const math::Point3 &origin)
        : lat0(origin(1)), lon0(origin(0)), h0(origin(2))
    {}
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Enu &e)
{
    os << "enu";

    os << std::fixed << std::setprecision(12);

    if (e.lat0) { os << " lat0=" << e.lat0; }
    if (e.lon0) { os << " lon0=" << e.lon0; }
    if (e.h0) { os << " h0=" << e.h0; }

    if (e.spheroid) {
        os << " a=" << e.spheroid->a << " b=" << e.spheroid->b;
    }

    if (!e.towgs84.empty()) {
        os << " towgs84=" << utility::join(e.towgs84, ",");
    }

    return os;
}

} // namespace geo

#endif // geo_enu_hpp_included_
