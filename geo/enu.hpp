#ifndef geo_enu_hpp_included_
#define geo_enu_hpp_included_

#include <vector>
#include <boost/optional.hpp>

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
};

} // namespace geo

#endif // geo_enu_hpp_included_
