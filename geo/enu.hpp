/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef geo_enu_hpp_included_
#define geo_enu_hpp_included_

#include <vector>
#include <iostream>

#include <boost/optional.hpp>

#include "math/geometry_core.hpp"

namespace geo {

struct SrsDefinition;

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

    /** Constructs ENU from origin in given SRS.
     */
    Enu(const math::Point3 &origin, const SrsDefinition &srs);
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
