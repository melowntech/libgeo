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
#ifndef geo_srsdef_hpp_included_
#define geo_srsdef_hpp_included_

#include <string>

#include <boost/optional.hpp>

#include "math/geometry.hpp"

#include "utility/enum-io.hpp"

// forward declaration
class OGRSpatialReference;


namespace geo {

// forward decalration
struct Enu;

struct SrsDefinition {
    enum class Type { proj4, wkt, epsg, enu };

    std::string srs;
    Type type;

    SrsDefinition() : srs(), type(Type::proj4) {}
    SrsDefinition(const std::string &srs) : srs(srs), type(Type::proj4) {}
    SrsDefinition(const std::string &srs, Type type) : srs(srs), type(type) {}
    SrsDefinition(int epsg);
    SrsDefinition(int epsg1, int epsg2);

    SrsDefinition as(Type type) const;

    const std::string& string() const { return srs; }
    const char* c_str() const { return srs.c_str(); }

    bool empty() const { return srs.empty(); }

    bool is(Type t) const { return type == t; }

    OGRSpatialReference reference() const;
    Enu enu() const;

    static SrsDefinition fromReference(const OGRSpatialReference &src
                                       , Type type = Type::proj4);
    static SrsDefinition fromEnu(const Enu &src);

    SrsDefinition geographic() const;

    /** Convers this srs definition to string. Uses operator<< internally.
     */
    std::string toString() const;

    /** Creates SRS efinition from any string. Detects SRS from string:
     *
     *  starts with '+': Type::proj4
     *  starts with 'epsg:': Type::epsg
     *  other: Type::wkt
     */
    static SrsDefinition fromString(std::string value);


    static SrsDefinition longlat();
    static SrsDefinition utm(unsigned int zone, bool isNorth = true );
    static SrsDefinition utmFromLonglat(const math::Point2 & longlat );
};

enum class SrsEquivalence { both, geographic, vertical };

bool areSame(const SrsDefinition &def1, const SrsDefinition &def2
             , SrsEquivalence type = SrsEquivalence::both);

/** Merges horizontal part from first parameter with vertical part from second
 *  parameter.
 */
SrsDefinition merge(const SrsDefinition &horiz, const SrsDefinition &vert);

/** Adds/replaces geoid to spatial reference.
 */
SrsDefinition setGeoid(const SrsDefinition &srs, const std::string &geoidGrid);

/** Extracts geographic system from given SRS definition.
 *  Fails if there is no GeogCS node present.
 */
SrsDefinition geographic(const SrsDefinition &srs);

/** Derives geocentric system from given SRS definition.
 */
SrsDefinition geocentric(const SrsDefinition &srs);

/** Returns true if SRS is a projected spatial reference system.
 */
bool isProjected(const SrsDefinition &srs);

/** Returns true if SRS is a geographic spatial reference system.
 */
bool isGeographic(const SrsDefinition &srs);

/** SRS periodicity
 */
struct Periodicity {
    enum class Type { x, y };

    /** In which axis is this SRS periodic.
     */
    Type type;

    /** Minimum coordinate value.
     */
    double min;

    /** Maximum coordinate value.
     */
    double max;

    Periodicity(Type type, double min, double max)
        : type(type), min(min), max(max)
    {}
};


/** Tries to determine whether given SRS is periodic.
 */
boost::optional<Periodicity> isPeriodic(const SrsDefinition &srs);

UTILITY_GENERATE_ENUM_IO(SrsDefinition::Type,
    ((proj4))
    ((wkt))
    ((epsg))
    ((enu))
)

UTILITY_GENERATE_ENUM_IO(Periodicity::Type,
    ((x))
    ((y))
)

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const SrsDefinition &s)
{
    switch (s.type) {
    case SrsDefinition::Type::epsg: os << "epsg:"; break;
    default: break;
    }
    return os << s.srs;
}

inline SrsDefinition SrsDefinition::geographic() const
{
    return geo::geographic(*this);
}

} // namespace geo

#ifdef GEO_HAS_PROGRAM_OPTIONS
#    include "./po.hpp"
#endif

#endif // geo_srsdef_hpp_included_
