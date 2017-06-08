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
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/utility/in_place_factory.hpp>


#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"

#include "./srs.hpp"

namespace ba = boost::algorithm;

namespace geo { namespace detail {

void import(OGRSpatialReference &sr, const SrsDefinition &def)
{
    switch (def.type) {
    case SrsDefinition::Type::proj4: {
        auto err(sr.importFromProj4(def.srs.c_str()));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing proj definition: <" << err << "> (input = "
                << def.srs << ").";
        }
        break;
    }

    case SrsDefinition::Type::wkt: {
        // convert NUL-terminated string to char vector
        std::vector<char> tmp(def.srs.c_str()
                              , def.srs.c_str() + def.srs.size() + 1);
        char *data(tmp.data());
        auto err(sr.importFromWkt(&data));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing wkt definition: <" << err << "> (input = "
                << def.srs << ").";
        }
        break;
    }

    case SrsDefinition::Type::epsg: {
        auto err(sr.SetFromUserInput(("EPSG:" + def.srs).c_str()));
        if (err != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Error parsing EPSG definition: <"
                    << err << "> (input = "
                << def.srs << ").";
        }
        break;
    }

    case SrsDefinition::Type::enu:
        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS is not supported by OGR library.";
    }
}

namespace {
typedef boost::iterator_range<std::string::const_iterator> SubString;
typedef std::vector<SubString> Args;
typedef std::pair<SubString, SubString> KeyValue;

inline KeyValue splitArgument(const SubString &arg)
{
    auto b(std::begin(arg));
    auto e(std::end(arg));
    for (auto i(b); i != e; ++i) {
        if (*i == '=') {
            return KeyValue(SubString(b, i), SubString(std::next(i), e));
        }
    }
    return KeyValue(SubString(b, e), SubString());
}

inline std::string asString(const SubString &arg)
{
    return std::string(std::begin(arg), std::end(arg));
}

inline double asDouble(const SubString &arg, const SubString &what)
{
    try {
        return boost::lexical_cast<double>(asString(arg));
    } catch (boost::bad_lexical_cast) {
        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS: Value <" << asString(arg)
            << "> of key <" << asString(what)
            << "> is not a real number.";
    }
    return .0;
}

} // namespace

void import(Enu &enu, const SrsDefinition &def)
{
    if (!def.is(SrsDefinition::Type::enu)) {
        LOGTHROW(err1, std::runtime_error)
            << "Only ENU SRS is supported by GeographicLib library.";
    }

    enu = Enu();
    auto a = boost::make_optional(false, 0.0);
    auto b = boost::make_optional(false, 0.0);

    Args args;
    ba::split(args, def.srs, ba::is_any_of(" \t\v\r\n\f")
              , ba::token_compress_on);

    for (auto bargs(args.begin()), iargs(bargs), eargs(args.end());
         iargs != eargs; ++iargs)
    {
        if (iargs == bargs) {
            if (!ba::equals(*iargs, "enu")) {
                LOGTHROW(err1, std::runtime_error)
                    << "ENU SRS not starting with \"enu\" keyword.";
            }

            continue;
        }

        auto kv(splitArgument(*iargs));

        if (ba::equals(kv.first, "lat0")) {
            enu.lat0 = asDouble(kv.second, kv.first);
            continue;
        }

        if (ba::equals(kv.first, "lon0")) {
            enu.lon0 = asDouble(kv.second, kv.first);
            continue;
        }

        if (ba::equals(kv.first, "h0")) {
            enu.h0 = asDouble(kv.second, kv.first);
            continue;
        }

        if (ba::equals(kv.first, "a")) {
            a = asDouble(kv.second, kv.first);
            continue;
        }

        if (ba::equals(kv.first, "b")) {
            b = asDouble(kv.second, kv.first);
            continue;
        }

        if (ba::equals(kv.first, "towgs84")) {
            Args wgs84;
            ba::split(wgs84, kv.second, ba::is_any_of(","));
            if ((wgs84.size() != 3) && (wgs84.size() != 7)) {
                LOGTHROW(err1, std::runtime_error)
                    << "ENU SRS: towgs84 must have either 3 or 7 elements.";
            }

            for (const auto &element : wgs84) {
                enu.towgs84.push_back(asDouble(element, kv.first));
            }

            continue;
        }

        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS: unknown key<" << asString(kv.first) << ">.";
    }

    if (bool(a) != bool(b)) {
        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS: either both or none of a and b must be specified.";
    }

    if (a) {
        enu.spheroid = boost::in_place(*a, *b);
    }
}

} } // namespace geo::detail
