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

#include <boost/utility/in_place_factory.hpp>

#include <ogr_spatialref.h>
#include <cpl_conv.h>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"

#include "srsdef.hpp"
#include "srs.hpp"
#include "enu.hpp"
#include "csconvertor.hpp"

namespace geo {

Enu::Enu(const math::Point3 &origin, const SrsDefinition &srs)
{
    auto geog(srs.reference());
    auto o(origin);

    if (!geog.IsGeographic()) {
        // extract geographic ref from this srs
        ::OGRSpatialReference tmp;
        if (tmp.CopyGeogCSFrom(&geog) != OGRERR_NONE) {
            LOGTHROW(err1, std::runtime_error)
                << "Could not extract geographic SRS from definition \""
                << srs << "\".";
        }

        // convert origin
        const CsConvertor conv(geog, tmp);
        o = conv(o);
    }

    // construct ENU from geographic system's spheroid and origin
    lon0 = o(0);
    lat0 = o(1);
    h0 = o(2);

    // TODO: check for WGS84 and do not set following
    spheroid = boost::in_place(geog.GetSemiMajor(), geog.GetSemiMinor());
    towgs84.resize(7);
    geog.GetTOWGS84(towgs84.data(), int(towgs84.size()));
}

} // namespace geo
