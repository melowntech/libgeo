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
#include <stdexcept>
#include <tuple>

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"
#include "imgproc/rastermask.hpp"

#include "./geodataset.hpp"
#include "./dem.hpp"
#include "./srsfactors.hpp"

namespace geo {

namespace fs = boost::filesystem;

DemCloud loadDem(const fs::path &path
                 , boost::optional<math::Extents2> extents
                 , boost::optional<SrsDefinition> dstSrs
                 , bool adjustVertical
                 , bool pointsInGrid)
{
    auto source(GeoDataset::createFromFS(path));

    if (!dstSrs) {
        dstSrs = source.srsProj4();
    }

    if (!extents) {
        extents = source.deriveExtents(*dstSrs);
    } else if (pointsInGrid) {
        // add halfpixel
        const auto res(source.resolution());
        extents->ll -= (res / 2.0);
        extents->ur += (res / 2.0);
    }

    // generate in memory data set; size is calculated from source resolution
    auto gd(GeoDataset::deriveInMemory
            (source, *dstSrs, boost::none, *extents));

    source.warpInto(gd);

    DemCloud dc{gd.exportPointCloud(), *dstSrs};

    if (adjustVertical) {
        geo::SrsFactors sf(gd.srsProj4(), source.srsProj4());
        for (auto &p : dc.pc) {
            // update height
            p(2) *= sf(p).meridionalScale;
        }
    }

    return dc;
}

} // namespace geo
