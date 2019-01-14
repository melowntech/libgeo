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
#ifndef geo_dem_hpp_included_
#define geo_dem_hpp_included_

#include <string>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"

namespace geo {

struct DemCloud {
    math::Points3 pc;
    SrsDefinition projectionReference;

    operator math::Points3&() { return pc; }
    operator const math::Points3&() const { return pc; }
};

/** Load part of dem as a pointcloud.
 *
 * Known deficiencies:
 *
 *     * vertical components are not touched, gives proper output only when
 *       vertical datum in source and destination SRS are the same
 *
 * \param path path to GDAL dataset
 * \param extents optional DEM subset extents; defaults to full DEM extents
 * \param dstSrs pointcloud SRS; dfefaults to DEM SRS
 * \param adjustVertical apply vertial adjustment if true
 * \param pointsInGrid update extents so that points are not in pixel centers
 *                     but in pixel corners
 *
 * \return derived pointcloud
 */
DemCloud loadDem(const boost::filesystem::path &path
                 , boost::optional<math::Extents2> extents = boost::none
                 , boost::optional<SrsDefinition> dstSrs = boost::none
                 , bool adjustVertical = false
                 , bool pointsInGrid = false);

} // namespace geo

#endif // geo_dem_hpp_included_
