#include <stdexcept>
#include <tuple>

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"
#include "imgproc/rastermask.hpp"

#include "./geodataset.hpp"
#include "./dem.hpp"

namespace geo {

namespace fs = boost::filesystem;

DemCloud loadDem(const fs::path &path
                 , boost::optional<math::Extents2> extents
                 , boost::optional<SrsDefinition> dstSrs)
{
    auto source(GeoDataset::createFromFS(path));

    if (!dstSrs) {
        dstSrs = source.srsProj4();
    }

    if (!extents) {
        extents = source.deriveExtents(*dstSrs);
    }

    // generate in memory data set; size is calculated from source resolution
    auto gd(GeoDataset::deriveInMemory
            (source, *dstSrs, boost::none, *extents));

    source.warpInto(gd);

    return { gd.exportPointCloud(), *dstSrs };
}

} // namespace geo
