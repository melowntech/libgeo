#ifndef VADSTENA_OGRPOLYGON_HPP
#define VADSTENA_OGRPOLYGON_HPP

#include <boost/filesystem.hpp>

#include "ogrsf_frmts.h"

#include "csconvertor.hpp"
#include "dbglog/dbglog.hpp"
#include "srsdef.hpp"

namespace geo::ogr
{
namespace
{
namespace fs = boost::filesystem;
}

/**
 * Wrapper class around OGRMultipolygon for interaction between GDAL/OGR and
 * vadstena geometry.
 */
class MultiPolygon
{
public:
    MultiPolygon(const OGRMultiPolygon& polygons);

    MultiPolygon(MultiPolygon&& polygons);

    const OGRMultiPolygon& polygons() const;

    /**
     * Checks if 2 multipolygons overlap.
     * Utilizes OGRGeometry::Overlaps.
     */
    bool overlaps(const MultiPolygon& other) const;

    /**
     * Checks if 2 multipolygons overlap.
     * Utilizes OGRGeometry::Overlaps.
     */
    bool overlaps(const OGRMultiPolygon& other) const;

    /**
     * Checks if multipolygon overlaps with extents.
     * Converts extents to OGRGeometry::OGRPolygon and utilizes
     * OGRGeometry::Overlaps.
     */
    bool overlaps(const math::Extents2& other) const;

    /**
     * Checks if multipolygon overlaps with given polygon (is closed here).
     * Converts points to OGRGeometry::OGRPolygon. The first point is added
     * to the end to close the ring.
     * Utilizes OGRGeometry::Overlaps.
     */
    bool overlaps(const math::Points2& other) const;

    static MultiPolygon from(const math::Extents2& extents);

private:
    OGRMultiPolygon polygons_;
};

/**
 * @throws std::runtime_error if unable to open dataset
 * @return non-null ptr to GDALDataset
 */
GDALDatasetUniquePtr openDataset(const fs::path& path);

/**
 * Loads Multipolygon from OGR readable file.
 * Goes through all available layers and merges geometries into OGRMultiPolygon.
 * Supports: wkbPolygon, wkbPolygon25D, wkbTriangle, wkbMultiPolygon,
 * wkbMultiPolygon25D.
 * @param path to OGR readable file
 * @param targetSrs optionally convert geometries to target SRS. Layers need to
 * contain SRS definition. Only 2D is supported.
 * @return all supported geometries found in input file merged into
 * MultiPolygon.
 */
MultiPolygon loadPolygons(const fs::path& path,
                          boost::optional<SrsDefinition> targetSrs);

void convert(const CsConvertor& conv, OGRPolygon& polygon);

void convert(const CsConvertor& conv, OGRMultiPolygon& polygonMultiPolygon);

MultiPolygon toPolygons(const math::Extents2& extents);

} // namespace geo::ogr


#endif // VADSTENA_OGRPOLYGON_HPP
