/**
  * @file geodataset.hpp
  * @author Ondrej Prochazka <ondrej.prochazka@citationtech.net>
  *
  * Access layer for georeferenced datasets, built atop the GDAL and OGR
  * libraries.
  */

#ifndef geo_geodataset_hpp_included
#define geo_geodataset_hpp_included

#include <memory>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include "math/geometry.hpp"
#include "geometry/mesh.hpp"
#include "imgproc/rastermask.hpp"

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <cpl_conv.h>

#include "./srsdef.hpp"

namespace geo {

class GeoDataset {
public:

    static GeoDataset createFromFS(const boost::filesystem::path &path);

    /** Derive in-memory data set from existing data set.
     *  Only metadata from source are used.
     *
     *  \param source reference data set
     *  \param srs spatial reference of created data set
     *  \param size pixel dimension of new data set
     *              (calculated from source and extents if boost::none)
     *  \param extents new data set extents
     *  \return new (empty) data set
     */
    static GeoDataset deriveInMemory(
        const GeoDataset & source, const SrsDefinition &srs,
        boost::optional<math::Size2i> size,
        const math::Extents2 &extents );

    enum Resampling { lanczos, average };

    void warpInto( GeoDataset & dst, Resampling alg = lanczos ) const;

    void expectRGB() const;
    void expectGray() const;

    void exportCvMat( cv::Mat & raster, int cvDataType );

    /**
     * @brief export geometry from a dsm dataset (in local coordinates)
     */
    void exportMesh( geometry::Mesh & mesh ) const;

    math::Points3 exportPointCloud() const;

    /**
     * @brief texture a mesh using a texture dataset
     * @details Input mesh is defined in local coordinates for given extents
     * which may be different from the texture datset extents Its texture
     * information, if any, is ignored. The mesh geometry is not altered,
     * but any faces which canont be textured are not copied into output.
     */
    void textureMesh(
        const geometry::Mesh & imesh,
        const math::Extents2 & extents,
        geometry::Mesh & omesh ) const;

    math::Size2i size() const { return size_; }

    std::string srsWkt() const { return srsWkt_; }
    std::string srsProj4() const { return srsProj4_; }
    SrsDefinition srs() const { return SrsDefinition(srsProj4_); }

    math::Extents2 extents() const { return extents_; };

    /** Data set resolution ("pixel size").
     */
    math::Point2 resolution() const;

    math::Point3 rowcol2geo( int row, int col, double value ) const;
    void geo2rowcol( const math::Point3 & gp, double & row, double & col ) const;

    math::Extents2 deriveExtents( const SrsDefinition &srs );

    static std::string wktToProj4( const std::string & op );
    static std::string proj4ToWkt( const std::string & op );

private:
    static bool initialized_;
    static void initialize();

    static math::Point2 applyGeoTransform(
        const double * trafo, double col, double row );
    static void applyInvGeoTransform(
        const double * trafo, const math::Point2 & gp,
        double & col, double & row );

    GeoDataset( GDALDataset * dset );

    void assertData() const;
    void loadData() const;

    bool valid( int i, int j ) const;
    bool validf( double i, double j ) const;

    math::Size2i size_;
    math::Extents2 extents_;
    std::string srsWkt_, srsProj4_;
    double geoTransform_[6];
    std::shared_ptr<GDALDataset> dset_;
    int numChannels_;
    boost::optional<double> noDataValue_;
    mutable boost::optional<cv::Mat> data_;
    mutable boost::optional<imgproc::quadtree::RasterMask> mask_;
};

// inline method implementation

inline math::Point2 GeoDataset::resolution() const
{
    auto esize(math::size(extents_));
    return math::Point2(esize.width / size_.width
                        , esize.height / size_.height);
}

} // namespace geo

#endif // geo_geodataset_hpp_included
