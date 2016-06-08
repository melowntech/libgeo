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
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "geometry/mesh.hpp"
#include "imgproc/rastermask.hpp"
#include "utility/enum-io.hpp"

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <cpl_conv.h>

#include "./srsdef.hpp"
#include "./geotransform.hpp"

namespace ublas = boost::numeric::ublas;

namespace geo {

/** Color to burn into raster during rasterization.
 */
class BurnColor {
public:
    template<class ...Args>
    BurnColor(Args &&...args)
        : color_{std::forward<Args>(args)...}
    {}

    /** Returns list of bands.
     */
    std::vector<int> bandList() const;

    /** Returns list of values.
     */
    std::vector<double> valueList() const;

private:
    typedef boost::optional<double> Color;
    std::vector<Color> color_;
};

class GeoDataset {
public:
    typedef imgproc::quadtree::RasterMask Mask;

    typedef boost::optional<double> NodataValue;
    typedef boost::optional<NodataValue> OptionalNodataValue;

    static GeoDataset createFromFS(const boost::filesystem::path &path) {
        return open(path);
    }

    static GeoDataset open(const boost::filesystem::path &path);

    static GeoDataset use(std::unique_ptr<GDALDataset> &&dset);

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
        const math::Extents2 &extents,
        boost::optional<GDALDataType> dstDataTypeOverride = boost::none
        , OptionalNodataValue dstNodataValue = boost::none);

    /** Derive in-memory data set from an existing set, using a generic
     * affine geographic transformation in destination source reference system.
     * Only metadata from source are used.
     * 
     *  \param source reference data set
     *  \param srs new dataset spatial reference
     *  \param trafo affine transformation which determines how the 
     *      reference system maps to pixel/line coordinates. This is basis 
     *      for the new dataset's geotransform, but it expresses only 
     *      rotation and shear, not scale, hence determinant should always be 1.
     *  \param pixelSize pixel size of the new dataset (pixel and line),
     *      optinally calculated from the input image.
     *  \param extents srs extents to be fully inscribed within the new dataset,
     *      optionally calculated from the input image. If the trafo is 
     *      nonorthogonal, resultant extents will differ.
     */
    
    static GeoDataset deriveInMemory(
        const GeoDataset & source, const SrsDefinition & srs,
        boost::optional<math::Point2d> pixelSize,
        boost::optional<math::Extents2> extents,
        const math::Matrix2 & trafo = ublas::identity_matrix<double>(2),
        boost::optional<GDALDataType> dstDataTypeOverride = boost::none
        , OptionalNodataValue dstNodataValue = boost::none);
    
    /** Creates an invalid placeholder that can be used to hold valid dataset.
     *  Do not call any method on placeholder except:
     *      * move assignent operator
     *      * destructor
     *      * operator bool()
     */
    static GeoDataset placeholder();

    
    /** Format descriptor
     */
    struct Format {
        enum class Storage { custom, gtiff, png, jpeg, jp2, vrt, memory };

        /** Datatype (GDT_Byte, GDT_UInt16, ...)
         */
        ::GDALDataType channelType;

        /** Number and color interpretation of channels.
         */
        std::vector<GDALColorInterp> channels;

        /** Type of storage (tiff image, png image, ...)
         */
        Storage storageType;

        /** Valid only if storageType == Storage::custom
         */
        std::string driver;

        Format() : channelType(::GDT_Byte), storageType(Storage::custom) {}
        Format(::GDALDataType channelType
               , const std::initializer_list< ::GDALColorInterp> &channels
               , Storage storageType = Storage::custom)
            : channelType(channelType)
            , channels(channels.begin(), channels.end())
            , storageType(storageType)
        {}

        static Format gtiffRGBPhoto() {
            return { GDT_Byte, { GCI_RedBand, GCI_GreenBand, GCI_BlueBand }
                , Format::Storage::gtiff };
        }

        static Format pngRGBPhoto() {
            return { GDT_Byte, { GCI_RedBand, GCI_GreenBand, GCI_BlueBand
                                 , GCI_AlphaBand }
                , Format::Storage::png };
        }

        static Format pngRGBAPhoto() {
            return { GDT_Byte, { GCI_RedBand, GCI_GreenBand, GCI_BlueBand }
                , Format::Storage::png };
        }

        static Format jpegRGBPhoto() {
            return { GDT_Byte, { GCI_RedBand, GCI_GreenBand, GCI_BlueBand
                                 , GCI_AlphaBand }
                , Format::Storage::jpeg };
        }

        static Format coverage(Storage storage = Storage::gtiff) {
            return { GDT_Byte, { GCI_AlphaBand }, storage };
        }

        static Format dsm(Storage storage = Storage::gtiff) {
            return { GDT_Float32, { GCI_GrayIndex }, storage };
        }
    };

    typedef boost::optional<int> Overview;
    typedef boost::optional<Overview> OptionalOverview;

    /** Various options. Thin wrapper around vector of string pairs.
     *
     *  For create-options see:
     *    * http://www.gdal.org/frmt_gtiff.html for gtiff options
     *    * http://www.gdal.org/frmt_various.html#PNG for png options
     *    * http://www.gdal.org/frmt_jpeg.html for jpeg options
     */
    struct Options {
        typedef std::pair<std::string, std::string> Option;
        typedef std::vector<Option> OptionList;
        OptionList options;

        Options() = default;

        template <typename T>
        Options(const std::string &name, const T &value) {
            operator()(name, value);
        }

        template <typename T>
        Options operator()(const std::string &name, const T &value) {
            options.emplace_back
                (name, boost::lexical_cast<std::string>(value));
            return *this;
        }

        /** Special handling for boolean -> YES/NO
         */
        Options operator()(const std::string &name, bool value) {
            options.emplace_back(name, value ? "YES" : "NO");
            return *this;
        }
    };

    /** Creates new dataset at given path
     *
     *  \param path path to created file
     *  \param srs spatial reference system of new data set
     *  \param geoTransform geo (pixel to geo) transform
     *  \param rasterSize size of raster in pixels
     *  \param format format of data (channel type, number of channels
     *                type of data).
     *  \param noDataValue special value for no data
     */
    static GeoDataset create(const boost::filesystem::path &path
                             , const SrsDefinition &srs
                             , const GeoTransform & geoTransform
                             , const math::Size2 &rasterSize
                             , const Format &format
                             , const NodataValue &noDataValue = boost::none
                             , const Options &options = Options());

    /** Creates new dataset at given path.
     *
     *  \param path path to created file
     *  \param srs spatial reference system of new data set
     *  \param extents geo extents
     *  \param rasterSize size of raster in pixels
     *  \param format format of data (channel type, number of channels
     *                type of data).
     *  \param noDataValue special value for no data
     */
    static GeoDataset create(const boost::filesystem::path &path
                             , const SrsDefinition &srs
                             , const math::Extents2 &extents
                             , const math::Size2 &rasterSize
                             , const Format &format
                             , const NodataValue &noDataValue = boost::none
                             , const Options &options = Options());

    enum class Type { custom, grayscale, rgb, rgba, alpha };
    Type type() const { return type_; }

    enum Resampling {
        nearest, bilinear, cubic, cubicspline, lanczos, average, mode
        , minimum, maximum, median, q1, q3
        , texture // selects best resampling based on scale for textures
        , dem  // selects best resampling based on scale for dem
    };

    /** Information about warp operation.
     */
    struct WarpResultInfo {
        /** Index of used overview. Unset if dataset has been warped using
         *  original dataset.
         */
        Overview overview;

        /** Maximum scale between destination and source image
         *  > 1: upscale, < 1 downscale
         *  used to select overview
         */
        double truescale;

        /** Maximum scale between destination image and the physical image used
         *  (original or overview).
         *  > 1: upscale, < 1 downscale
         */
        double scale;

        /** Used resampling algorithm
         */
        Resampling resampling;
    };

    /** Warp options. Generic options + special stuff.
     */
    struct WarpOptions : Options {
    
        WarpOptions(): 
            warpMemoryLimit(0x10000000), // 256 MB
            safeChunks(true) {}

        template <typename T>
        WarpOptions(const std::string &name, const T &value) {
            operator()(name, value);
        }

        /** Src nodata override.
         */
        OptionalNodataValue srcNodataValue;

        /** Dst nodata override.
         */
        OptionalNodataValue dstNodataValue;

        /** Force overview:
         *  boost::none: auto
         *  { boost::none }: original dataset
         *  { int }: overview
         */
        OptionalOverview overview;

        /** Warp memory limit in bytes */       
        int warpMemoryLimit;
        
        /** Use overviews agressively to restrict memory usage (and prevent overflows).
          * There is some performance penalty when this option is set, resulting from  
          * the chunking code being run (at least) twice. */
        bool safeChunks; 
    };

    /** \brief Warp data from this dataset to destination dataset.
     *
     * If resampling algorithm is not set (i.e. boost::none) best algorithm is
     * selected automatically:
     *    * lanczos by default
     *
     * Futhermore, if resolutions are equal and distance between this and dst
     * extents are in whole pixels we proceed with pure pixel copy from src to
     * dst instead of more time consuming warp.
     *
     * \param dst destination dataset
     * \param alg resampling algorithm (i.e. filter to use)
     * \return info about processed warp operation
     */
    WarpResultInfo
    warpInto(GeoDataset &dst
             , const boost::optional<Resampling> &alg = boost::none
             , const WarpOptions &options = WarpOptions()) const;

    void expectRGB() const;
    void expectGray() const;
    void expectAlpha() const;

    /** Passes test if type is one of (grayscale, alpha)
     */
    void expectMask() const;

    void exportCvMat( cv::Mat & raster, int cvDataType );

    /**
     * @brief export geometry from a dsm dataset (in local coordinates)
     */
    void exportMesh( geometry::Mesh & mesh ) const;


    /**
    * @brief remove vertices and their faces where the dataset's data are missing  
    * @details Input mesh is defined in local coordinates for given extents
    * which may be different from the texture dataset extents.
    * Its texture information, if any, is ignored. The mesh geometry is not altered,
    */
    void filterMesh( const geometry::Mesh& mesh
                   , const math::Extents2 & extents
                   , geometry::Mesh& omesh ) const;

    math::Points3 exportPointCloud() const;

    /**
     * @brief texture a mesh using a texture dataset
     * @details Input mesh is defined in local coordinates for given extents
     * which may be different from the texture dataset extents Its texture
     * information, if any, is ignored. The mesh geometry is not altered,
     * but any faces which canont be textured are not copied into output.
     */
    void textureMesh(
        const geometry::Mesh & imesh,
        const math::Extents2 & extents,
        geometry::Mesh & omesh ) const;

    math::Size2i size() const { return size_; }

    /** Returns size of given overview.
     */
    math::Size2i size(const Overview &ovr) const;

    std::string srsWkt() const { return srsWkt_; }
    std::string srsProj4() const { return srsProj4_; }
    SrsDefinition srs() const {
        return SrsDefinition(srsWkt_, SrsDefinition::Type::wkt);
    }

    /**
     * @brief dataset extents, defined only for orthogonal datasets.
     */
    math::Extents2 extents() const;

    /**
     * @brief geoTransform (pixel to geo) transform. Opaque on this level.
     */
    GeoTransform geoTransform() const { return geoTransform_; }
    
    /** Data set resolution ("pixel size").
     */
    math::Point2 resolution() const;

    math::Point3 rowcol2geo( int row, int col, double value ) const {
        return geoTransform().rowcol2geo( row, col, value );
    }

    void geo2rowcol( const math::Point3 & gp, double & row, double & col ) const {
        return geoTransform().geo2rowcol( gp, row, col );
    }

    math::Point3 raster2geo( math::Point2 p, double value ) const {
        return geoTransform().raster2geo( p, value );
    }

    template <typename T>
    T geo2raster( const math::Point3 & gp) const {
        double x, y; geo2rowcol(gp, y, x); return T(x, y);
    }

    template <typename T>
    T geo2raster(double gx, double gy, double gz = .0) const {
        double x, y; geo2rowcol({gx, gy, gz}, y, x); return T(x, y);
    }

    template <typename T>
    T geo2raster(const math::Point2 &gp) const {
        double x, y; geo2rowcol({gp(0), gp(1), .0}, y, x); return T(x, y);
    }
    /* end obsolete functions */
    
    double geo2height(double gx, double gy, double gz = .0) const;

    
    math::Extents2 deriveExtents( const SrsDefinition &srs ) const;

    static std::string wktToProj4( const std::string & op );
    static std::string proj4ToWkt( const std::string & op );

    /** 
     * @brief obtain a longlat geo converter for dataset. 
     * @details converter allows conversions from pixel coordinates to
     * WGS84. */
    GeoConverter2 longlatConverter() const { return GeoConverter2( 
        geoTransform_, srs(), SrsDefinition::longlat() ); }

    /** Convert other's mask into this grid.
     */
    Mask convertMask(const geo::GeoDataset &other) const;

    /** Apply mask from another dataset over this dataset.
     *  Uses convertMask.
     */
    void applyMask(const GeoDataset &other);

    /** Applies mask over current mask. Called must ensure that mask gives
     *  sense.
     */
    void applyMask(const Mask &mask);

    void flush();

    /** Returns geo extents of given pixel in raster
     */
    math::Extents2 pixelExtents(const math::Point2i &raster) const;

    /** Returns geo extents of given pixel extents in raster
     */
    math::Extents2 pixelExtents(const math::Extents2i &raster) const;

    /** Get data for reading/writing.
     */
    cv::Mat& data(bool justData = false);

    /** Get data for reading.
     */
    const cv::Mat& data(bool justData = false) const;

    /** Get data for reading.
     */
    const cv::Mat& cdata(bool justData = false) const;

    /** Get mask for reading/writing.
     */
    Mask& mask(bool justMask = false);

    /** Get mask for reading.
     */
    const Mask& mask(bool justMask = false) const;

    /** Get mask for reading.
     */
    const Mask& cmask(bool justMask = false) const;

    /** Move ctor. Allows initialization from return value.
     */
    GeoDataset(GeoDataset &&other) noexcept
        : size_(other.size_)
        , srsWkt_(other.srsWkt_), srsProj4_(other.srsProj4_)
        , geoTransform_(std::move(other.geoTransform_))
        , dset_(std::move(other.dset_))
        , numChannels_(other.numChannels_)
        , channelMapping_(other.channelMapping_)
        , noDataValue_(other.noDataValue_)
        , changed_(other.changed_)
    {
        // we need to steal content from data and mask :)
        std::swap(data_, other.data_);
        std::swap(mask_, other.mask_);
    }

    /** Move assignment. Allows overwrite from return value.
     */
    GeoDataset& operator=(GeoDataset &&other) noexcept
    {
        size_ = other.size_;
        srsWkt_ = other.srsWkt_; srsProj4_ = other.srsProj4_;
        geoTransform_ = std::move(other.geoTransform_);
        dset_ = std::move(other.dset_);
        numChannels_ = other.numChannels_;
        channelMapping_ = other.channelMapping_;
        noDataValue_ = other.noDataValue_;
        changed_ = other.changed_;

        // we need to steal content from data and mask :)
        std::swap(data_, other.data_);
        std::swap(mask_, other.mask_);
        return *this;
    }

    ~GeoDataset() noexcept;

    class Metadata {
    public:
        typedef std::map<std::string, std::string> Items;

        Metadata() {}

        template <typename T>
        Metadata(const std::string &name, const T &value) {
            operator()(name, value);
        }

        template <typename T>
        Metadata& operator()(const std::string &name, const T &value) {
            items_[name] = boost::lexical_cast<std::string>(value);
            return *this;
        }

        Metadata& operator()(const std::string &name, const std::string &value)
        {
            items_[name] = value;
            return *this;
        }

        template <typename T>
        boost::optional<T> get(const std::string &name) const
        {
            auto fitems(items_.find(name));
            if (fitems == items_.end()) { return boost::none; }
            return boost::lexical_cast<T>(fitems->second);
        }

        const Items& items() const { return items_; }

    private:
        Items items_;
    };

    /** Get dataset's metadata.
     */
    Metadata getMetadata(const std::string &domain = "") const;

    /** Set dataset's metadata.
     */
    void setMetadata(const Metadata &metadata
                     , const std::string &domain = "");

    /** Get band's metadata. NB: Bands are zero based.
     */
    Metadata getMetadata(int band, const std::string &domain = "") const;

    /** Set band's metadata. NB: Bands are zero based.
     */
    void setMetadata(int band, const Metadata &metadata
                     , const std::string &domain = "");

    std::size_t bandCount() const;

    struct BandProperties {
        ::GDALDataType dataType;
        :: GDALColorInterp colorInterpretation;
        math::Size2 size;
        math::Size2 blockSize;
    };

    /** Get band propeties. NB: Bands are zero based.
     */
    BandProperties bandProperties(int band) const;

    // rawish data interface

    /** IO block
     */
    struct Block {
        /** Block data;
         */
        cv::Mat data;

        Block() = default;

        operator bool() const { return data.data; }
    };

    /** Reads block that contains given point.
     */
    Block readBlock(const math::Point2i &blockOffset) const;

    /** Size of IO block.
     */
    math::Size2 blockSize() const;

    /** Transforms pixel position to block offset and position inside block
     */
    std::tuple<math::Point2i, math::Point2i>
    blockCoord(const math::Point2i &point) const;

    /** Checks whether this dataset is valid dataset (i.e. is backed by valid
     *  GDAL dataset. Returns false for placeholder.
     */
    operator bool() const { return dset_.get(); }

    double noDataValue(double defaultValue) const {
        return noDataValue_ ? *noDataValue_ : defaultValue;
    }

    NodataValue rawNodataValue() const { return noDataValue_; }

    /** returns true if dataset is orthogonal */
    bool isOrthogonal() const;

    /** returns geographic coordinates of origin */
    math::Point2 origin() const {
        return math::Point2( geoTransform_[0], geoTransform_[3] ); }

    /** Says whether all pixels in this datasets are valid.
     */
    bool allValid() const;

    /** Rasterize single of OGR geometry into this dataset.
     */
    void rasterize(const ::OGRGeometry *geometry, const BurnColor &color);

    /** Fetches mask as single-channel byte matrix.
     *  NB: result is NOT cached.
     *
     *  User internally by loadMask which converts it to rastermask (which is
     *  cached)
     *
     * \param optimized return invalid matrix (i.e. data is null) when all
     *        pixels in the dataset are valid
     *
     */
    cv::Mat fetchMask(bool optimized = false) const;

    Format getFormat() const;

private:
    static bool initialized_;
    static void initialize();

    GeoDataset(const GeoDataset &other) = delete;

    GeoDataset& operator=(const GeoDataset &other) = delete;

    GeoDataset(std::unique_ptr<GDALDataset> &&dset
               , bool freshlyCreated = false);

    struct DataFlag {
        enum {
            data = 0x1
            , mask = 0x2
            , all = data | mask
        };
    };
    void assertData(int what = DataFlag::all) const;
    void loadData() const;
    void loadMask() const;

    bool valid( int i, int j ) const;
    bool validf( double i, double j ) const;

    Type type_;
    math::Size2i size_;
    std::string srsWkt_, srsProj4_;
    GeoTransform geoTransform_;
    std::unique_ptr<GDALDataset> dset_;
    int numChannels_;
    /** Read/write channel mapping: maps GDAL band number to opencv channel. */
    std::vector<int> channelMapping_;
    NodataValue noDataValue_;
    mutable boost::optional<cv::Mat> data_;
    mutable boost::optional<Mask> mask_;

    bool changed_;
};

// inline method implementation

inline math::Point2 GeoDataset::resolution() const
{
    return math::Point2(
        sqrt( math::sqr( geoTransform_[1] ) + math::sqr( geoTransform_[4] ) ),                        
        sqrt( math::sqr( geoTransform_[2] ) + math::sqr( geoTransform_[5] ) ) );
}

// enum i/o mapping
UTILITY_GENERATE_ENUM_IO(GeoDataset::Resampling,
                         ((nearest))
                         ((bilinear))
                         ((cubic))
                         ((cubicspline))
                         ((lanczos))
                         ((average))
                         ((mode))
                         ((minimum))
                         ((maximum))
                         ((median))
                         ((q1))
                         ((q3))
                         ((texture))
                         ((dem))
                         )


inline cv::Mat& GeoDataset::data(bool justData)
{
    assertData(justData ? DataFlag::data : DataFlag::all);
    changed_ = true;
    return *data_;
}

inline const cv::Mat& GeoDataset::data(bool justData) const
{
    return cdata(justData);
}

inline const cv::Mat& GeoDataset::cdata(bool justData) const
{
    assertData(justData ? DataFlag::data : DataFlag::all);
    return *data_;
}

inline GeoDataset::Mask& GeoDataset::mask(bool justMask)
{
    assertData(justMask ? DataFlag::mask : DataFlag::all);
    changed_ = true;
    return *mask_;
}

inline const GeoDataset::Mask& GeoDataset::mask(bool justMask) const
{
    return cmask(justMask);
}

inline const GeoDataset::Mask& GeoDataset::cmask(bool justMask) const
{
    assertData(justMask ? DataFlag::mask : DataFlag::all);
    return *mask_;
}

} // namespace geo

#endif // geo_geodataset_hpp_included
