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
/*
 * @file geodataset.cpp
 */

#include <cassert>
#include <sstream>
#include <numeric>
#include <limits>

#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <gdalwarper.h>

#include "utility/openmp.hpp"
#include "utility/expect.hpp"
#include "utility/path.hpp"
#include "math/math.hpp"
#include "geometry/polygon.hpp"

#include "imgproc/rastermask/transform.hpp"

#include "./geodataset.hpp"
#include "./coordinates.hpp"
#include "./csconvertor.hpp"
#include "./io.hpp"
#include "./detail/warpmemorymeter.hpp"
#include "./gdal.hpp"


extern "C" {
void GDALErrorHandler( CPLErr eErrClass, int err_no, const char *msg)
{
    switch ( eErrClass ) {
    case CE_Debug:
        LOG( debug ) << "gdal error " << err_no << ": " << msg;
        break;

    case CE_Warning:
        LOG( warn2 ) << "gdal error " << err_no << ": " << msg;
        break;

    case CE_Failure:
        LOG( err2 ) << "gdal error " << err_no << ": " << msg;
        break;

    case CE_Fatal:
        LOG( fatal ) << "gdal error " << err_no << ": " << msg;
        break;

    default:
        break;
    }
}

} // extern "C"

namespace geo {

namespace ut = utility;
namespace fs = boost::filesystem;

typedef imgproc::quadtree::RasterMask RasterMask;

namespace {

/**Get value of GEO_DUMP_GDALWARP environmental variable before main is run.
 *
 * Nonset or empty value is considered as false, anything else is true.
 */
const bool GEO_DUMP_GDALWARP([]() -> bool
{
    const char *value(std::getenv("GEO_DUMP_GDALWARP"));
    if (!value || !*value) { return false; }
    return true;
}());

int gdal2cv(GDALDataType gdalDataType, int numChannels)
{
    // determine output datatype automatically
    switch (gdalDataType) {
    case GDT_Byte:    return CV_8UC(numChannels);
    case GDT_UInt16:  return CV_16UC(numChannels);
    case GDT_Int16:   return CV_16SC(numChannels);
    case GDT_UInt32:  return CV_32SC(numChannels);
    case GDT_Int32:   return CV_32SC(numChannels);
    case GDT_Float32: return CV_32FC(numChannels);
    case GDT_Float64: return CV_64FC(numChannels);

    default:
        LOGTHROW(err2, std::logic_error)
            << "Unsupported datatype " << gdalDataType << " in raster.";
    }
    return 0; // never reached
}

::GDALDataType cv2gdal(int cvDataType)
{
    // determine output datatype automatically
    switch (cvDataType) {
    case CV_8UC1:  return GDT_Byte;
    case CV_16UC1: return GDT_UInt16;
    case CV_16SC1: return GDT_Int16;
    case CV_32SC1: return GDT_UInt32;
    case CV_32FC1: return GDT_Float32;
    case CV_64FC1: return GDT_Float64;

    default:
        LOGTHROW(err2, std::logic_error)
            << "Unsupported datatype " << cvDataType << " in raster.";
    }
    return GDT_Byte; // never reached
}


bool areIdenticalButForShift( const GeoTransform & trafo1,
                              const GeoTransform & trafo2 )
{
    return(
        trafo1[1] == trafo2[1] &&
        trafo1[2] == trafo2[2] &&
        trafo1[4] == trafo2[4] &&
        trafo1[5] == trafo2[5]
    );
}

// options support

class OptionsWrapper : boost::noncopyable {
public:
    OptionsWrapper(const GeoDataset::Options &options)
        : opts_(nullptr)
    {
        for (const auto &opt : options.options) {
            opts_ = ::CSLSetNameValue(opts_, opt.first.c_str()
                                      , opt.second.c_str());
        }
    }

    ~OptionsWrapper() { ::CSLDestroy(opts_); }

    operator char**() const { return opts_; }

    char** release() {
        char** opts(opts_);
        opts_ = nullptr;
        return opts;
    }

    OptionsWrapper& operator()(const char *name, const char *value) {
        opts_ = ::CSLSetNameValue(opts_, name, value);
        return *this;
    }

    template <typename T>
    OptionsWrapper& operator()(const char *name, const T &value) {
        return operator()
            (name, boost::lexical_cast<std::string>(value).c_str());
    }

    OptionsWrapper& operator()(const char *name, bool value) {
        return operator()(name, value ? "YES" : "NO");
    }

private:
    char **opts_;
};

} // namespace

/* class GeoDataset */

bool GeoDataset::initialized_ = false;



std::string GeoDataset::wktToProj4( const std::string & op )
{
    return SrsDefinition(op, SrsDefinition::Type::wkt)
        .as(SrsDefinition::Type::proj4).srs;
}


std::string GeoDataset::proj4ToWkt( const std::string & op )
{
    return SrsDefinition(op, SrsDefinition::Type::proj4)
        .as(SrsDefinition::Type::wkt).srs;
}

GeoDataset GeoDataset::placeholder()
{
    return GeoDataset({}, false);
}

GeoDataset GeoDataset::use(std::unique_ptr<GDALDataset> &&dset)
{
    return GeoDataset(std::move(dset));
}

GeoDataset::GeoDataset(std::unique_ptr<GDALDataset> &&dset
                       , bool freshlyCreated)
    : type_(Type::custom), dset_(std::move(dset)), changed_(false)
    , fresh_(freshlyCreated)
{
    // stop here if invalid
    if (!dset_) { return; }

    // size & extents
    dset_->GetGeoTransform( geoTransform_.data() );

    size_ = math::Size2i( dset_->GetRasterXSize(), dset_->GetRasterYSize() );

    // srs
    srsWkt_ = std::string( dset_->GetProjectionRef() );
    srsProj4_ = wktToProj4( srsWkt_ );

    // noDataValue
    if ( ( numChannels_ = dset_->GetRasterCount() ) > 0 ) {

        int success;
        double ndv = dset_->GetRasterBand(1)->GetNoDataValue( & success );

        if ( success ) {
            noDataValue_ = ndv;
        }

        // initialize read/write mapping with identity
        channelMapping_.resize(numChannels_ + 1);
        std::iota(channelMapping_.begin() + 1, channelMapping_.end(), 0);

        if ((numChannels_ == 3) || (numChannels_ == 4)) {
            // calculate collor mapping (cv::Mat for images has BGR or BGRA
            // layout)
            int red(0), green(0), blue(0), alpha(0);

            for (int i(1); i <= numChannels_; ++i) {
                switch (dset_->GetRasterBand(i)->GetColorInterpretation()) {
                case GCI_RedBand: red = i; break;
                case GCI_GreenBand: green = i; break;
                case GCI_BlueBand: blue = i; break;
                case GCI_AlphaBand: alpha = i; break;
                default: break;
                }
            }

            if (((numChannels_ == 3) && red && green && blue)
                || ((numChannels_ == 4) && red && green && blue && alpha))
            {
                // RGB(A) image, use custom mapping

                channelMapping_[blue] = 0;
                channelMapping_[green] = 1;
                channelMapping_[red] = 2;
                if (alpha) {
                    channelMapping_[alpha] = 3;
                    type_ = Type::rgba;
                } else {
                    type_ = Type::rgb;
                }
            }
        } else if (numChannels_ == 1) {
            // check for grayscale/alpha
            switch (dset_->GetRasterBand(1)->GetColorInterpretation()) {
            case GCI_GrayIndex: type_ = Type::grayscale; break;
            case GCI_AlphaBand: type_ = Type::alpha; break;
            default: break;
            }
        }
    }
}

GeoDataset::~GeoDataset() noexcept
{
    // valid dataset that has not been flushed and no exception is thrown ->
    // someone forgot to flush :)
    if (dset_ && changed_ && !std::uncaught_exception()) {
        LOG(warn2) << "GeoDataset was changed but not flushed!";
    }
}

void GeoDataset::initialize() {
    CPLSetErrorHandler( GDALErrorHandler );
    GDALAllRegister();


    initialized_ = true;
}


GeoDataset GeoDataset::open(const fs::path &path, const Options&)
{

    if ( ! initialized_ ) initialize();

    std::unique_ptr<GDALDataset> dset
        (static_cast<GDALDataset*>
         (GDALOpen( path.string().c_str(), GA_ReadOnly)));

    if ( !dset ) {
        LOGTHROW( err2, std::runtime_error )
            << "Failed to open dataset " << path << ".";
    }

    return GeoDataset( std::move(dset) );
}


GeoDataset GeoDataset::deriveInMemory(
        const GeoDataset & source, const SrsDefinition & srs,
        boost::optional<math::Point2d> pixelSize,
        boost::optional<math::Extents2> extents,
        const math::Matrix2 & trafo,
        boost::optional<GDALDataType> dstDataTypeOverride
        , OptionalNodataValue dstNodataValueOverride)
{

    // source dataset handle
    GDALDataset * sdset( const_cast<GDALDataset *>( source.dset_.get() ) );

    // obtain driver
    GDALDriver * driver = GetGDALDriverManager()->GetDriverByName( "MEM" ); //"GTiff" );

    ut::expect( driver, "Failed to initialize in memory driver." );

    ut::expect( source.dset_->GetRasterCount() >= 1,
        "No bands exist in input dataset." );

    // use source's SRS if same (this prevents conversion through lat/lon if
    // GDAL thinks that these are not the same)
    std::string srsWkt(
        areSame(srs, SrsDefinition(source.srsWkt_, SrsDefinition::Type::wkt))
         ? source.srsWkt_
         : srs.as(SrsDefinition::Type::wkt).srs );
    
    // establish image size and geotransform
    GeoTransform geoTransform;
    math::Size2i size;
    
    math::Point2d lpixelSize( source.resolution() );
    if ( pixelSize ) lpixelSize = pixelSize.get();
                                 
    math::Extents2 lextents;
    
    if ( extents ) 
        lextents = extents.get();
    else
        lextents = source.deriveExtents( srs );
    
    math::Matrix3 igtrafo = ublas::identity_matrix<double>(3);
    subrange( igtrafo, 0, 2, 0, 2 ) = trafo;
    
    math::Matrix3 lt, rt;
    lt = rt = ublas::identity_matrix<double>(3);
    lt(0,2) = lextents.ll(0); lt(1,2) = lextents.ll(1);
    rt(0,2) = - lextents.ll(0); rt(1,2) = - lextents.ll(1);
    igtrafo = prod( igtrafo, rt ); igtrafo = prod( lt, igtrafo );
    
    //LOG(debug) << igtrafo;
    
    math::Point2 ll, lr, ul, ur;
    
    ll = subrange( prod( igtrafo,  math::Point3( 
        lextents.ll[0], lextents.ll[1], 1 ) ), 0, 2 );
    lr = subrange( prod( igtrafo, math::Point3( 
        lextents.ur[0], lextents.ll[1], 1 ) ), 0, 2 ); 
    ul = subrange( prod( igtrafo, math::Point3( 
        lextents.ll[0], lextents.ur[1], 1 ) ), 0, 2 ); 
    ur = subrange( prod( igtrafo,  math::Point3( 
        lextents.ur[0], lextents.ur[1], 1 ) ), 0, 2 ); 
    
    math::Extents2 textents;
    
    textents.ll[0] = std::min( { ll[0], lr[0], ul[0], ur[0] } );
    textents.ll[1] = std::min( { ll[1], lr[1], ul[1], ur[1] } );
    textents.ur[0] = std::max( { ll[0], lr[0], ul[0], ur[0] } );
    textents.ur[1] = std::max( { ll[1], lr[1], ul[1], ur[1] } );

    math::Matrix3 istrafo = ublas::identity_matrix<double>(3);
    
    size.width = std::round(( textents.ur[0] - textents.ll[0] ) / lpixelSize(0));
    size.height = std::round(( textents.ur[1] - textents.ll[1] ) / lpixelSize(1));
    
    istrafo(0,0) = 1.0 / lpixelSize(0); istrafo(1,1) = - 1.0 / lpixelSize(1);
    math::Point2 offset = - subrange( prod( istrafo, 
        math::Point3( textents.ll[0], textents.ur[1], 1.0 ) ), 0 , 2 );
    
    istrafo(0,2) = offset(0); istrafo(1,2) = offset(1);
    
    //LOG( debug) << istrafo;
    
    igtrafo = prod( istrafo, igtrafo );
    
    //LOG( debug ) << igtrafo;
    
    math::Matrix3 gtrafo = math::matrixInvert( igtrafo );

    //LOG( debug ) << gtrafo;
    
    geoTransform[0] = gtrafo(0,2);
    geoTransform[1] = gtrafo(0,0);
    geoTransform[2] = gtrafo(0,1);
    geoTransform[3] = gtrafo(1,2);
    geoTransform[4] = gtrafo(1,0);
    geoTransform[5] = gtrafo(1,1);

    // determine datatypes
    GDALDataType srcDataType, dstDataType( GDT_Byte );
    NodataValue dstNoDataValue( 0.0 );

    srcDataType = sdset->GetRasterBand(1)->GetRasterDataType();

    if ( source.noDataValue_ ) {
        dstNoDataValue = source.noDataValue_.get();
        dstDataType = srcDataType;
    }

    if ( ! source.noDataValue_ ) switch( srcDataType ) {

        case GDT_Byte :
            dstDataType = GDT_Int16; dstNoDataValue = -1; break;

        case GDT_UInt16 :
            dstDataType = GDT_Int32; dstNoDataValue = -1; break;

        case GDT_Int16 :
            dstDataType = GDT_Int32; dstNoDataValue = 1 << 16; break;

        case GDT_Float32:
        case GDT_Float64:
            // float/double -> double, and use insanely large negative number
            dstDataType = GDT_Float64;
            dstNoDataValue = std::numeric_limits<double>::lowest();
            break;

        default :
            LOGTHROW( err2, std::runtime_error )
                << "A no data value is compulsory for dataset of type "
                << srcDataType << ". Please patch your data.";
    }

    // override dstDatatype if requested
    if (dstDataTypeOverride) {
        dstDataType = *dstDataTypeOverride;

        if ( ! source.noDataValue_ ) switch( dstDataType ) {

            case GDT_Byte :
                dstNoDataValue = 0; break;

            case GDT_UInt16 :
                dstNoDataValue = 0; break;

            case GDT_Int16 :
                dstNoDataValue = -1; break;

            case GDT_Int32:
                dstNoDataValue = 1 << 16; break;

            case GDT_Float32 :
                dstNoDataValue = 1.0E15; break;

            case GDT_Float64:
                dstNoDataValue = 1.0E40; break;


            default :
                LOGTHROW( err2, std::runtime_error )
                    << "A no data value is compulsory for source dataset "
                    << "when override type is" << dstDataType
                    << ". Please patch your data.";
        }
    }

    if (dstNodataValueOverride) {
        dstNoDataValue = *dstNodataValueOverride;
    }

    // create dataset
    std::unique_ptr<GDALDataset> tdset(driver->Create(
        "MEM", //"MEM.tif",
        size.width, size.height,
        sdset->GetRasterCount(), dstDataType, nullptr ));

    ut::expect( tdset.get(), "Failed to create in memory dataset.\n" );

    for ( int i = 1; i <= tdset->GetRasterCount(); i++ ) {

        tdset->GetRasterBand(i)->SetColorInterpretation(
            sdset->GetRasterBand(i)->GetColorInterpretation() );
        if (dstNoDataValue) {
            tdset->GetRasterBand(i)->SetNoDataValue(*dstNoDataValue);
        }
    }

    tdset->SetGeoTransform( geoTransform.data() );
    tdset->SetProjection( srsWkt.c_str() );

    // all done
    return GeoDataset(std::move(tdset));
}


GeoDataset GeoDataset::deriveInMemory(
        const GeoDataset & source, const SrsDefinition &srs,
        boost::optional<math::Size2i> size,
        const math::Extents2 &extents,
        boost::optional<GDALDataType> dstDataTypeOverride
        , OptionalNodataValue dstNodataValue)
{
    if (!size) {
        auto esize(math::size(extents));
        auto res(source.resolution());
        size = boost::in_place(esize.width / res(0), esize.height / res(1));
    }

    return deriveInMemory( source, srs, math::Point2(
                            ( extents.ur[0] - extents.ll[0] ) / size->width,
                            ( extents.ur[1] - extents.ll[1] ) / size->height ),
                           extents, ublas::identity_matrix<double>(2),
                           dstDataTypeOverride, dstNodataValue);
}

GeoDataset GeoDataset::create(const boost::filesystem::path &path
                             , const SrsDefinition &srs
                             , const GeoTransform & geoTransform
                             , const math::Size2 &rasterSize
                             , const Format &format
                             , const NodataValue &noDataValue
                             , const Options &options )
{
    if (!initialized_) { initialize(); }

    // driver name to use
    std::string storageFormat;
    // world file extension: both world and prj files are not written when empty
    std::string wfExt;
    // path passed to driver
    boost::filesystem::path usePath(path);

    switch (format.storageType) {
    case Format::Storage::gtiff:
        storageFormat = "GTiff";
        // do not write world file since GeoTiff contains everything
        wfExt = "";
        break;

    case Format::Storage::png:
        storageFormat = "PNG";
        wfExt = "pgw";
        break;

    case Format::Storage::jpeg:
        storageFormat = "JPEG";
        wfExt = "jgw";
        break;

    case Format::Storage::vrt:
        storageFormat = "VRT";
        wfExt = "";
        break;

    case Format::Storage::memory:
        storageFormat = "MEM";
        usePath = "MEM";
        wfExt = "";
        break;

    case Format::Storage::custom:
        storageFormat = format.driver;
        wfExt = "";
        break;
    }

    auto driver(::GetGDALDriverManager()
                ->GetDriverByName(storageFormat.c_str()));
    if (!driver) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot find GDAL driver for <" << storageFormat
            << "> format.";
    }

    auto metadata(driver->GetMetadata());
    if (!CSLFetchBoolean(metadata, GDAL_DCAP_CREATE, FALSE)) {
        LOGTHROW(err2, std::runtime_error)
            << "GDAL driver for <" << storageFormat
            << "> format doesn't support creation.";
    }

    /* create parent directory for output dataset */ {
        auto pp(usePath.parent_path());
        if (!pp.empty()) {
            create_directories(pp);
        }
    }

    std::unique_ptr<GDALDataset>
        ds(driver->Create(usePath.string().c_str()
                          , rasterSize.width
                          , rasterSize.height
                          , format.channels.size()
                          , format.channelType
                          , OptionsWrapper(options)));

    ut::expect(ds.get(), "Failed to create new dataset.");

    // set projection
    if (ds->SetProjection(srs.as(SrsDefinition::Type::wkt).c_str())
        != CE_None)
    {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to set projection to newly created GDAL data set.";
    }

    // set geo trafo
    auto geoTrafo( geoTransform );

    if (ds->SetGeoTransform(geoTrafo.data()) != CE_None) {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to set geo transform to newly created GDAL data set.";
    }

    // set no data value and color interpretation
    {
        int i = 1;
        for (auto colorInterpretation : format.channels) {
            auto band(ds->GetRasterBand(i));
            ut::expect(band, "Cannot find band %d.", (i));
            if (noDataValue) {
                band->SetNoDataValue(*noDataValue);
            }
            band->SetColorInterpretation(colorInterpretation);
            ++i;
        }
    }

    if (!wfExt.empty()) {
        // write .prj file
        geo::writeSrs(utility::replaceOrAddExtension(path, "prj"), srs);

        // write world file
        geo::writeTfwFromGdal(utility::replaceOrAddExtension(path, wfExt)
                              , geoTrafo);
    }

    // OK, tell ctor that dset was freshly created
    return GeoDataset(std::move(ds), true);
}



GeoDataset GeoDataset::create(const boost::filesystem::path &path
                              , const SrsDefinition &srs
                              , const math::Extents2 &extents
                              , const math::Size2 &rasterSize
                              , const Format &format
                              , const NodataValue &noDataValue
                              , const Options &options) {

    auto geoTrafo( GeoTransform::northUpFromExtents(extents, rasterSize) );
    
    return create( path, srs, geoTrafo, rasterSize, format, noDataValue, 
                   options );
}

namespace {

GDALResampleAlg algoToGdal(GeoDataset::Resampling alg)
{
    switch (alg) {
    case GeoDataset::Resampling::nearest: return GRA_NearestNeighbour;
    case GeoDataset::Resampling::bilinear: return GRA_Bilinear;
    case GeoDataset::Resampling::cubic: return GRA_Cubic;
    case GeoDataset::Resampling::cubicspline: return GRA_CubicSpline;
    case GeoDataset::Resampling::lanczos: return GRA_Lanczos;
    case GeoDataset::Resampling::average: return GRA_Average;
    case GeoDataset::Resampling::mode: return GRA_Mode;
    case GeoDataset::Resampling::minimum: return GRA_Min;
    case GeoDataset::Resampling::maximum: return GRA_Max;
    case GeoDataset::Resampling::median: return GRA_Med;
    case GeoDataset::Resampling::q1: return GRA_Q1;
    case GeoDataset::Resampling::q3: return GRA_Q3;

    // these are just placeholders
    case GeoDataset::Resampling::texture: return GRA_Lanczos;
    case GeoDataset::Resampling::dem: return GRA_Lanczos;
    }

    // falback
    return GRA_Lanczos;
}

double sourcePixelSize(const geo::CsConvertor &conv
                       , const geo::GeoDataset &src
                       , const geo::GeoDataset &dst
                       , const math::Point2 &srcScale = math::Point2(1.0, 1.0))
{
    // two points in the center of dst raster one pixel apart:
    const auto dstSize(dst.size());
    const math::Point2 dstR1(dstSize.width / 2. - .5, dstSize.height / 2.);
    const math::Point2 dstR2(dstSize.width / 2. + .5, dstSize.height / 2.);

    // convert them to geo coordinates
    const auto dstG1(dst.raster2geo(dstR1, .0));
    const auto dstG2(dst.raster2geo(dstR2, .0));

    // now, convert them to source ref system
    const auto srcG1(conv(dstG1));
    const auto srcG2(conv(dstG2));

    // and finally convert them to src raster
    auto srcR1(src.geo2raster<math::Point2>(srcG1));
    auto srcR2(src.geo2raster<math::Point2>(srcG2));
    srcR2(0) *= srcScale(0); srcR2(1) *= srcScale(1);
    srcR1(0) *= srcScale(0); srcR1(1) *= srcScale(1);

    // calculate (scaled) source distance -> size of source pixel
    return boost::numeric::ublas::norm_2(srcR2 - srcR1);
}

double sourcePixelSize(const geo::GeoDataset &src
                       , const geo::GeoDataset &dst
                       , const math::Point2 &srcScale = math::Point2(1.0, 1.0))
{
    return sourcePixelSize(geo::CsConvertor(dst.srs(), src.srs())
                           , src, dst, srcScale);
}

template <typename T>
inline char cpSign(const math::Point2_<T> &prev
                   , const math::Point2_<T> &cur
                   , const math::Point2_<T> &next)
{
    math::Point2_<T> v1(cur - prev);
    math::Point2_<T> v2(next - cur);
    auto cp(math::crossProduct(normalize(v1), normalize(v2)));

    if (cp > 0.1) {
        return 1;
    } else if (cp < -0.1) {
        return 2;
    }

    return 0;
}

/** True only when polygon is right rotating and has sane shape (i.e. not
 *  flattened to (almost) line)
 */
bool rightRotating(const math::Points2 &polygon, bool yGrowsUp = true)
{
    if (polygon.size() < 3) {
        // this is not a polygon
        return false;
    }

    char sign(0);

    // first, special cases
    sign = cpSign(polygon.back(), polygon.front(), polygon[1]);
    if (!sign) { return false; }

    sign |= cpSign(polygon[polygon.size() - 2], polygon.back()
                   , polygon.front());
    // check for signs
    if (!sign || (sign == 3)) { return false; }

    // rest of polygon
    for (auto ip(polygon.begin() + 1), ep(polygon.end() - 1); ip != ep; ++ip) {
        auto s(cpSign(*(ip - 1), *ip, *(ip + 1)));
        if (!s) { return false; }
        sign |= s;
        if (sign == 3) { return false; }
    }

    return (yGrowsUp ? (sign == 2) : (sign == 1));
}

template <int count>
struct Corners {
    std::array<double, count> x;
    std::array<double, count> y;
    std::array<double, count> z;
    std::array<int, count> success;

    void add(int i, double xx, double yy) {
        x[i] = xx;
        y[i] = yy;
        z[i] = 0.0;
    }

    constexpr int size() const { return count; }

    bool allValid() const {
        auto valid(std::accumulate(success.begin(), success.end()
                                   , 0
                                   , [&](int out, int in) -> int {
                                       return out + in;
                                   }));
        return (valid == count);
    }

    math::Points2 asPoints() const {
        math::Points2 out;
        for (int i(0); i < 4; ++i) {
            out.emplace_back(x[i], y[i]);
        }
        return out;
    }
};

void sourceExtra(const GeoDataset &src, const GeoDataset &dst
                 , OptionsWrapper &wo, const GDALWarpOptions *warpOptions)
{
    auto size(dst.size());

    Corners<4> corners;

    corners.add(0, 0.0, 0.0);
    corners.add(1, size.width, 0.0);
    corners.add(2, size.width, size.height);
    corners.add(3, 0.0, size.height);

    // try to transform points
    warpOptions->pfnTransformer(warpOptions->pTransformerArg, true, 4
                                , corners.x.data(), corners.y.data()
                                , corners.z.data()
                                , corners.success.data());

    if (corners.allValid()) {
        // all pixels were transformed, check for right rotation (in pixel
        // space, Y grows down)
        if (rightRotating(corners.asPoints(), false)) {
            // right rotating polygon maps to sane right rotating polygon -> no
            // wrap arround
            return;
        }
    }

    try {
        auto ps(sourcePixelSize(src, dst));
        auto s(dst.size());
        ps *= std::max(s.width, s.height);

        // TODO: dynamic limit
        auto se(std::ceil(ps));
        if (se > 1000) {
            se = 1000;
        }

        LOG(info1) << "Setting SOURCE_EXTRA=" << se << " and SAMPLE_GRID=YES.";
        wo("SOURCE_EXTRA", int(se));
        wo("SAMPLE_GRID", true);
    } catch (ProjectionError) {
        // cannot project point from one SRS to another -> do nothing
    }
}

void createTransformer(GDALWarpOptions *wo)
{
    // destroy transformer if present
    if (wo->pTransformerArg) {
        ::GDALDestroyGenImgProjTransformer(wo->pTransformerArg);
    }

    // create new
    wo->pfnTransformer = ::GDALGenImgProjTransform;
    wo->pTransformerArg =
        ::GDALCreateGenImgProjTransformer
        (wo->hSrcDS, ::GDALGetProjectionRef(wo->hSrcDS)
         , wo->hDstDS, ::GDALGetProjectionRef(wo->hDstDS)
         , true, 0, 1);         
}


void obtainScale(GDALWarpOptions *wo, GeoDataset::WarpResultInfo & wri) {

    try {

        auto dst(static_cast<GDALDataset*>(wo->hDstDS));
    
        struct {
            std::array<double,45> x,y,z;
            std::array<int,45> successf, successb;
        } samples;
     
        for (int j = 0; j < 3; j++) 
            for(int i = 0; i < 3; i++) {
        
                samples.x[3*j+i] = i / 2.0 * dst->GetRasterXSize();
                samples.y[3*j+i] = j / 2.0 * dst->GetRasterYSize();
                samples.z[3*j+i] = 0;
            }
        
        
        if ( wo->pfnTransformer(wo->pTransformerArg, TRUE, 9, 
            samples.x.data(), samples.y.data(), samples.z.data(), 
            samples.successf.data() ) != TRUE ) {
        }
        
        for (int i = 0; i < 9; i++) {
    
            samples.x[9+i] = samples.x[i] + 1.0;
            samples.y[9+i] = samples.y[i];
            samples.z[9+i] = 0;
            samples.x[18+i] = samples.x[i] - 1.0;
            samples.y[18+i] = samples.y[i];
            samples.z[18+i] = 0;
            samples.x[27+i] = samples.x[i];
            samples.y[27+i] = samples.y[i] + 1.0;
            samples.z[27+i] = 0;
            samples.x[36+i] = samples.x[i];
            samples.y[36+i] = samples.y[i] - 1.0;
            samples.z[36+i] = 0;
        }
    
        if ( wo->pfnTransformer(wo->pTransformerArg, FALSE, 45, 
            samples.x.data(), samples.y.data(), samples.z.data(), 
            samples.successb.data() ) != TRUE ) {
            LOGTHROW( err2, std::runtime_error ) << "Transformer failed.";
        }  
        
        auto scale = boost::make_optional(false, 0.0);
    
        for ( int i = 0; i < 9; i++ ) 
            if (samples.successf[i] && samples.successb[i] &&
                samples.successb[9+i] && samples.successb[18+i] &&
                samples.successb[27+i] && samples.successb[36+i] ) {
                 ublas::vector<double> a(4);
                 a[0] = std::min(fabs(samples.x[9+i] - samples.x[i]), fabs(samples.x[18+i] - samples.x[i]));
                 a[1] = std::min(fabs(samples.y[9+i] - samples.y[i]),  fabs(samples.y[18+i] - samples.y[i]));
                 a[2] = std::min(fabs(samples.x[27+i] - samples.x[i]), fabs(samples.x[36+i] - samples.x[i]));
                 a[3] = std::min(fabs(samples.y[27+i] - samples.y[i]), fabs(samples.y[36+i] - samples.y[i]));
                 double nscale = norm_2(a);
             
                 //LOG( debug ) << a;
                 //LOG( debug ) << boost::format("Dst scale is %.5f.") % nscale;

                 if ( ! scale || nscale > *scale )
                    scale = nscale;
            }

       if ( ! scale )    
           LOGTHROW(err1, std::runtime_error) << "Transformer failed.";
   
       wri.truescale = wri.scale = *scale;
       
   } catch ( std::runtime_error & ) {
   
       LOG(warn3) << "Could not determine scale, falling back to 1.";
       wri.truescale = wri.scale = 1.0;
   }
}

std::unique_ptr<GDALDataset> useOverview(
    GDALDataset *src, 
    GDALWarpOptions *wo, int ovr,
    GeoDataset::WarpResultInfo & wri) {

    // use chosen overwiew
    std::unique_ptr<GDALDataset> ovrDs;
    
    if (ovr >= 0) {
       ovrDs = std::unique_ptr<GDALDataset>(
           ::GDALCreateOverviewDataset
           (src, ovr, false
#if GDAL_VERSION_NUM < 2020000
            // bOwnDS available only up to 2.1.x
            , false
#endif
            ));
    }
    
    if (!ovrDs) {
        // failed -> go on with original dataset
        return ovrDs;
    }

    // update warp options

    // set new source
    wo->hSrcDS = ovrDs.get();

    // re-create transformer
    createTransformer(wo);


    // determine scale
    double overviewScale = double(ovrDs->GetRasterXSize()) / src->GetRasterXSize();
    wri.scale = wri.truescale / overviewScale;
    
    LOG(debug)( "Using overview #%d, truescale %.5f, scale %.5f"
                , ovr, wri.truescale, wri.scale );

    // and return holder
    return ovrDs;
}

std::unique_ptr<GDALDataset> overviewByScale(GDALDataset *src, 
    GDALWarpOptions *wo, int & ovr, GeoDataset::WarpResultInfo &wri) {

    auto band(src->GetRasterBand(1));
    auto count(band->GetOverviewCount());
    
    // true scale is presumed to be defined
    LOG(info1)
        ( "Overview selection based on dst/src scale of %.5f.", wri.truescale );

    // choose overview
    for (; ovr < count - 1; ++ovr) {

        double nextScale(double(band->GetOverview(ovr + 1)->GetXSize())
          / double(src->GetRasterXSize()));

        if (nextScale < wri.truescale) {
            break;
        }
    }

    // all done
    return useOverview(src, wo, ovr, wri);
}

std::unique_ptr<GDALDataset> overviewByMemoryReqs(GDALDataset *src, 
  GDALWarpOptions * wo, int & ovr, GeoDataset::WarpResultInfo & wri) {
  
  auto band(src->GetRasterBand(1));
  auto count(band->GetOverviewCount());

  bool requirementsMet(false);
  ulong measure(0UL);
  
  if (wo->dfWarpMemoryLimit == 0.0) {
  
     LOG(warn2) << "Warp memory limit set to internal default, "
       "cannot test for memory requirements.";
     return {}; 
  }
    
  std::unique_ptr<GDALDataset> ovrDs;  
    
  for ( ;ovr < count; ovr++) {
  
     // use overview
     ovrDs = useOverview(src, wo, ovr, wri);     

     // obtain measure
     measure = detail::WarpMemoryMeter(wo).measure();
  
     // test
     if ( ovr >= 0 ) {
     
         LOG(info1)("Memory requirements at overview %d: %lu bytes."
                    , ovr, measure);
         
     } else {
     
         LOG(info1)("Memory requirements at original: %lu bytes."
                    , measure);
     }
        
     if (measure <= wo->dfWarpMemoryLimit) {
     
        requirementsMet = true; break; 
     }
  }
    
  if (! requirementsMet) {
     LOG(warn2)("Could not meet desired memory requirements "
                "(%lu est > %lu target), need more overviews?"
                , measure, wo->dfWarpMemoryLimit);
     ovr = count - 1;
  }
         
  return ovrDs;
}


std::unique_ptr<GDALDataset>
chooseOverview(GDALWarpOptions *wo, GeoDataset::WarpResultInfo &wri
               , const GeoDataset::WarpOptions &options)
{
    auto src(static_cast<GDALDataset*>(wo->hSrcDS));
    std::unique_ptr<GDALDataset> ovrDs;

    wri.scale = 1.0; // IMPROVE - fallback value for original dataset

    // overview set explicitely in options
    if (options.overview) {
    
        // no auto selection
        auto ovr(*options.overview);
        
        // set to none -> original
        if (!ovr) { 
           return {}; 
        }

        // use given overview
        wri.overview = *ovr;
        ovrDs = useOverview(src, wo, *ovr, wri);
        return ovrDs;
    }

    // automatic overview determination - start at original dataset
    int ovr(-1);

    // relax to overview due to scale
    ovrDs = overviewByScale(src, wo, ovr, wri);

    LOG(info1) << "Most efficient lossless source is " << (( ovr > -1 ) ? 
        boost::format( "overview #%d." ) % ovr : boost::format("original."));
      
    // force overview due to memory requirements
    if (options.safeChunks) {

       ovrDs = overviewByMemoryReqs(src, wo, ovr, wri);

       LOG(info1) << "Safe chunks requirement results in usage of " 
           << (( ovr > -1 ) ? boost::format( "overview #%d." ) % ovr 
               : boost::format("original."));
    }

    // -1 -> original dataset
    if (ovr < 0) {
    
        wri.overview = boost::none;
        LOG(info1) << "Warp uses original dataset (no suitable overviews).";
        
    } else {
    
        wri.overview = ovr;
        LOG(info1)("Warp uses overview #%d instead of the full dataset."
                   , ovr);
    }
    
    // use given overview
    return ovrDs;
}

const GeoDataset::NodataValue&
chooseNodataValue(const GeoDataset::NodataValue &original
                  , const GeoDataset::OptionalNodataValue &override)
{
    // return original unless override is set
    return override ? *override : original;
}

GDALResampleAlg chooseResampling(GeoDataset::Resampling alg
                                 , GeoDataset::WarpResultInfo &wri)
{
    switch (alg) {
    case GeoDataset::Resampling::texture:
        if (wri.scale >= 0.5) {
            // upscale or dowscale not less than half of original
            return algoToGdal
                (wri.resampling = GeoDataset::Resampling::cubic);
        }

        // downscaling to less than half
        return algoToGdal
            (wri.resampling = GeoDataset::Resampling::average);

    case GeoDataset::Resampling::dem:
        if (wri.scale >= 0.5) {
            // upscale or dowscale not less than half of original
            return algoToGdal
                (wri.resampling = GeoDataset::Resampling::cubicspline);
        }

        // keep or downscale
        return algoToGdal
            (wri.resampling = GeoDataset::Resampling::average);

    default: break;
    }

    // use provided resampling
    return algoToGdal(wri.resampling = alg);
}

struct CmdlineLogger {
    CmdlineLogger(const GDALWarpOptions *warpOptions
                  , const GeoDataset::WarpOptions &options
                  , const GeoDataset::WarpResultInfo &wri
                  , const GeoDataset &src
                  , const GeoDataset &dst
                  , GDALDataType ot)
        : warpOptions(warpOptions), options(options)
        , wri(wri), src(src), dst(dst), ot(ot)
    {}

    const GDALWarpOptions *warpOptions;
    const GeoDataset::WarpOptions &options;
    const GeoDataset::WarpResultInfo &wri;
    const GeoDataset &src;
    const GeoDataset &dst;
    GDALDataType ot;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const CmdlineLogger &l)
{
    os << std::fixed << "gdalwarp";
    os << " -r " << l.wri.resampling;
    if (l.wri.overview) {
        os << " -ovr " << *l.wri.overview;
    } else {
        os << " -ovr NONE";
    }

    auto extents(l.dst.extents());
    auto size(l.dst.size());
    os << " -te " << extents.ll(0) << ' ' << extents.ll(1) << ' '
       << extents.ur(0) << ' ' << extents.ur(1);

    os << " -ts " << size.width << ' ' << size.height;

    if (l.options.srcNodataValue) {
        if (*l.options.srcNodataValue) {
            os << " -srcnodata " << **l.options.srcNodataValue;
        } else {
            os << " -srcnodata None";
        }
    }

    if (l.options.dstNodataValue) {
        if (*l.options.dstNodataValue) {
            os << " -dstnodata " << **l.options.dstNodataValue;
        } else {
            os << " -dstnodata None";
        }
    }

    switch (l.ot) {
    case ::GDT_Byte: os << " -ot byte"; break;
    case ::GDT_UInt16: os << " -ot uint16"; break;
    case ::GDT_Int16: os << " -ot int16"; break;
    case ::GDT_UInt32: os << " -ot uint32"; break;
    case ::GDT_Int32: os << " -ot int32"; break;
    case ::GDT_Float32: os << " -ot float32"; break;
    case ::GDT_Float64: os << " -ot float64"; break;
    default: break;
    }

    os << " -t_srs \"" << l.dst.srsProj4() << "\"";

    if (l.warpOptions->papszWarpOptions) {
        for (char **opts(l.warpOptions->papszWarpOptions);
             *opts; ++opts)
        {
            os << " -wo " << *opts;
        }
    }

    if (l.warpOptions->dfWarpMemoryLimit) {
        os << " -wm " << l.warpOptions->dfWarpMemoryLimit / (1024 * 1024);
    }

    return os;
}

} // namespace

GeoDataset::WarpResultInfo
GeoDataset::warpInto(GeoDataset &dst
                     , const boost::optional<Resampling> & requestedAlg
                     , const WarpOptions &options)
    const
{
    // sanity
    ut::expect( dset_->GetRasterCount() == dst.dset_->GetRasterCount()
                && dset_->GetRasterCount() > 0,
            "Error in warp: both dataset need to have the same number of bands." );

    Resampling alg(requestedAlg ? *requestedAlg : Resampling::lanczos);

    OptionsWrapper wo(options);

    // ---- optimization starts here ----
    if (areSame(srs(), dst.srs(), SrsEquivalence::geographic)) {
        // both datsets have equivalent SRS
        bool canCopy(false);
        math::Point2 dsOffset;

        if ( areIdenticalButForShift( geoTransform_, dst.geoTransform() ) ) {
            // distance between origins
            auto res(resolution());

            dsOffset = dst.origin() - origin();
            dsOffset(0) /= res(0);
            dsOffset(1) /= res(1);
            const double tolerance(1e-4);
            canCopy = (math::isInteger(dsOffset(0), tolerance)
                       && math::isInteger(dsOffset(1), tolerance));
        }


        if (canCopy) {
            LOG(info1)
                << "Source and destination datasets match pixel to pixel -> "
                "using nearest filter.";
            alg = Resampling::nearest;
        }
    }
    // ---- optimization ends here ----

    // create warp options
    GDALWarpOptions * warpOptions = GDALCreateWarpOptions();

    warpOptions->hSrcDS = dset_.get();
    warpOptions->hDstDS = dst.dset_.get();

    warpOptions->nBandCount = dset_->GetRasterCount();

    warpOptions->panSrcBands =
        (int *) CPLMalloc(sizeof(int) * warpOptions->nBandCount );
    warpOptions->panDstBands =
        (int *) CPLMalloc(sizeof(int) * warpOptions->nBandCount );

    const NodataValue &srcNodataValue
        (chooseNodataValue(noDataValue_, options.srcNodataValue));
    const NodataValue &dstNodataValue
        (chooseNodataValue(dst.noDataValue_, options.dstNodataValue));

    if ( srcNodataValue ) {

        warpOptions->padfSrcNoDataReal =
            (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );
        warpOptions->padfSrcNoDataImag =
        (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );

    }

    if ( dstNodataValue ) {

        warpOptions->padfDstNoDataReal =
            (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );
        warpOptions->padfDstNoDataImag =
            (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );
    }


    for ( int i = 0; i < warpOptions->nBandCount; i++ ) {

        warpOptions->panSrcBands[i] = i + 1;
        warpOptions->panDstBands[i] = i + 1;

        if ( srcNodataValue ) {

            warpOptions->padfSrcNoDataReal[i] = srcNodataValue.get();
            warpOptions->padfSrcNoDataImag[i] = 0.0;
        }

        if ( dstNodataValue ) {

            warpOptions->padfDstNoDataReal[i] = dstNodataValue.get();
            warpOptions->padfDstNoDataImag[i] = 0.0;
        }
    }

    warpOptions->pfnProgress = GDALDummyProgress;

    // set virtual memory for warp operation (GDAL default is 64 megs)
    warpOptions->dfWarpMemoryLimit = options.warpMemoryLimit;

    // establish reprojection transformer.
    createTransformer(warpOptions);

    // calculate sourceExtra parameter from datasets (if needed)
    // TODO: move after overview selection and use overview information
    sourceExtra(*this, dst, wo, warpOptions);

    GeoDataset::WarpResultInfo wri;

    // obtain maximum scaling factor source->destination
    obtainScale(warpOptions, wri);

    // choose overview and hold the dataset
    auto ovrDs(chooseOverview(warpOptions, wri, options));

    // update options and grab options since it is destroyed by
    // GDALDestroyWarpOptions
    if (!options.noInit) {
        // tell gdal to initilize output with no-data value
        wo("INIT_DEST", "NO_DATA");
    }
    warpOptions->papszWarpOptions = wo.release();

    // choose resampling
    warpOptions->eResampleAlg = chooseResampling(alg, wri);

    if (options.safeChunks) {
       // re-check memory requirements, resampling may change everything
       int ovr( wri.overview ? *wri.overview : -1 );
       ovrDs = overviewByMemoryReqs(dset_.get(), warpOptions, ovr, wri);
    }

    // initialize and execute the warp operation.
    GDALWarpOperation oOperation;

    oOperation.Initialize( warpOptions );
    CPLErr err = oOperation.ChunkAndWarpImage( 0, 0,
                                  dst.dset_->GetRasterXSize(),
                                  dst.dset_->GetRasterYSize() );

    GDALDestroyGenImgProjTransformer( warpOptions->pTransformerArg );

    // log before options destruction
    if (GEO_DUMP_GDALWARP) {
        LOG(info4) << CmdlineLogger
            (warpOptions, options, wri, *this, dst
             , dst.dset_->GetRasterBand(1)->GetRasterDataType());
        LOG(info4) << "scale: " << wri.scale;
        LOG(info4) << "truescale: " << wri.truescale;
    }

    GDALDestroyWarpOptions( warpOptions );

    if (err != CE_None) {
        LOGTHROW(err1, WarpError)
            << "Warp failed: <" << ::CPLGetLastErrorMsg() << ">"
            << " (err=" << err << ").";
    }

    // invalidate local copies of target dataset content
    dst.data_ = boost::none;
    dst.mask_ = boost::none;
    dst.fresh_ = false;

    // done
    return wri;
}

void GeoDataset::assertData(int what) const {

    UTILITY_OMP(critical)
    {
        if (!data_ && (what & DataFlag::data)) {
            loadData();
        }
        if (!mask_ && (what & DataFlag::mask)) {
            loadMask();
        }
    }
}

void GeoDataset::loadData() const {
    // sanity
    ut::expect( dset_->GetRasterCount() > 0 );

    if (fresh_) {
        // freshly created -> create initial data
        data_ = cv::Mat(size_.height, size_.width, CV_64FC(numChannels_));
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
        return;
    }

    // determine matrix data type
    GDALDataType gdalDataType = dset_->GetRasterBand(1)->GetRasterDataType();
    int numChannels = dset_->GetRasterCount();
    int cvDataType(gdal2cv(gdalDataType, numChannels));

    // create matrix
    cv::Mat raster( size_.height, size_.width, cvDataType );
    int valueSize = raster.elemSize() / raster.channels();

    // transfer data

    for ( int i = 1; i <= numChannels; i++ ) {

        int bandMap( i );

        auto err = dset_->RasterIO(
            GF_Read, // GDALRWFlag  eRWFlag,
            0, 0, // int nXOff, int nYOff
            size_.width, size_.height, // int nXSize, int nYSize,
            (void *) ( raster.data
                       + ( channelMapping_[i]  * valueSize) ),  // void * pData,
            size_.width, size_.height, // int nBufXSize, int nBufYSize,
            gdalDataType, // GDALDataType  eBufType,
            1, //  int nBandCount,
            & bandMap,  // int * panBandMap,
            raster.elemSize(), // int nPixelSpace,
            size_.width * raster.elemSize(), 0 ); // int nLineSpace

        ut::expect( err == CE_None, "Reading of raster data failed." );
    }

    // convert
    data_ = cv::Mat();
    raster.convertTo( data_.get(), CV_64FC( numChannels ) );

    // all done
}

cv::Mat GeoDataset::fetchMask(bool optimized) const
{
    // sanity
    ut::expect( dset_->GetRasterCount() > 0 );

    // establish mask
    // generate mask

    // get mask band
    auto band(dset_->GetRasterBand(1));
    if (optimized && (band->GetMaskFlags() & GMF_ALL_VALID)) {
        return {};
    }

    // some invalid pixels
    auto maskBand(band->GetMaskBand());
    auto gdalDataType = maskBand->GetRasterDataType();
    auto cvDataType = gdal2cv(gdalDataType, 1);
    ut::expect((cvDataType == CV_8UC1)
               , "Expected band mask to be of byte type.");

    cv::Mat raster(size_.height, size_.width, cvDataType);

    auto err = maskBand->RasterIO
        (GF_Read // GDALRWFlag  eRWFlag,
         , 0, 0 // int nXOff, int nYOff
         , size_.width, size_.height // int nXSize, int nYSize,
         , (void *)(raster.data)  // void * pData,
         , size_.width, size_.height // int nBufXSize, int nBufYSize,
         , gdalDataType // GDALDataType  eBufType,
         , raster.elemSize() // int nPixelSpace,
         , size_.width * raster.elemSize(), 0); // int nLineSpace

    ut::expect((err == CE_None), "Reading of mask band data failed.");

    // done
    return raster;
}

void GeoDataset::loadMask() const {
    // fetch mask (in optimized mode)
    auto raster(fetchMask(true));

    if (fresh_) {
        // freshly created -> create initial data
        data_ = cv::Mat(size_.height, size_.width, CV_64FC(numChannels_));
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
        return;
    }

    if (!raster.data) {
        // invalid matrix + optimized mode -> whole dataset is valid
        LOG(debug) << "Loaded fully valid mask.";
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
        return;
    }

    // statistics
    std::size_t total(size_.width * size_.height);
    std::size_t nz(countNonZero(raster));
    LOG(debug) << "Loaded mask with " << nz << "/" << total << " pixels.";

    if (!nz) {
        // empty
        mask_ = RasterMask(size_.width, size_.height, RasterMask::EMPTY);
        return;
    }
    if (nz == total) {
        // full
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
        return;
    }

    // partial
    const auto *d(raster.data);
    if (nz >= (total / 2)) {
        // at least half is set, start with full and remove unset pixels
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
        for (int j = 0; j < size_.height; ++j) {
            for (int i = 0; i < size_.width; ++i) {
                if (!*d++) {
                    mask_->set(i, j, false);
                }
            }
        }
    } else {
        // less than hald is set, start with empty and set pixels
        mask_ = RasterMask(size_.width, size_.height, RasterMask::EMPTY);
        for (int j = 0; j < size_.height; ++j) {
            for (int i = 0; i < size_.width; ++i) {
                if (*d++) {
                    mask_->set(i, j, true);
                }
            }
        }
    }
}

void GeoDataset::exportCvMat( cv::Mat & raster, int cvDataType ) {

    assertData();
    data_->convertTo( raster, cvDataType );
}

void GeoDataset::expectGray() const {

    ut::expect((type_ == Type::grayscale)
               ,  "Not a single channel grayscale image.");
}

void GeoDataset::expectRGB() const {

    ut::expect((type_ == Type::rgb), "Not a 3 channel RGB image." );
}

void GeoDataset::expectAlpha() const
{
    ut::expect((type_ == Type::alpha), "Not a single channel alpha image.");
}

void GeoDataset::expectMask() const
{
    ut::expect((type_ == Type::grayscale) || (type_ == Type::alpha)
               ,  "Not a single channel grayscale or alpha image.");
}


double GeoDataset::geo2height(double gx, double gy, double gz) const
{
        double x, y; 
        geoTransform_.geo2rowcol({gx,gy,gz},y,x); 
        if(!valid(x,y)){
            LOGTHROW( err3, std::runtime_error )<<"Invalid coordinates in geodataset.";    
        }
        return data_->at<double>(y, x);
}

void GeoDataset::exportMesh( geometry::Mesh & mesh) const {

    std::map<std::pair<int,int>, int> vpos2ord; // i,j -> vertex ordinal

    // expect a single gray channel
    expectGray();

    // obtain heightfield data
    assertData();

    // dump vertices
    int ord( 0 );

    math::Matrix4 localTrafo = geo2local( extents() );

    for ( int i = 0; i < size_.height; i++ )
        for ( int j = 0; j < size_.width; j++ ) {

            //LOG( debug ) << "row " << i << ", col " << j << ": " <<
            //    mdsm.at<double>( i, j );

            // check if vertex is defined in dsm
            if ( ! valid( i, j ) ) continue;

            // vertex coordinates
            math::Point3 pvertex;
            math::Point3 geoVertex;

            geoVertex = geoTransform_.rowcol2geo( i, j, data_->at<double>( i, j ) );
            pvertex = transform(
                localTrafo,
                geoVertex );

            mesh.vertices.push_back( pvertex );

            // take note of vertex ordinal number
            vpos2ord[ std::make_pair( i, j ) ] = ord++;
    }

    mesh.tCoords.push_back( math::Point2( 0.0, 0.0 ) );

    // dump faces
    for ( int i = 0; i < size_.height - 1; i++ )
        for ( int j = 0; j < size_.width - 1; j++ ) {

            bool v00, v01, v10, v11;
            std::map<std::pair<int,int>, int>::iterator
                ref00, ref01, ref10, ref11;

            v00 = ( ref00 = vpos2ord.find(
                std::make_pair(i,j) ) ) != vpos2ord.end();
            v01 = ( ref01 = vpos2ord.find(
                std::make_pair(i,j+1) ) ) != vpos2ord.end();
            v10 = ( ref10 = vpos2ord.find(
                std::make_pair(i+1,j) ) ) != vpos2ord.end();
            v11 = ( ref11 = vpos2ord.find(
                std::make_pair(i+1,j+1) ) ) != vpos2ord.end();

            
             // lower
            if ( v10 && v11 && v00 ) {

                mesh.addFace(
                    ref10->second, ref11->second, ref00->second,
                    ref10->second, ref11->second, ref00->second );

            }

            // upper
            if ( v11 && v01 && v00 ) {

                mesh.addFace(
                    ref11->second, ref01->second, ref00->second,
                    ref11->second, ref01->second, ref00->second );

            }

            //alternative face diagonal direction
            if ( v00 && v10 && v01 && !v11) {

                mesh.addFace(
                    ref00->second, ref10->second, ref01->second,
                    ref00->second, ref10->second, ref01->second );
            }

            if ( v10 && v11 && v01 && !v00) {
                mesh.addFace(
                    ref10->second, ref11->second, ref01->second,
                    ref10->second, ref11->second, ref01->second );
            }        
        }

    // all done
}

void GeoDataset::filterMesh( const geometry::Mesh& mesh, const math::Extents2 & extents
                           , geometry::Mesh& omesh ) const
{
    // sanity
    ut::expect( omesh.vertices.size() == 0 && omesh.tCoords.size() == 0 &&
        omesh.faces.size() == 0, "Output mesh expected to be empty." );

    bool maskFull = mask_->count() == mask_->capacity();
    if(maskFull){
        omesh = mesh;
        return;
    };

    std::map<int,int> iord2oord; // i,j -> vertex ordinal
    int ord(0);

    math::Matrix4 il2geo
        = local2geo( extents );

    // iterate through vertices
    for ( uint i = 0; i < mesh.vertices.size(); i++ ) {
        const math::Point3 & vertex( mesh.vertices[i] );

        // skip non-texturable vertices
        double row, col;

        geoTransform_.geo2rowcol( transform( il2geo, vertex ), row, col );

        if ( ! validf( row, col ) ) continue;

        // write out vertex and tcoords
        omesh.vertices.push_back( vertex );
        iord2oord[i] = ord++;
    }

    // iterate though faces
    for ( geometry::Face face : mesh.faces ) {

        std::map<int,int>::iterator ref0, ref1, ref2;
        bool v0, v1, v2;

        v0 = ( ref0 = iord2oord.find( face.a ) ) != iord2oord.end();
        v1 = ( ref1 = iord2oord.find( face.b ) ) != iord2oord.end();
        v2 = ( ref2 = iord2oord.find( face.c ) ) != iord2oord.end();

        if ( v0 && v1 && v2 )
            omesh.faces.emplace_back(
                ref0->second, ref1->second, ref2->second,
                ref0->second, ref1->second, ref2->second );
    }


}

math::Points3 GeoDataset::exportPointCloud() const
{
    // expect a single gray channel
    expectGray();

    // obtain heightfield data
    assertData();

    // dump vertices
    math::Points3 pc;
    for ( int i = 0; i < size_.height; i++ ) {
        for ( int j = 0; j < size_.width; j++ ) {
            if ( ! valid( i, j ) ) continue;

            // vertex coordinates
            math::Point3 pvertex;

            pvertex = geoTransform_.rowcol2geo( i, j, data_->at<double>( i, j ) );

            pc.push_back(pvertex);
        }
    }
    // all done
    return pc;
}

void GeoDataset::textureMesh(
    const geometry::Mesh & imesh, const math::Extents2 & extents,
    geometry::Mesh & omesh ) const {

    std::map<  math::Points3::size_type
             , math::Points3::size_type> iord2oord; // i,j -> vertex ordinal
    int ord(0);

    // sanity
    ut::expect( omesh.vertices.size() == 0 && omesh.tCoords.size() == 0 &&
        omesh.faces.size() == 0, "Output mesh expected to be empty." );

    // establish input local -> dset normalized
    math::Matrix4 il2dn = prod(
        geo2normalized( this->extents() ), local2geo( extents ) );
    math::Matrix4 il2geo
        = local2geo( extents );
    math::Matrix4 igeo2l
        = geo2local( extents );    


    // to find out if the triangle is texturable
    std::vector<bool> faceValid(imesh.faces.size(), false);
    bool maskFull = mask_->count() == mask_->capacity();
    if(!maskFull){
        // compute intersection of each triangle with each white rectange in quadtree rastermask
        // if at least one rectange intersects the triangle, triangle is texturable
        mask_->forEachQuad([&](uint xstart, uint ystart, uint xsize
                             , uint ysize, bool)
        {
            double eps = 1.0/16;
            math::Point3 ll = transform( igeo2l, 
                                         raster2geo(math::Point2(xstart-0.5f-eps,ystart+ysize-0.5f+eps), 0));
            math::Point3 ur = transform( igeo2l, 
                                         raster2geo(math::Point2(xstart+xsize-0.5f+eps,ystart-0.5f-eps), 0));
            for ( std::size_t fid = 0; fid < imesh.faces.size(); ++fid ) {
                if ( faceValid[fid] )
                    continue;
                math::Point2 points[3] = {
                     math::Point2( imesh.vertices[imesh.faces[fid].a][0]
                                 , imesh.vertices[imesh.faces[fid].a][1])
                   , math::Point2( imesh.vertices[imesh.faces[fid].b][0]
                                 , imesh.vertices[imesh.faces[fid].b][1])
                   , math::Point2( imesh.vertices[imesh.faces[fid].c][0]
                                 , imesh.vertices[imesh.faces[fid].c][1])
                };            
                //use rectangle coordinates with small inside margin, to prevent edge collision
                faceValid[fid] = faceValid[fid] || math::triangleRectangleCollision( points
                                                , math::Point2(ll[0],ll[1])
                                                , math::Point2(ur[0],ur[1]));                                              
            }
        }, RasterMask::Filter::white);
    }

    // iterate though faces
    for ( std::size_t fid = 0; fid < imesh.faces.size(); ++fid ) {
        if ( maskFull || faceValid[fid] ) {
            std::size_t indices[3] = {
                  imesh.faces[fid].a
                , imesh.faces[fid].b
                , imesh.faces[fid].c
             };
            for ( uint i=0;i<3;++i ) {
                std::size_t srcVid = indices[i];
                auto pair = iord2oord.insert(std::make_pair(srcVid, ord));
                if ( pair.second ) ord++;
                indices[i]=pair.first->second;

                if ( indices[i] >= omesh.vertices.size() ) {
                    const math::Point3 & vertex( imesh.vertices[srcVid] );
                    // normalized coords are in (-1.0,1.0) range -> transform to (0.0,1.0)
                    math::Point3 tcoord3 = transform( il2dn, vertex ) * 0.5
                        + math::Point3( 0.5, 0.5, 0.0 );

                    math::Point2 tcoord( tcoord3[0], tcoord3[1] );

                    // write out vertex and tcoords
                    omesh.vertices.push_back( vertex );
                    omesh.tCoords.push_back( math::Point2( tcoord[0], tcoord[1] ) );
                }
            }

            omesh.faces.emplace_back( indices[0], indices[1], indices[2]
                                    , indices[0], indices[1], indices[2] );
        }
    }
}

math::Extents2 GeoDataset::deriveExtents( const SrsDefinition &srs ) const
{
    // use source's SRS if same (this prevents conversion through lat/lon if
    // GDAL thinks that these are not the same)
    std::string srsWkt
        (areSame(srs, SrsDefinition(srsWkt_, SrsDefinition::Type::wkt))
         ? srsWkt_
         : srs.as(SrsDefinition::Type::wkt).srs);

    CPLErr err;

    void * transformer =
        GDALCreateGenImgProjTransformer(
            dset_.get(),
            srsWkt_.c_str(),
            nullptr,
            srsWkt.c_str(),
            false, 0, 1 );

    if ( ! transformer )
        LOGTHROW( err2, std::runtime_error ) << "Error obtaining transformer.";

    GeoTransform outputTransform;
    math::Size2i outputSize;

    err = GDALSuggestedWarpOutput(
            dset_.get(),
            GDALGenImgProjTransform,
            transformer,
            outputTransform.data(),
            & outputSize.width, & outputSize.height );

    if ( err != CE_None )
        LOGTHROW( err2, std::runtime_error ) << "Error transforming extents.";

    math::Extents2 retval;

    math::Point2 ll, lr, ul, ur;

    ll = outputTransform.applyGeoTransform( 0, outputSize.height );
    lr = outputTransform.applyGeoTransform( outputSize.width, outputSize.height );
    ul = outputTransform.applyGeoTransform( 0, 0 );
    ur = outputTransform.applyGeoTransform( outputSize.width, 0 );


    retval.ll[0] = std::min( { ll[0], lr[0], ul[0], ur[0] } );
    retval.ll[1] = std::min( { ll[1], lr[1], ul[1], ur[1] } );
    retval.ur[0] = std::max( { ll[0], lr[0], ul[0], ur[0] } );
    retval.ur[1] = std::max( { ll[1], lr[1], ul[1], ur[1] } );

    return retval;
}

bool GeoDataset::isOrthogonal() const {
    
    double epsilon( 1.0e-6 );
    
    return ( fabs( geoTransform_[2] ) < epsilon 
        && fabs( geoTransform_[4] ) < epsilon );
}

math::Extents2 GeoDataset::extents() const {
    
    if ( ! isOrthogonal() ) 
        LOGTHROW( err3, std::runtime_error ) 
            << "Non orthogonal dataset cannot be goreferenced by extents.";
            
    math::Extents2 retval;
    math::Point2 ll, lr, ul, ur;

    ll = geoTransform_.applyGeoTransform( 0, size_.height );
    lr = geoTransform_.applyGeoTransform( size_.width, size_.height );
    ul = geoTransform_.applyGeoTransform( 0, 0 );
    ur = geoTransform_.applyGeoTransform( size_.width, 0 );


    retval.ll[0] = std::min( { ll[0], lr[0], ul[0], ur[0] } );
    retval.ll[1] = std::min( { ll[1], lr[1], ul[1], ur[1] } );
    retval.ur[0] = std::max( { ll[0], lr[0], ul[0], ur[0] } );
    retval.ur[1] = std::max( { ll[1], lr[1], ul[1], ur[1] } );

    return retval;         
}

bool GeoDataset::valid( int i, int j ) const {

    assertData();
    return mask_->get( j, i );
}

bool GeoDataset::validf( double i, double j ) const {

    bool v00 = valid( int(floor(i)), int(floor(j)) );
    bool v01 = valid( int(floor(i)), int(ceil(j)) );
    bool v11 = valid( int(ceil(i)), int(ceil(j)) );
    bool v10 = valid( int(ceil(i)), int(floor(j)) );

    if ( v00 && v01 && v11 & v10 ) return true;

    // silly corner cases which testify to the deficiency of our data design
    bool top = math::ccinterval( -0.5, 0.0, i );
    bool bottom = math::ccinterval( size_.height-1.0, size_.height-0.5, i );
    bool left = math::ccinterval( -0.5, 0.0, j );
    bool right = math::ccinterval( size_.width-1.0, size_.width-0.5, j );

    if ( top && v10 & v11 ) return true;
    if ( left && v01 & v11 ) return true;
    if ( bottom && v00 & v01 ) return true;
    if ( right && v00 & v10 ) return true;

    if ( top & left & v11 ) return true;
    if ( top & right & v10 ) return true;
    if ( bottom & left & v01 ) return true;
    if ( bottom & right & v00 ) return true;
    
    return false;
}


/** Converts mask from src space of this space
 */
GeoDataset::Mask GeoDataset::convertMask(const geo::GeoDataset &src) const
{
    const auto srcExtents(src.extents());
    const auto dstExtents(extents());
    const auto srcSize(src.size());
    const auto dstSize(size());
    auto ssize(math::size(srcExtents));
    auto dsize(math::size(dstExtents));

    // transforms point from dst into src
    imgproc::quadtree::Matrix2x3 trafo
        (boost::numeric::ublas::zero_matrix<double>(2, 3));

    /* Transformation from dst space to src space
     *     p' = S * (p + 0.5) - 0.5 + O'
     * where:
     *     p: point in dst
     *     p': point in src
     *     S: scaling factor, i.e. ratio between src and dst pixel sizes
     *     +0.5: shift from pixel index to pixel coordinate
     *     -0.5: shift pixel coordinate to pixel index
     *     O': offset between upper left corners of extents in src space
     *
     * this leads to:
     *    p' = S * p + 0.5 * S - 0.5 + O'
     *    p' = S * p + 0.5 * (S - 1) + O'
     *
     * broken to transformation matrix:
     *    Sx  0   (0.5 * (Sx - 1) + Ox')
     *    0   Sy  (0.5 * (Sy - 1) + Oy')
     */

    // scale component (ratio between src and dst pixel size):
    trafo(0, 0) = ((srcSize.width * dsize.width)
                   / (dstSize.width * ssize.width));
    trafo(1, 1) = ((srcSize.height * dsize.height)
                   / (dstSize.height * ssize.height));

    /* extents shift (distance of ur(dstExtents) from ur(srcExtents) measured in
     * src pixels)
     */
    auto shiftX((srcSize.width * (dstExtents.ll(0) - srcExtents.ll(0)))
                / ssize.width);
    auto shiftY((srcSize.height * (srcExtents.ur(1) - dstExtents.ur(1)))
                / ssize.height);

    // shift component:
    trafo(0, 2) = shiftX + 0.5 * (trafo(0, 0) - 1.0);
    trafo(1, 2) = shiftY + 0.5 * (trafo(1, 1) - 1.0);

    LOG(debug) << "mask trafo: " << trafo;

    // and generate dst raster mask
    return transform(src.cmask(), dstSize, trafo);
}

void GeoDataset::applyMask(const GeoDataset &other)
{
    assertData();

    // if we have same grid at same extents -> just intersect
    if ((size() == other.size()) && (extents() == other.extents())) {
        mask_->intersect(other.cmask());
        return;
    }

    // we have to convert mask to this space
    mask_->intersect(convertMask(other));
}

void GeoDataset::applyMask(const Mask &mask)
{
    assertData();
    mask_->intersect(mask);
}

void GeoDataset::flush()
{
    if (!changed_) {
        return;
    }

    auto gdalDataType(dset_->GetRasterBand(1)->GetRasterDataType());
    int numChannels(dset_->GetRasterCount());
    int cvDataType(gdal2cv(gdalDataType, numChannels));

    // create tmp matrix
    cv::Mat raster(size_.height, size_.width, cvDataType);
    int valueSize(raster.elemSize() / raster.channels());

    // apply raster mask if there is a no data value
    if (noDataValue_) {
        auto undef = cv::Scalar(*noDataValue_);

        // raster all black pixels with undefined color
        mask_->forEachQuad([&](uint xstart, uint ystart, uint xsize
                               , uint ysize, bool)
        {
            cv::Point2i start(xstart, ystart);
            cv::Point2i end((xstart + xsize - 1), (ystart + ysize - 1));

            cv::rectangle(*data_, start, end, undef, CV_FILLED, 4);
        }, RasterMask::Filter::black); // BLACK PIXELS!
    }

    // convert data to tmp matrix
    data_->convertTo(raster, cvDataType);

    for (int i = 1; i <= numChannels; ++i) {
        int bandMap(i);

        auto err = dset_->RasterIO
            (GF_Write, // GDALRWFlag  eRWFlag,
             0, 0, // int nXOff, int nYOff
             size_.width, size_.height, // int nXSize, int nYSize,
             (void *) (raster.data
                       + (channelMapping_[i] * valueSize)), // void * pData,
             size_.width, size_.height, // int nBufXSize, int nBufYSize,
             gdalDataType, // GDALDataType  eBufType,
             1, //  int nBandCount,
             &bandMap,  // int * panBandMap,
             raster.elemSize(), // int nPixelSpace,
             size_.width * raster.elemSize(), 0); // int nLineSpace

        ut::expect(err == CE_None, "Writing of raster data failed.");
    }

    changed_ = false;
}

namespace {

GeoDataset::Metadata getMetadataFrom(GDALMajorObject *obj
                                     , const std::string &domain)
{
    auto md(obj->GetMetadata(domain.empty() ? 0x0 : domain.c_str()));
    if (!md) {
        return {};
    }

    GeoDataset::Metadata metadata;
    char *key;
    for (; *md; ++md) {
        auto value(::CPLParseNameValue(*md, &key));
        metadata(key, value);
        // get rid of key, it was allocated by parse function
        ::VSIFree(key);
    }

    return metadata;
}

void setMetadataIn(GDALMajorObject *obj, const GeoDataset::Metadata &metadata
                   , const std::string &domain)
{
    // convert metadata into list
    std::vector<std::string> strings;

    for (const auto &item : metadata.items()) {
        std::ostringstream os;
        os << item.first << "=" << item.second;
        strings.push_back(os.str());
    }

    std::vector<char*> md;
    for (auto &string : strings) {
        md.push_back(const_cast<char*>(string.c_str()));
    }
    md.push_back(nullptr);

    auto err(obj->SetMetadata
             (md.data(), domain.empty() ? 0x0 : domain.c_str()));

    if (err != CE_None) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot set metadata: <" << ::CPLGetLastErrorMsg() << ">"
            << " (err=" << err << ").";
    }
}

} // namespace

GeoDataset::Metadata GeoDataset::getMetadata(const std::string &domain) const
{
    return getMetadataFrom(dset_.get(), domain);
}

void GeoDataset::setMetadata(const Metadata &metadata
                             , const std::string &domain)
{
    setMetadataIn(dset_.get(), metadata, domain);
}

GeoDataset::Metadata
GeoDataset::getMetadata(int band, const std::string &domain) const
{
    return getMetadataFrom(dset_->GetRasterBand(band), domain);
}

void GeoDataset::setMetadata(int band, const Metadata &metadata
                             , const std::string &domain)
{
    setMetadataIn(dset_->GetRasterBand(band), metadata, domain);
}

// rawish interface

GeoDataset::Block GeoDataset::readBlock(const math::Point2i &blockOffset
                                        , bool native)
    const
{
    auto bs(blockSize());
    int numChannels(dset_->GetRasterCount());

    ut::expect(((blockOffset(0) >= 0) && (blockOffset(1) >= 0))
               , "Block out of raster.");

    // check whether we are not outside raster
    math::Point2 offset(blockOffset(0) * bs.width
                        , blockOffset(1) * bs.height);

    ut::expect(((offset(0) < size_.width) && (offset(1) < size_.height))
               , "Block out of raster.");

    math::Point2 end(offset(0) + bs.width, offset(1) + bs.height);
    if (end(0) > size_.width) { end(0) = size_.width; }
    if (end(1) > size_.height) { end(1) = size_.height; }

    math::Size2 size(end(0) - offset(0), end(1) - offset(1));

    ::GDALDataType gdalDataType
          (native
           ? dset_->GetRasterBand(1)->GetRasterDataType()
           : GDT_Float64);
    int cvDataType(gdal2cv(gdalDataType, numChannels_));

    Block block;
    block.data.create(size.height, size.width, cvDataType);
    int valueSize(block.data.elemSize() / block.data.channels());

    for (int i = 1; i <= numChannels; ++i) {
        int bandMap(i);
        auto err = dset_->RasterIO
            (GF_Read // GDALRWFlag  eRWFlag,
             , offset(0) // int nXOff
             , offset(1) // int nYOff
             , size.width, size.height // int nXSize, int nYSize,
             , (void *) (block.data.data
                         + (channelMapping_[i]  * valueSize))  // void * pData,
             , size.width, size.height // int nBufXSize, int
                                                 // nBufYSize,
             , gdalDataType // GDALDataType  eBufType,
             , 1 //  int nBandCount,
             , &bandMap  // int * panBandMap,
             , block.data.elemSize() // int nPixelSpace,
             , size.width * block.data.elemSize(), 0); // int nLineSpace

        ut::expect(err == CE_None, "Reading of raster data failed.");
    }

    return block;
}

GeoDataset::Block GeoDataset::readBlock(const math::Point2i &blockOffset
                                        , int band, bool native)
    const
{
    auto bs(blockSize());
    int numChannels(dset_->GetRasterCount());
    ut::expect((band < numChannels) && (band >= -numChannels)
               , "Band out of bounds.");
    // convert band to 1-based
    bool useMaskBand(false);
    if (band >= 0) {
        // band start at 1
        ++band;
    } else {
        // use mask band
        band = -band;
        useMaskBand = true;
    }

    ut::expect(((blockOffset(0) >= 0) && (blockOffset(1) >= 0))
               , "Block out of raster.");

    // check whether we are not outside raster
    math::Point2 offset(blockOffset(0) * bs.width
                        , blockOffset(1) * bs.height);

    ut::expect(((offset(0) < size_.width) && (offset(1) < size_.height))
               , "Block out of raster.");

    math::Point2 end(offset(0) + bs.width, offset(1) + bs.height);
    if (end(0) > size_.width) { end(0) = size_.width; }
    if (end(1) > size_.height) { end(1) = size_.height; }

    math::Size2 size(end(0) - offset(0), end(1) - offset(1));

    // use either band or first band's mask band
    GDALRasterBand *rb(useMaskBand
                       ? dset_->GetRasterBand(band)->GetMaskBand()
                       : dset_->GetRasterBand(band));
    ut::expect(rb, "No such band.");

    ::GDALDataType gdalDataType
          (native ? rb->GetRasterDataType() : GDT_Float64);
    int cvDataType(gdal2cv(gdalDataType, 1));

    Block block;
    block.data.create(size.height, size.width, cvDataType);

    auto err = rb->RasterIO
        (GF_Read // GDALRWFlag  eRWFlag,
         , offset(0) // int nXOff
         , offset(1) // int nYOff
         , size.width, size.height // int nXSize, int nYSize,
         , (void *) block.data.data // void * pData,
         , size.width, size.height // int nBufXSize, int
                                                 // nBufYSize,
         , gdalDataType // GDALDataType  eBufType,
         , block.data.elemSize() // int nPixelSpace,
         , size.width * block.data.elemSize(), 0); // int nLineSpace

    ut::expect(err == CE_None, "Reading of raster data failed.");

    return block;
}

void GeoDataset::writeBlock(const math::Point2i &blockOffset
                            , const cv::Mat &block)
{
    int numChannels(dset_->GetRasterCount());

    ut::expect((numChannels == block.channels())
               , "Invalid number of channels in block to write.");

    const int valueSize(block.elemSize() / block.channels());
    const auto type(cv2gdal(block.depth()));

    for (int i = 1; i <= numChannels; ++i) {
        int bandMap(i);
        auto err = dset_->RasterIO
            (GF_Write // GDALRWFlag  eRWFlag,
             , blockOffset(0) // int nXOff
             , blockOffset(1) // int nYOff
             , block.cols, block.rows // int nXSize, int nYSize,
             , (void *) (block.data
                         + (channelMapping_[i]  * valueSize))  // void * pData,
             , block.cols, block.rows // int nBufXSize, int nBufYSize,
             , type // GDALDataType  eBufType,
             , 1                      //  int nBandCount,
             , &bandMap               // int * panBandMap,
             , block.elemSize()       // int nPixelSpace,
             , block.cols * block.elemSize(), 0); // int nLineSpace

        ut::expect(err == CE_None, "Writing of raster data failed.");
    }
}

void GeoDataset::writeMaskBlock(const math::Point2i &blockOffset
                                , const cv::Mat &block)
{
    ut::expect((block.channels() == 1)
               , "Invalid number of channels in mask block to write.");

    auto *band(dset_->GetRasterBand(1));
    auto flags(band->GetMaskFlags());
    if (flags & GMF_ALL_VALID) {
        // no mask band
        dset_->CreateMaskBand(GMF_PER_DATASET);
    }

    auto *mask(band->GetMaskBand());

    const auto type(cv2gdal(block.depth()));

    auto err = mask->RasterIO
        (GF_Write // GDALRWFlag  eRWFlag,
         , blockOffset(0) // int nXOff
         , blockOffset(1) // int nYOff
         , block.cols, block.rows // int nXSize, int nYSize,
         , (void *) block.data  // void * pData,
         , block.cols, block.rows         // int nBufXSize, int nBufYSize
         , type                          // GDALDataType eBufType
         , block.elemSize()              // int nPixelSpace
         , block.cols * block.elemSize() // int nLineSpace
         , 0)                            // psExtraArg
        ;

    ut::expect(err == CE_None, "Writing of raster data failed.");
}

math::Size2 GeoDataset::blockSize() const
{
    math::Size2 size;
    dset_->GetRasterBand(1)->GetBlockSize(&size.width, &size.height);
    return size;
}

std::tuple<math::Point2i, math::Point2i>
GeoDataset::blockCoord(const math::Point2i &point) const
{
    auto size(blockSize());
    return std::tuple<math::Point2i, math::Point2i>
        (math::Point2i(point(0) / size.width, point(1) / size.height)
         , math::Point2i(point(0) % size.width, point(1) % size.height));
}

GeoDataset::BlockInfo::list GeoDataset::getBlocking() const
{
    GeoDataset::BlockInfo::list out;

    const auto bs(blockSize());

    // size in blocks
    const math::Size2 blocked((size_.width + bs.width - 1) / bs.width
                              , (size_.height + bs.height - 1) / bs.height);

    // last block size
    math::Point2i last((blocked.width - 1), (blocked.height - 1));
    const math::Size2 tail(size_.width - last(0) * bs.width
                           , size_.height - last(1) * bs.height);

    // generate list of blocks
    for (int j(0), y(0); j != blocked.height; ++j, y += bs.height) {
        int height((j == last(1)) ? tail.height : bs.height);
        for (int i(0), x(0); i != blocked.width; ++i, x += bs.width) {
            int width((i == last(0)) ? tail.width : bs.width);
            out.emplace_back(math::Point2i(x, y), math::Size2(width, height));
        }
    }

    return out;
}

math::Extents2 GeoDataset::pixelExtents(const math::Point2i &raster) const
{
    const math::Point2 halfPixel(resolution() / 2.0);
    const auto geo(geoTransform_.rowcol2geo(raster(1), raster(0), 0));
    return { geo(0) - halfPixel(0)
            , geo(1) - halfPixel(1)
            , geo(0) + halfPixel(0)
            , geo(1) + halfPixel(1) };
}

math::Extents2 GeoDataset::pixelExtents(const math::Extents2i &raster) const
{
    const math::Point2 halfPixel(resolution() / 2.0);
    const auto geo_ll(geoTransform_.rowcol2geo(raster.ll(1), raster.ll(0), 0));
    const auto geo_ur(geoTransform_.rowcol2geo(raster.ur(1), raster.ur(0), 0));
    return { geo_ll(0) - halfPixel(0)
            , geo_ur(1) - halfPixel(1)
            , geo_ur(0) + halfPixel(0)
            , geo_ll(1) + halfPixel(1) };
}

bool GeoDataset::allValid() const
{
    // all pixels in mask must be valid
    return (dset_->GetRasterBand(1)->GetMaskFlags() & GMF_ALL_VALID);
}

std::vector<int> BurnColor::bandList()
    const
{
    std::vector<int> out;
    int i(1);
    for (const auto &v : color_) {
        if (v) {
            // remember channel
            out.push_back(i);
        }
        ++i;
    }
    return out;
}

std::vector<double> BurnColor::valueList()
    const
{
    std::vector<double> out;
    for (const auto &v : color_) {
        if (v) { out.push_back(*v); }
    }
    return out;
}

void GeoDataset::rasterize(const ::OGRGeometry *geometry
                           , const BurnColor &color)
{
    auto bands(color.bandList());
    auto values(color.valueList());

    void *g(const_cast< ::OGRGeometry*>(geometry));
    auto err(::GDALRasterizeGeometries(dset_.get(), bands.size(), bands.data()
                                       , 1, &g
                                       , nullptr, nullptr, values.data()
                                       , nullptr, nullptr, nullptr));
    if (err != CE_None) {
        LOGTHROW(err2, std::runtime_error)
            << "Error rasterizing geometry.";
    }

    // invalidate
    data_ = boost::none;
    mask_ = boost::none;
    fresh_ = false;
}

GeoDataset::Format GeoDataset::getFormat() const
{
    Format f;
    f.storageType = Format::Storage::custom;
    f.driver = dset_->GetDriver()->GetDescription();

    int rc(dset_->GetRasterCount());
    if (!rc) { return f; }

    for (int i(1); i <= rc; ++i) {
        auto *band(dset_->GetRasterBand(i));
        if (i == 1) {
            f.channelType = band->GetRasterDataType();
        }
        f.channels.push_back(band->GetColorInterpretation());
    }

    return f;
}

std::size_t GeoDataset::bandCount() const
{
    return dset_->GetRasterCount();
}

GeoDataset::BandProperties GeoDataset::bandProperties(int band) const
{
    auto *b(dset_->GetRasterBand(band + 1));
    BandProperties bp;
    bp.dataType = b->GetRasterDataType();
    bp.colorInterpretation = b->GetColorInterpretation();
    bp.size.width = b->GetXSize();
    bp.size.height = b->GetYSize();
    b->GetBlockSize(&bp.blockSize.width, &bp.blockSize.height);
    return bp;
}

GeoDataset::BandProperties::list GeoDataset::bandProperties() const
{
    BandProperties::list out;
    for (int i(0), ei(dset_->GetRasterCount()); i != ei; ++i) {
        out.push_back(bandProperties(i));
    }
    return out;
}

math::Size2i GeoDataset::size(const Overview &ovr) const
{
    if (!ovr) { return size(); }

    // overview specified -> analyze overview of first band
    auto *b(dset_->GetRasterBand(1));
    auto count(b->GetOverviewCount());

    // no overview -> use full dataset
    if (!count) { return size(); }

    // if ovr is past end, use end
    int o(*ovr);
    if (o >= count) { o = count - 1; }

    auto ob(b->GetOverview(o));
    return math::Size2i(ob->GetXSize(), ob->GetYSize());
}

GeoDataset GeoDataset::createCopy(const boost::filesystem::path &path
                                  , const std::string &storageFormat
                                  , const geo::GeoDataset &src
                                  , const Options &options)
{
    auto *driver
        (GetGDALDriverManager()->GetDriverByName(storageFormat.c_str()));
    if (!driver) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot find GDAL driver for <" << storageFormat << ">.";
    }

    auto metadata(driver->GetMetadata());
    if (!CSLFetchBoolean(metadata, GDAL_DCAP_CREATECOPY, FALSE)) {
        LOGTHROW(err2, std::runtime_error)
            << "GDAL driver for <" << storageFormat
            << "> format doesn't know how to crete a copy.";
    }

    std::unique_ptr<GDALDataset>
        ds(driver->CreateCopy(path.c_str(), src.dset_.get()
                              , true // strict
                              , OptionsWrapper(options)
                              , ::GDALDummyProgress
                              , nullptr));
    if (!ds) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to copy dataset to " << path << ": <"
            << ::CPLGetLastErrorMsg() << ">";
    }

    return GeoDataset(std::move(ds));
}

GeoDataset GeoDataset::copy(const boost::filesystem::path &path
                            , const std::string &storageFormat
                            , const Options &options) const
{
    auto *driver
        (GetGDALDriverManager()->GetDriverByName(storageFormat.c_str()));
    if (!driver) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot find GDAL driver for <" << storageFormat << ">.";
    }

    auto metadata(driver->GetMetadata());
    if (!CSLFetchBoolean(metadata, GDAL_DCAP_CREATECOPY, FALSE)) {
        LOGTHROW(err2, std::runtime_error)
            << "GDAL driver for <" << storageFormat
            << "> format doesn't know how to crete a copy.";
    }

    std::unique_ptr<GDALDataset>
        ds(driver->CreateCopy(path.c_str(), dset_.get()
                              , true // strict
                              , OptionsWrapper(options)
                              , ::GDALDummyProgress
                              , nullptr));
    if (!ds) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to copy dataset to " << path << ": <"
            << ::CPLGetLastErrorMsg() << ">";
    }

    return GeoDataset(std::move(ds));
}

GeoDataset::Descriptor GeoDataset::descriptor() const
{
    Descriptor d;
    d.extents = extents();
    d.size = size();
    d.srs = srs();

    d.bands = numChannels_;
    if (d.bands) {
        auto *b(dset_->GetRasterBand(1));
        d.overviews = b->GetOverviewCount();
        d.dataType = b->GetRasterDataType();
        d.maskType = b->GetMaskFlags();
    }
    return d;
}

GDALRasterBand* GeoDataset::createPerDatasetMaskImpl()
{
    dset_->CreateMaskBand(GMF_PER_DATASET);
    return dset_->GetRasterBand(1)->GetMaskBand();
}

std::vector<fs::path> GeoDataset::files() const
{
    struct CSLDestroyDeleter {
        void operator()(char **ptr) const { ::CSLDestroy(ptr); }
    };
    std::unique_ptr<char*, CSLDestroyDeleter> tmp(dset_->GetFileList());

    std::vector<fs::path> files;

    for (auto head(tmp.get()); *head; ++head) {
        files.emplace_back(*head);
    }

    return files;
}

ValueTransformation GeoDataset::valueTransformation(int bandIndex) const
{
    auto band(dset_->GetRasterBand(bandIndex + 1));
    ValueTransformation t;
    t.offset = band->GetOffset();
    t.scale = band->GetScale();
    return t;
}

ValueTransformation::list GeoDataset::valueTransformation() const
{
    int count = dset_->GetRasterCount();
    ValueTransformation::list list(count);
    for(int i = 0; i < count; ++i) {
        list[i] = valueTransformation(i);
    }

    return list;
}

} // namespace geo
