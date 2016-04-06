/*
 * @file geodataset.cpp
 */

#include <cassert>
#include <sstream>

#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem/path.hpp>

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
            << "Unsupported datatype " << gdalDataType << "in raster.";
    }
    return 0; // never reached
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

GeoDataset::GeoDataset(std::unique_ptr<GDALDataset> &&dset
                       , bool freshlyCreated)
    : type_(Type::custom), dset_(std::move(dset)), changed_(false)
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

    if (freshlyCreated) {
        // freshly created -> create initial data
        data_ = cv::Mat(size_.height, size_.width, CV_64FC(numChannels_));
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
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


GeoDataset GeoDataset::open(const fs::path & path)
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
        const math::Matrix2 & trafo ) {

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
    
    size.width = ( textents.ur[0] - textents.ll[0] ) / lpixelSize(0);
    size.height = ( textents.ur[1] - textents.ll[1] ) / lpixelSize(1);
    
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
    double dstNoDataValue( 0.0 );

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

        case GDT_Float32 :
            dstDataType = GDT_Float64; dstNoDataValue = 1.0E40; break;

        default :
            LOGTHROW( err2, std::runtime_error )
                << "A no data value is compulsory for dataset of type "
                << srcDataType << ". Please patch your data.";
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
        tdset->GetRasterBand(i)->SetNoDataValue( dstNoDataValue );
    }

    tdset->SetGeoTransform( geoTransform.data() );
    tdset->SetProjection( srsWkt.c_str() );

    // all done
    return GeoDataset(std::move(tdset));
}


GeoDataset GeoDataset::deriveInMemory(
        const GeoDataset & source, const SrsDefinition &srs,
        boost::optional<math::Size2i> size,
        const math::Extents2 &extents )
{
    if (!size) {
        auto esize(math::size(extents));
        auto res(source.resolution());
        size = boost::in_place(esize.width / res(0), esize.height / res(1));
    }

    return deriveInMemory( source, srs, math::Point2(
                            ( extents.ur[0] - extents.ll[0] ) / size->width,
                            ( extents.ur[1] - extents.ll[1] ) / size->height ),
                           extents );
}

GeoDataset GeoDataset::create(const boost::filesystem::path &path
                             , const SrsDefinition &srs
                             , const GeoTransform & geoTransform
                             , const math::Size2 &rasterSize
                             , const Format &format
                             , double noDataValue
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
        wfExt = "tfw";
        break;

    case Format::Storage::png:
        storageFormat = "PNG";
        wfExt = "pgw";
        break;

    case Format::Storage::jpeg:
        storageFormat = "JPEG";
        wfExt = "jgw";
        break;

    case Format::Storage::memory:
        storageFormat = "MEM";
        usePath = "MEM";
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
            band->SetNoDataValue(noDataValue);
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
                              , double noDataValue
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
    }

    // falback
    return GRA_Lanczos;
}

double sourcePixelSize(const geo::CsConvertor conv
                       , const geo::GeoDataset &src
                       , const geo::GeoDataset &dst)
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
    const auto srcR1(src.geo2raster<math::Point2>(srcG1));
    const auto srcR2(src.geo2raster<math::Point2>(srcG2));

    // calculate source distance -> size of source pixel
    return boost::numeric::ublas::norm_2(srcR2 - srcR1);
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
bool rightRotating(const math::Points2 &polygon)
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

    return (sign == 2);
}

void sourceExtra(const GeoDataset &src, const GeoDataset &dst
                 , OptionsWrapper &wo)
{
    const geo::CsConvertor conv(dst.srs(), src.srs());

    std::vector<math::Point2> corners;
    auto dstExtents(dst.extents());
    for (const auto &corner : {
            math::ul(dstExtents), math::ur(dstExtents)
            , math::lr(dstExtents), math::ll(dstExtents) })
    {
        corners.push_back(conv(corner));
    }

    if (rightRotating(corners)) {
        // right rotating polygon maps to sane right rotating polygon -> no wrap
        // arround
        return;
    }

    auto ps(sourcePixelSize(conv, src, dst));
    auto s(dst.size());
    ps *= std::max(s.width, s.height);

    // TODO: dynamic limit
    auto se(std::ceil(ps));
    if (se > 1000) {
        se = 1000;
    }

    LOG(info2) << "Setting SOURCE_EXTRA=" << se << ".";
    wo("SOURCE_EXTRA", se);
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

std::unique_ptr<GDALDataset> chooseOverview(GDALWarpOptions *wo)
{
    auto src(static_cast<GDALDataset*>(wo->hSrcDS));

    auto band(src->GetRasterBand(1));
    // get number of overviews and bail out if there are none
    auto count(band->GetOverviewCount());
    if (!count) { return {}; }

    // Compute what the "natural" output resolution (in pixels) would be for
    // this input dataset
    double suggestedGeoTransform[6];
    double extents[4];
    int x, y;
    if (::GDALSuggestedWarpOutput2(src, wo->pfnTransformer
                                   , wo->pTransformerArg
                                   , suggestedGeoTransform
                                   , &x, &y, extents, 0)
        != CE_None)
    {
        return {};
    }

    // calculate target ratio
    double targetRatio(1.0 / suggestedGeoTransform[1]);
    if (targetRatio < 1.0) { return {}; }

    int ovr(-1);
    for (; ovr < count - 1; ++ovr) {
        double thisRatio(1.0);
        if (ovr >= 0) {
            thisRatio = (double(src->GetRasterXSize())
                         / double(band->GetOverview(ovr)->GetXSize()));
        }
        double nextRatio(double(src->GetRasterXSize())
                         / double(band->GetOverview(ovr + 1)->GetXSize()));

        if ((thisRatio < targetRatio) && (nextRatio > targetRatio)) {
            // found
            break;
        }

        if (std::abs(thisRatio - targetRatio) < 1e-1) {
            // close enough
            break;
        }
    }

    // -1 -> original dataset
    if (ovr < 0) { return {}; }

    // use chosen overwiew
    std::unique_ptr<GDALDataset> ovrDs
        (::GDALCreateOverviewDataset(src, ovr, false, false));
    if (!ovrDs) {
        // failed -> go on with original dataset
        return ovrDs;
    }

    // update warp options

    // set new source
    wo->hSrcDS = ovrDs.get();

    // re-create transformer
    createTransformer(wo);

    LOG(info1)
        << "Wapr uses overview #" << ovr << " instead of the full dataset.";

    // and return holder
    return ovrDs;
}

} // namespace

void GeoDataset::warpInto(GeoDataset & dst
                          , const boost::optional<Resampling> &requestedAlg
                          , const Options &options)
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

    // calculate sourceExtra parameter from datasets (if needed)
    sourceExtra(*this, dst, wo);

    // create warp options
    GDALWarpOptions * warpOptions = GDALCreateWarpOptions();

    warpOptions->hSrcDS = dset_.get();
    warpOptions->hDstDS = dst.dset_.get();

    warpOptions->nBandCount = dset_->GetRasterCount();

    warpOptions->panSrcBands =
        (int *) CPLMalloc(sizeof(int) * warpOptions->nBandCount );
    warpOptions->panDstBands =
        (int *) CPLMalloc(sizeof(int) * warpOptions->nBandCount );

    if ( noDataValue_ ) {

        warpOptions->padfSrcNoDataReal =
            (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );
        warpOptions->padfSrcNoDataImag =
        (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );

    }


    if ( dst.noDataValue_ ) {

        warpOptions->padfDstNoDataReal =
            (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );
        warpOptions->padfDstNoDataImag =
            (double *) CPLMalloc(sizeof(double) * warpOptions->nBandCount );
    }


    for ( int i = 0; i < warpOptions->nBandCount; i++ ) {

        warpOptions->panSrcBands[i] = i + 1;
        warpOptions->panDstBands[i] = i + 1;

        if ( noDataValue_ ) {

            warpOptions->padfSrcNoDataReal[i] = noDataValue_.get();
            warpOptions->padfSrcNoDataImag[i] = 0.0;
        }

        if ( dst.noDataValue_ ) {

            warpOptions->padfDstNoDataReal[i] = dst.noDataValue_.get();
            warpOptions->padfDstNoDataImag[i] = 0.0;
        }
    }

    warpOptions->eResampleAlg = algoToGdal(alg);
    warpOptions->pfnProgress = GDALDummyProgress;

    // update options and grab options since it is destroyed by
    // GDALDestroyWarpOptions
    wo("INIT_DEST", "NO_DATA");
    warpOptions->papszWarpOptions = wo.release();

    // establish reprojection transformer.
    createTransformer(warpOptions);

    // choose overview and hold the dataset
    auto ovrDs(chooseOverview(warpOptions));

    // initialize and execute the warp operation.
    GDALWarpOperation oOperation;

    oOperation.Initialize( warpOptions );
    CPLErr err = oOperation.ChunkAndWarpImage( 0, 0,
                                  dst.dset_->GetRasterXSize(),
                                  dst.dset_->GetRasterYSize() );

    GDALDestroyGenImgProjTransformer( warpOptions->pTransformerArg );
    GDALDestroyWarpOptions( warpOptions );

    ut::expect( err == CE_None, "Warp failed." );

    // invalidate local copies of target dataset content
    dst.data_ = boost::none; dst.mask_ = boost::none;

    // done
    //LOG(debug) << "Warped.";
}

void GeoDataset::assertData() const {

    UTILITY_OMP(critical)
    if ( ! mask_ || ! data_ ) {
        loadData();
    }
}

void GeoDataset::loadData() const {
    // sanity
    ut::expect( dset_->GetRasterCount() > 0 );

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

    // establish mask
    mask_ = RasterMask( size_.width,size_.height,RasterMask::FULL );

    if ( noDataValue_ ) {
#if 0
        // get mask band
        auto maskBand(dset_->GetRasterBand(1)->GetMaskBand());
        auto gdalDataType = maskBand->GetRasterDataType();
        auto cvDataType = gdal2cv(gdalDataType, 1);
        ut::expect((cvDataType == CV_8UC1)
                   , "Expected band mask to be of byte type.");
        raster.create(size_.height, size_.width, cvDataType);
        int bandMap(1);

        auto err = dset_->RasterIO
            (GF_Read // GDALRWFlag  eRWFlag,
             , 0, 0 // int nXOff, int nYOff
             , size_.width, size_.height // int nXSize, int nYSize,
             , (void *)(raster.data)  // void * pData,
             , size_.width, size_.height // int nBufXSize, int nBufYSize,
             , gdalDataType // GDALDataType  eBufType,
             , 1 //  int nBandCount,
             , &bandMap  // int * panBandMap,
             , raster.elemSize() // int nPixelSpace,
             , size_.width * raster.elemSize(), 0); // int nLineSpace

        ut::expect((err == CE_None)
                   , "Reading of mask band data failed.");
        const auto *d(raster.data);
        for (int i = 0; i < size_.height; ++i) {
            for (int j = 0; j < size_.width; ++j) {
                if (!*d++) {
                    mask_->set(j, i, false);
                }
            }
        }

#else
        double * curpos = reinterpret_cast<double *>( data_->data );

        for ( int i = 0; i < size_.height; i++ )
            for ( int j = 0; j < size_.width; j++ )
                for ( int k = 0; k < numChannels_; k++ )
                    if ( *curpos++ == noDataValue_.get() )
                        mask_->set(j,i,false);
#endif
    }

    // all done
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

GeoDataset::Metadata GeoDataset::getMetadata(const std::string &domain) const
{
    auto md(dset_->GetMetadata(domain.empty() ? 0x0 : domain.c_str()));
    if (!md) {
        return {};
    }

    Metadata metadata;
    char *key;
    for (; *md; ++md) {
        auto value(::CPLParseNameValue(*md, &key));
        metadata(key, value);
    }

    return metadata;
}

void GeoDataset::setMetadata(const Metadata &metadata
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

    dset_->SetMetadata(md.data(), domain.empty() ? 0x0 : domain.c_str());
}

// rawish interface

GeoDataset::Block GeoDataset::readBlock(const math::Point2i &blockOffset)
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

    Block block;
    block.data.create(size.height, size.width, CV_64FC(numChannels_));
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
             , GDT_Float64 // GDALDataType  eBufType,
             , 1 //  int nBandCount,
             , &bandMap  // int * panBandMap,
             , block.data.elemSize() // int nPixelSpace,
             , size.width * block.data.elemSize(), 0); // int nLineSpace

        ut::expect(err == CE_None, "Reading of raster data failed.");
    }

    return block;
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

} // namespace geo
