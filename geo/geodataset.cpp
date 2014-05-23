/*
 * @file geodataset.cpp
 */

#include <cassert>

#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem/path.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <gdalwarper.h>

#include "utility/expect.hpp"
#include "utility/path.hpp"
#include "math/math.hpp"

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

GeoDataset::GeoTransform buildGeoTransform(const math::Extents2 &extents
                                           , const math::Size2 &size)
{
    GeoDataset::GeoTransform geoTrafo;

    geoTrafo[0] = extents.ll[0];
    geoTrafo[1] = ( extents.ur[0] - extents.ll[0] ) / size.width;
    geoTrafo[2] = 0.0;
    geoTrafo[3] = extents.ur[1];
    geoTrafo[4] = 0.0;
    geoTrafo[5] = ( extents.ll[1] - extents.ur[1] ) / size.height;

    return geoTrafo;
}

math::Point2 applyGeoTransform(const GeoDataset::GeoTransform &trafo
                               , double col, double row )
{

    math::Point2 retval;

    retval[0] = trafo[0] + col * trafo[1] + row * trafo[2];
    retval[1] = trafo[3] + col * trafo[4] + row * trafo[5];

    return retval;
}

void applyInvGeoTransform(const GeoDataset::GeoTransform &trafo
                          , const math::Point2 & gp
                          , double & col, double & row )
{

    double det = trafo[1] * trafo[5] - trafo[2] * trafo[4];

    col = ( (gp[0]-trafo[0])*trafo[5] - (gp[1]-trafo[3])*trafo[2] ) / det;
    row = ( trafo[1]*(gp[1]-trafo[3]) - trafo[4]*(gp[0]-trafo[0]) ) / det;
}

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

GeoDataset::GeoDataset(std::unique_ptr<GDALDataset> &&dset
                       , bool freshlyCreated)
    : type_(Type::custom), dset_(std::move(dset)), changed_(false)
{
    // size & extents
    dset_->GetGeoTransform( geoTransform_.data() );

    if ( geoTransform_[2] != 0 || geoTransform_[4] != 0 )
        LOGTHROW( err3, std::runtime_error ) <<
            "Non-orthogonal affine transforms within datasets "
            "are not supported.";

    size_ = math::Size2i( dset_->GetRasterXSize(), dset_->GetRasterYSize() );

    extents_.ll = applyGeoTransform( geoTransform_, 0, size_.height );
    extents_.ur = applyGeoTransform( geoTransform_, size_.width, 0 );

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
            // check for grayscale
            if (dset_->GetRasterBand(1)->GetColorInterpretation()
                == GCI_GrayIndex)
            {
                type_ = Type::grayscale;
            }
        }
    }

    if (freshlyCreated) {
        // freshly created -> create initial data
        data_ = cv::Mat(size_.height, size_.width, CV_64FC(numChannels_));
        mask_ = RasterMask(size_.width, size_.height, RasterMask::FULL);
    }
}

GeoDataset::~GeoDataset()
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
        const GeoDataset & source, const SrsDefinition &srs,
        boost::optional<math::Size2i> size,
        const math::Extents2 &extents )
{
    std::string srsProj4(srs.as(SrsDefinition::Type::proj4).srs);

    if (!size) {
        auto esize(math::size(extents));
        auto res(source.resolution());
        size = boost::in_place(esize.width / res(0), esize.height / res(1));
    }

    GDALDataset * sdset( const_cast<GDALDataset *>( source.dset_.get() ) );

    // obtain driver
    GDALDriver * driver = GetGDALDriverManager()->GetDriverByName( "MEM" ); //"GTiff" );

    ut::expect( driver, "Failed to initialize in memory driver." );

    ut::expect( source.dset_->GetRasterCount() >= 1,
        "No bands exist in input dataset." );

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
        size->width,
        size->height,
        sdset->GetRasterCount(), dstDataType, nullptr ));

    ut::expect( tdset.get(), "Failed to create in memory dataset.\n" );

    for ( int i = 1; i <= tdset->GetRasterCount(); i++ ) {

        tdset->GetRasterBand(i)->SetColorInterpretation(
            sdset->GetRasterBand(i)->GetColorInterpretation() );
        tdset->GetRasterBand(i)->SetNoDataValue( dstNoDataValue );
    }

    auto geoTrafo(buildGeoTransform(extents, *size));

    tdset->SetGeoTransform( geoTrafo.data() );
    tdset->SetProjection( proj4ToWkt( srsProj4 ).c_str() );

    // all done
    return GeoDataset(std::move(tdset));
}


GeoDataset GeoDataset::create(const boost::filesystem::path &path
                              , const SrsDefinition &srs
                              , const math::Extents2 &extents
                              , const math::Size2 &rasterSize
                              , const Format &format
                              , double noDataValue)
{
    if (!initialized_) { initialize(); }

    std::string storageFormat;
    std::string wfExt;

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

    char **options(nullptr);

    std::unique_ptr<GDALDataset>
        ds(driver->Create(path.string().c_str()
                          , rasterSize.width
                          , rasterSize.height
                          , format.channels.size()
                          , format.channelType
                          , options));
    ut::expect(ds.get(), "Failed to create new dataset.");

    // set projection
    if (ds->SetProjection(srs.as(SrsDefinition::Type::wkt).c_str())
        != CE_None)
    {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to set projection to newly created GDAL data set.";
    }

    auto geoTrafo(buildGeoTransform(extents, rasterSize));

    // set geo trafo
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

    // write .prj file
    geo::writeSrs(utility::replaceOrAddExtension(path, "prj"), srs);

    // write world file
    geo::writeTfwFromGdal(utility::replaceOrAddExtension(path, wfExt)
                          , geoTrafo);

    // OK, tell ctor that dset was freshly created
    return GeoDataset(std::move(ds), true);
}


void GeoDataset::warpInto( GeoDataset & dst, Resampling alg ) const {

    // sanity
    ut::expect( dset_->GetRasterCount() == dst.dset_->GetRasterCount()
                && dset_->GetRasterCount() > 0,
            "Error in warp: both dataset need to have the same number of bands." );

    // create warp options
    GDALWarpOptions * warpOptions = GDALCreateWarpOptions();

    warpOptions->hSrcDS = const_cast<GDALDataset *>( dset_.get() );
    warpOptions->hDstDS = const_cast<GDALDataset *>( dst.dset_.get() );

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

    switch( alg ) {
#if 0
        case average:
            warpOptions->eResampleAlg = GRA_Average; break;
#endif
        case lanczos:
        default:
            warpOptions->eResampleAlg = GRA_Lanczos; break;
    }

    warpOptions->pfnProgress = GDALDummyProgress;

    warpOptions->papszWarpOptions = CSLSetNameValue(
        warpOptions->papszWarpOptions, "INIT_DEST", "NO_DATA" );

    // establish reprojection transformer.
    warpOptions->pTransformerArg =
        GDALCreateGenImgProjTransformer( warpOptions->hSrcDS,
                                         dset_->GetProjectionRef(),
                                         warpOptions->hDstDS,
                                         dst.dset_->GetProjectionRef(),
                                         true, 0, 1 );
    warpOptions->pfnTransformer = GDALGenImgProjTransform;

    // initialize and execute the warp operation.
    GDALWarpOperation oOperation;

    oOperation.Initialize( warpOptions );
    CPLErr err = oOperation.ChunkAndWarpImage( 0, 0,
                                  dst.dset_->GetRasterXSize(),
                                  dst.dset_->GetRasterYSize() );

    ut::expect( err == CE_None, "Warp failed." );

    GDALDestroyGenImgProjTransformer( warpOptions->pTransformerArg );
    GDALDestroyWarpOptions( warpOptions );

    // invalidate local copies of target dataset content
    dst.data_ = boost::none; dst.mask_ = boost::none;

    // done
}

void GeoDataset::assertData() const {

#ifdef _OPENMP
#   pragma omp critical
#endif
    if ( ! mask_ || ! data_ ) {
        loadData();
    }
}

void GeoDataset::loadData() const {

    cv::Mat raster;

    // sanity
    ut::expect( dset_->GetRasterCount() > 0 );

    // determine matrix data type
    GDALDataType gdalDataType = dset_->GetRasterBand(1)->GetRasterDataType();
    int numChannels = dset_->GetRasterCount();
    int cvDataType(gdal2cv(gdalDataType, numChannels));

    // create matrix
    raster.create( size_.height, size_.width, cvDataType );
    int valueSize = raster.elemSize() / raster.channels();

    // transfer data
    CPLErr err( CE_None );

    for ( int i = 1; i <= numChannels; i++ ) {

        int bandMap( i );

        err = dset_->RasterIO(
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

    }

    ut::expect( err == CE_None, "Reading of raster data failed." );

    // convert
    data_ = cv::Mat();
    raster.convertTo( data_.get(), CV_64FC( numChannels ) );

    // establish mask
    mask_ = RasterMask( size_.width,size_.height,RasterMask::FULL );

    if ( noDataValue_ ) {

        double * curpos = reinterpret_cast<double *>( data_->data );

        for ( int i = 0; i < size_.height; i++ )
            for ( int j = 0; j < size_.width; j++ )
                for ( int k = 0; k < numChannels_; k++ )
                    if ( *curpos++ == noDataValue_.get() )
                        mask_->set(j,i,false);
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


math::Point3 GeoDataset::rowcol2geo( int row, int col, double value ) const {

    math::Point2 p2 = applyGeoTransform( geoTransform_, col + 0.5, row + 0.5 );

    return math::Point3( p2[0], p2[1], value );
}

void GeoDataset::geo2rowcol(
    const math::Point3 & gp, double & row, double & col ) const {

   math::Point2 gp2( gp[0], gp[1] );

   applyInvGeoTransform( geoTransform_, gp2, col, row );
   row -= 0.5, col -= 0.5;
}

void GeoDataset::exportMesh( geometry::Mesh & mesh ) const {

    std::map<std::pair<int,int>, int> vpos2ord; // i,j -> vertex ordinal

    // expect a single gray channel
    expectGray();

    // obtain heightfield data
    assertData();
    
    // dump vertices
    int ord( 0 );

    math::Matrix4 localTrafo = geo2local( extents_ );

    for ( int i = 0; i < size_.height; i++ )
        for ( int j = 0; j < size_.width; j++ ) {

            //LOG( debug ) << "row " << i << ", col " << j << ": " <<
            //    mdsm.at<double>( i, j );

            // check if vertex is defined in dsm
            if ( ! valid( i, j ) ) continue;

            // vertex coordinates
            math::Point3 pvertex;
            math::Point3 geoVertex;
            
            geoVertex = rowcol2geo( i, j, data_->at<double>( i, j ) );

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
        }

    // all done
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

            pvertex = rowcol2geo( i, j, data_->at<double>( i, j ) );

            pc.push_back(pvertex);
        }
    }
    // all done
    return pc;
}

void GeoDataset::textureMesh(
    const geometry::Mesh & imesh, const math::Extents2 & extents,
    geometry::Mesh & omesh ) const {

    std::map<int,int> iord2oord; // i,j -> vertex ordinal
    int ord(0);

    // sanity
    ut::expect( omesh.vertices.size() == 0 && omesh.tCoords.size() == 0 &&
        omesh.faces.size() == 0, "Output mesh expected to be empty." );

    // establish input local -> dset normalized
    math::Matrix4 il2dn = prod(
        geo2normalized( extents_ ), local2geo( extents ) );
    math::Matrix4 il2geo
        = local2geo( extents );

    // iterate through vertices
    for ( uint i = 0; i < imesh.vertices.size(); i++ ) {

        const math::Point3 & vertex( imesh.vertices[i] );

        // skip non-texturable vertices
        double row, col;

        geo2rowcol( transform( il2geo, vertex ), row, col );

        if ( ! validf( row, col ) ) continue;

        // normalized coords are in (-1.0,1.0) range -> transform to (0.0,1.0)
        math::Point3 tcoord3 = transform( il2dn, vertex ) * 0.5
            + math::Point3( 0.5, 0.5, 0.0 );

        math::Point2 tcoord( tcoord3[0], tcoord3[1] );


        // write out vertex and tcoords
        omesh.vertices.push_back( vertex );
        omesh.tCoords.push_back( math::Point2( tcoord[0], tcoord[1] ) );
        iord2oord[i] = ord++;
    }

    // iterate though faces
    for ( geometry::Face face : imesh.faces ) {

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

math::Extents2 GeoDataset::deriveExtents( const SrsDefinition &srs )
{
    std::string srsProj4(srs.as(SrsDefinition::Type::proj4).srs);

    CPLErr err;

    void * transformer =
        GDALCreateGenImgProjTransformer(
            dset_.get(),
            srsWkt_.c_str(),
            nullptr,
            proj4ToWkt( srsProj4 ).c_str(),
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


    if ( outputTransform[2] != 0.0 || outputTransform[4] != 0.0 ) {

        LOGTHROW( err2, std::runtime_error )
            << "Output SRS leads to shear/rotation.";
    }

    math::Extents2 retval;
    
    math::Point2 ll, lr, ul, ur;
    
    ll = applyGeoTransform( outputTransform, 0, outputSize.height );
    lr = applyGeoTransform( outputTransform, outputSize.width, outputSize.height );
    ul = applyGeoTransform( outputTransform, 0, 0 );
    ur = applyGeoTransform( outputTransform, outputSize.width, 0 );
    

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
            cv::Point2i end((xstart + xsize), (ystart + ysize));

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

} // namespace geo
