#include <stdexcept>
#include <tuple>

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include "dbglog/dbglog.hpp"
#include "imgproc/rastermask.hpp"

#include "dem.hpp"

namespace geo {

namespace {

// TODO: read from DEM if present
const float INVALID_DEM_VALUE(0.f);

namespace fs = boost::filesystem;

struct GdalInit {
    GdalInit() { GDALAllRegister(); }
    ~GdalInit() { GDALDestroyDriverManager(); }
};

template<typename Out> struct DemOutput;

template<> struct DemOutput<cv::Mat> {
    DemOutput(int width, int height, float scale, float offset)
        : value(width, height, CV_32F), scale(scale), offset(offset)
    {
        LOG(info2) << "[DEM] height = value * " << scale << " + " << offset;
    }

    void copyRow(const float *irow, const float *erow
                 , int x, int y)
    {
        // get start of block row in out matrix
        auto orow(value.ptr<float>(y) + x);

        // copy data (apply scale and offset)
        std::transform(irow, erow, orow
                       , [scale, offset] (float v) {
                           return (v * scale) + offset;
                       });
    }

    cv::Mat value;
    float scale;
    float offset;
};

template<> struct DemOutput<imgproc::RasterMask> {
    DemOutput(int width, int height, float scale, float offset)
        : value(width, height), scale(scale), offset(offset)
    {}

    void copyRow(const float *irow, const float *erow
                 , int x, int y)
    {
        // process row and set bits for pixels with value 255
        for (; irow < erow; ++irow, ++x) {
            value.set(x, y, (*irow == 255));
        }
    }

    imgproc::RasterMask value;
    float scale;
    float offset;
};

template <typename Out>
Out processBand(GDALRasterBand *band)
{
    if (band->GetRasterDataType() != GDT_Float32) {
        LOGTHROW(err3, std::runtime_error)
            << "DEM TIFF data type is not float!";
    }

    math::Size2i size(band->GetXSize(), band->GetYSize());

    DemOutput<Out> out(size.height, size.width, band->GetScale()
                       , band->GetOffset());

    math::Size2i bsize;
    band->GetBlockSize(&bsize.width, &bsize.height);
    math::Size2i bcount((size.width + bsize.width - 1) / bsize.width
                        , (size.height + bsize.height - 1) / bsize.height);

    LOG(info2) << "[DEM] band size: " << size;
    LOG(info2) << "[DEM] band block size: " << bsize;
    LOG(info2) << "[DEM] band block count: " << bcount;

    boost::scoped_array<float> block(new float[bsize.width * bsize.height]);

    for (int by(0), y(0); by < bcount.height; ++by, y += bsize.height) {
        for (int bx(0), x(0); bx < bcount.width; ++bx, x += bsize.width) {
            // calculate valid area of current block
            math::Size2i range
                ((((bx + 1) == bcount.width)
                  ? (size.width % bsize.width) : bsize.width)
                 , (((by + 1) == bcount.height)
                    ? (size.height % bsize.height) : bsize.height));
            if (!range.width) { range.width = bsize.width; }
            if (!range.height) { range.height = bsize.height; }

            LOG(info2) << "processing block: " << range
                       << " at " << x << ", " << y;

            // read block
            band->ReadBlock(bx, by, block.get());

            // copy block to matrix
            for (int yy(0); yy < range.height; ++yy) {
                // get start of block row
                auto irow(block.get() + yy * range.width);
                // and copy it to row in matrix
                out.copyRow(irow, irow + range.width, x, yy + y);
            }
        }
    }

    return out.value;
}

math::Point2 demCorner(GDALDataset *dataset, double x, double y)
{
    double trans[6];
    if (GDALGetGeoTransform(dataset, trans) == CE_None) {
        return {trans[0] + trans[1] * x + trans[2] * y
                , trans[3] + trans[4] * x + trans[5] * y};
    }

    return {x, y};
}

std::tuple<math::Extents2, math::Extents2i>
demRegions(const math::Extents2 &extents, const math::Size2f &px
           , const math::Extents2 &dextents, const cv::Mat &dem)
{
    // compute region of interest (throws when no intersection is found)
    auto region(intersect(extents, dextents));
    LOG(info1) << "region: " << std::setprecision(15) << region;
    // normalize inside DEM extents
    math::Extents2 nregion(region.ll - dextents.ll, region.ur - dextents.ll);
    LOG(info1) << "nregion: " << std::setprecision(15) << nregion;

    // compute extents in pixels and clip by dem dimensions
    return std::tuple<math::Extents2, math::Extents2i>
        (region
         , intersect(math::Extents2i(std::floor(nregion.ll(0) / px.width)
                                     , std::floor(nregion.ll(1) / px.height)
                                     , std::ceil(nregion.ur(0) / px.width)
                                     , std::ceil(nregion.ur(1) / px.height))
                     , math::Extents2i(0, 0, dem.cols, dem.rows)));
}

std::tuple<math::Extents2, math::Extents2i>
demRegions(const math::Extents2 &dextents, const cv::Mat &dem)
{
    return std::tuple<math::Extents2, math::Extents2i>
        (dextents, math::Extents2i(0, 0, dem.cols, dem.rows));
}

std::string projectionReference(GDALDataset *ds)
{
    char *pr;
    OGRSpatialReference(ds->GetProjectionRef()).exportToProj4(&pr);
    std::string res(pr);
    CPLFree(pr);
    return res;
}

DemCloud loadDemImpl(const fs::path &path
                     , const math::Extents2 *extents = nullptr)
{
    GdalInit gdal;

    GDALDataset *dataset
        (static_cast<GDALDataset*>(GDALOpenShared(path.native().c_str()
                                                  , GA_ReadOnly)));

    if (!dataset) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot open DEM file " << path
            << ": <" << CPLGetLastErrorNo() << ", "
            << CPLGetLastErrorMsg() << ">.";
    }

    // get DEM from GeoTIFF
    auto dem(processBand<cv::Mat>(dataset->GetRasterBand(1)));
    // auto mask(processBand<imgproc::RasterMask>(dataset->GetRasterBand(2)));

    math::Extents2 dextents
        (demCorner(dataset, .0, GDALGetRasterYSize(dataset))
         , demCorner(dataset, GDALGetRasterXSize(dataset), .0));


    if (extents) {
        LOG(info1) << "extents: " << std::setprecision(15) << *extents;
    }
    LOG(info1) << "dextents: " << std::setprecision(15) << dextents;

    // compute pixel size
    auto esize(size(dextents));
    LOG(info1) << "esize: " << std::setprecision(15) << esize;
    math::Size2f px(esize.width / dem.cols, esize.height / dem.rows);

    LOG(info1) << "px: " << std::setprecision(15) << px;

    math::Extents2 region;
    math::Extents2i pregion;
    std::tie(region, pregion) = (extents
                                 ? demRegions(*extents, px, dextents, dem)
                                 : demRegions(dextents, dem));

    LOG(info1) << "pregion: " << pregion;

    math::Points3 pc;

    // process all pixels from DEM
    for (int j(pregion.ll(1)), ej(pregion.ur(1)); j < ej; ++j) {
        for (int i(pregion.ll(0)), ei(pregion.ur(0)); i < ei; ++i) {
            auto height(dem.at<float>(j, i));
            if (height == INVALID_DEM_VALUE) { continue; }

            // add point to point cloud; NB half pixel offset
            pc.emplace_back(region.ll(0) + (i + .5) * px.width
                            , region.ll(1) + (j + .5) * px.height
                            , height);
        }
    }

    return { pc, projectionReference(dataset) };
}

} // namespace

DemCloud loadDem(const fs::path &path)
{
    return loadDemImpl(path);
}

DemCloud loadDem(const fs::path &path, const math::Extents2 &extents)
{
    return loadDemImpl(path, &extents);
}

} // namespace geo
