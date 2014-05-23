#include <iostream>
#include <stdexcept>

#include <boost/test/unit_test.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "geo/geodataset.hpp"

const geo::SrsDefinition utm33
(R"XXX(PROJCS["UTM Zone 33, Northern Hemisphere"
,GEOGCS["WGS 84"
,DATUM["WGS_1984"
,SPHEROID["WGS 84",6378137,298.257223563
,AUTHORITY["EPSG","7030"]]
,TOWGS84[0,0,0,0,0,0,0]
,AUTHORITY["EPSG","6326"]]
,PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]]
,UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9108"]]
,AUTHORITY["EPSG","4326"]]
,PROJECTION["Transverse_Mercator"]
,PARAMETER["latitude_of_origin",0]
,PARAMETER["central_meridian",15]
,PARAMETER["scale_factor",0.9996]
,PARAMETER["false_easting",500000]
,PARAMETER["false_northing",0]
,UNIT["Meter",1]])XXX"
, geo::SrsDefinition::Type::wkt);

BOOST_AUTO_TEST_CASE(geo_generate_dataset)
{
    math::Extents2 extents(586464, 5457440, 590560, 5463584);

    auto ds(geo::GeoDataset::create
            ("./test-gdal-dataset.tif"
             , utm33
             , extents
             , { 1024, 1536 }
             , geo::GeoDataset::Format::gtiffRGBPhoto()
             , 0));

    LOG(info3) << "Original extents: " << std::setprecision(15) << extents;
    LOG(info3) << "Dataset extents: " << std::setprecision(15) << ds.extents();

    BOOST_CHECK_EQUAL(extents, ds.extents());

    // one third of extents
    auto bsize(size(extents) / 3.);

    cv::Scalar color[9] = {
        {0, 0, 255} // red
        , {0, 255, 0} // green
        , {255, 0, 0} // blue
        , {0, 255, 255} // yellow
        , {255, 0, 255} // magenta
        , {255, 255, 0} // cyan
        , {255, 255, 255} // white
        , {128, 128, 128} // gray
        , {0, 0, 0} // black
    };

    for (int j : { 0, 1, 2 }) {
        for (int i : { 0, 1, 2 }) {
            cv::rectangle(ds.data()
                          , ds.geo2raster<cv::Point>
                          (extents.ll(0) + i * bsize.width
                           , extents.ll(1) + j * bsize.height)
                          , ds.geo2raster<cv::Point>
                          (extents.ll(0) + (i + 1) * bsize.width
                           , extents.ll(1) + (j + 1) *bsize.height)
                          , color[(i + 3 * j)]
                          , CV_FILLED, 4);
        }
    }

    ds.flush();
}
