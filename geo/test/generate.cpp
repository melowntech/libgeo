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

#include <iostream>
#include <stdexcept>

#include <boost/test/unit_test.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "imgproc/fillrect.hpp"
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
             , 0
             , geo::GeoDataset::CreateOptions
             ("COMPRESS", "LZW")("BIGTIFF", "IF_NEEDED")
             ));

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
            imgproc::fillRectangle
                (ds.data()
                 , ds.geo2raster<cv::Point>
                 (extents.ll(0) + i * bsize.width
                  , extents.ll(1) + j * bsize.height)
                 , ds.geo2raster<cv::Point>
                 (extents.ll(0) + (i + 1) * bsize.width
                  , extents.ll(1) + (j + 1) *bsize.height)
                 , color[(i + 3 * j)]);
        }
    }

    ds.flush();
}
