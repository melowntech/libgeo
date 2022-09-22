#include "io.hpp"

#include <fstream>
#include <boost/algorithm/string.hpp>

#include <ogr_spatialref.h>

#include "utility/streams.hpp"
#include "dbglog/dbglog.hpp"

namespace ba = boost::algorithm;

namespace geo
{
void writeSrs( const boost::filesystem::path &path
                    , const SrsDefinition & srs
                    , const SrsDefinition::Type &type)
{
    std::ofstream f( path.native() );
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    f << srs.as(type).srs;

    f.close();
}

void writeSrsToGdalPam( const boost::filesystem::path &path
                             , const SrsDefinition & srs)
{
    if (!ba::ends_with(path.native(), ".aux.xml"))
    {
        LOGTHROW(err3, std::runtime_error) << "Invalid GDAL PAM file path: " 
                                           << path;
    }
    std::ofstream f( path.native() );
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    const bool isVertical = srs.reference().IsVertical();
    const std::string axes = isVertical ? "1,2,3" : "1,2";
    f << "<PAMDataset>" << std::endl; 
    f << "<SRS dataAxisToSRSAxisMapping=\"" << axes << "\">" 
      << srs.toString() << "</SRS>" << std::endl;
    f << "</PAMDataset>" << std::endl;

    f.close();
}

geo::SrsDefinition readSrs( const boost::filesystem::path &path
                                 , const SrsDefinition::Type &type)
{
    try {
        return { utility::read(path), type };
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to read srs file " << path << ": " << e.what()
            << ".";
    }
    throw;
}

geo::SrsDefinition srsFromFile(const boost::filesystem::path &path)
{
    try {
        return SrsDefinition::fromString(utility::read(path));
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to read srs file " << path << ": " << e.what()
            << ".";
    }
    throw;
}

void writeTfw( const boost::filesystem::path &path
                    , const math::Extents2 &extents
                    , const math::Size2f &pixelSize
                    , const bool pixelReg)
{
    std::ofstream tfw( path.native() );
    tfw.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    double a = pixelSize.width, e = pixelSize.height,
           b = 0, d = 0;
    double c = extents.ll(0) + pixelReg * 0.5 * pixelSize.width;
    double f = extents.ur(1) + pixelReg * 0.5 * pixelSize.height;

    tfw << std::setprecision(15)
        << a << "\n" << d << "\n" << b << "\n"
        << e << "\n" << c << "\n" << f << "\n";

    tfw.close();
}

void writeTfwFromGdal(const boost::filesystem::path &path
                             , std::array<double, 6> gdalGeoTrafo)
{
    std::ofstream tfw(path.native());
    tfw.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    gdalGeoTrafo[0] += .5 * gdalGeoTrafo[1];
    gdalGeoTrafo[3] += .5 * gdalGeoTrafo[5];

    tfw << std::setprecision(15)
        << gdalGeoTrafo[1] << '\n'
        << gdalGeoTrafo[2] << '\n'
        << gdalGeoTrafo[4] << '\n'
        << gdalGeoTrafo[5] << '\n'
        << gdalGeoTrafo[0] << '\n'
        << gdalGeoTrafo[3] << '\n'
        ;

    tfw.close();
}
}
