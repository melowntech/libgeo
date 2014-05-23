#ifndef geo_io_hpp_included_
#define geo_io_hpp_included_

#include "srsdef.hpp"
#include <fstream>
#include "utility/streams.hpp"
#include "dbglog/dbglog.hpp"

namespace geo {

/**
 * @brief Writes srs to a given file.
 * @detail Srs is saved in format specified in 'type' .
 */
inline void writeSrs( const boost::filesystem::path &path
                    , const SrsDefinition & srs
                    , const SrsDefinition::Type &type
                        = SrsDefinition::Type::wkt )
{
    std::ofstream f( path.native() );
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    f << srs.as(type).srs;

    f.close();
}

/**
 * @brief Reads srs from file
 * @detail The content of file is expected to have type 'type'
 */
inline geo::SrsDefinition readSrs( const boost::filesystem::path &path
                                 , const SrsDefinition::Type &type
                                        = SrsDefinition::Type::wkt )
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

/**
 * @brief Saves transformation from image coordinates to world coordinates.
 * @detail PixelSize is in map units (meters), therefore height
 *         is usually negative.
 */
inline void writeTfw( const boost::filesystem::path &path
                    , const math::Extents2 &extents
                    , const math::Size2f &pixelSize = math::Size2f(1.0,-1.0)
                    , const bool pixelReg = false)
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

/**
 * @brief Saves transformation from gdal geo transformation
 */
inline void writeTfwFromGdal(const boost::filesystem::path &path
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

} // namespace geo

#endif // geo_io_hpp_included_

