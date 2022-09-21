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
#ifndef geo_io_hpp_included_
#define geo_io_hpp_included_

#include "srsdef.hpp"

namespace geo {

/**
 * @brief Writes srs to a given file.
 * @detail Srs is saved in format specified in 'type' .
 */
void writeSrs( const boost::filesystem::path &path
                    , const SrsDefinition & srs
                    , const SrsDefinition::Type &type
                        = SrsDefinition::Type::wkt );

/**
 * @brief Write SRS to GDAL PAM file
 * &detail When this file is available next to a GeoTIFF, it overrides
 * SRS in that GeoTIFF. GDAL resolves this override automatically.
 * @param path Path to the PAM file.
 * @param srs SRS to be written.
 */
void writeSrsToGdalPam( const boost::filesystem::path &path
                             , const SrsDefinition & srs);

/**
 * @brief Reads srs from file
 * @detail The content of file is expected to have type 'type'
 */
geo::SrsDefinition readSrs( const boost::filesystem::path &path
                                 , const SrsDefinition::Type &type
                                        = SrsDefinition::Type::wkt );

/**
 * @brief Reads srs from file using fromString, i.e. SRS type is deduced from
 * content itself
 */
geo::SrsDefinition srsFromFile(const boost::filesystem::path &path);

/**
 * @brief Saves transformation from image coordinates to world coordinates.
 * @detail PixelSize is in map units (meters), therefore height
 *         is usually negative.
 */
void writeTfw( const boost::filesystem::path &path
                    , const math::Extents2 &extents
                    , const math::Size2f &pixelSize = math::Size2f(1.0,-1.0)
                    , const bool pixelReg = false);

/**
 * @brief Saves transformation from gdal geo transformation
 */
void writeTfwFromGdal(const boost::filesystem::path &path
                             , std::array<double, 6> gdalGeoTrafo);

} // namespace geo

#endif // geo_io_hpp_included_
