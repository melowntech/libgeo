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
/**
  * @file gdal.hpp
  * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
  *
  * GDAL-specific stuff
  */

#ifndef geo_gdal_hpp_included
#define geo_gdal_hpp_included

#include <boost/lexical_cast.hpp>

#include <gdal.h>

#include "utility/enum-io.hpp"

namespace geo {

struct Gdal {
    static void setOption(const std::string &name, const std::string &value
                           , bool thisTreadOnly = false);
    static void setOption(const std::string &name, bool value
                           , bool thisTreadOnly = false);

    template <typename T>
    static void setOption(const std::string &name, const T &value
                           , bool thisTreadOnly = false);
};

// inlines

inline void Gdal::setOption(const std::string &name, bool value
                            , bool thisTreadOnly)
{
    setOption(name, (value ? "TRUE" : "FALSE"), thisTreadOnly);
}

template <typename T>
inline void Gdal::setOption(const std::string &name, const T &value
                            , bool thisTreadOnly)

{
    setOption(name, boost::lexical_cast<std::string>(value)
              , thisTreadOnly);
}

} // namespace geo

#if GDAL_VERSION_NUM >= 3050000
UTILITY_GENERATE_ENUM_IO_CI(::GDALDataType,
                            ((GDT_Byte)("Byte"))
                            ((GDT_UInt16)("UInt16"))
                            ((GDT_Int16)("Int16"))
                            ((GDT_UInt32)("UInt32"))
                            ((GDT_Int32)("Int32"))
                            ((GDT_UInt64)("UInt64"))
                            ((GDT_Int64)("Int64"))
                            ((GDT_Float32)("Float32"))
                            ((GDT_Float64)("Float64"))
                            ((GDT_CInt16)("CInt16"))
                            ((GDT_CInt32)("CInt32"))
                            ((GDT_CFloat32)("CFloat32"))
                            ((GDT_CFloat64)("CFloat64"))
                            ((GDT_Unknown)("Unknown"))
                            ((GDT_TypeCount)("TypeCount"))
                            )
#else
UTILITY_GENERATE_ENUM_IO_CI(::GDALDataType,
                            ((GDT_Byte)("Byte"))
                            ((GDT_UInt16)("UInt16"))
                            ((GDT_Int16)("Int16"))
                            ((GDT_UInt32)("UInt32"))
                            ((GDT_Int32)("Int32"))
                            ((GDT_Float32)("Float32"))
                            ((GDT_Float64)("Float64"))
                            ((GDT_CInt16)("CInt16"))
                            ((GDT_CInt32)("CInt32"))
                            ((GDT_CFloat32)("CFloat32"))
                            ((GDT_CFloat64)("CFloat64"))
                            ((GDT_Unknown)("Unknown"))
                            ((GDT_TypeCount)("TypeCount"))
                            )
#endif // GDAL_VERSION_NUM

/*! Types of color interpretation for raster bands. */
UTILITY_GENERATE_ENUM_IO_CI(::GDALColorInterp,
                            ((GCI_Undefined)("Undefined"))
                            ((GCI_GrayIndex)("GrayIndex"))
                            ((GCI_PaletteIndex)("PaletteIndex"))
                            ((GCI_RedBand)("RedBand"))
                            ((GCI_GreenBand)("GreenBand"))
                            ((GCI_BlueBand)("BlueBand"))
                            ((GCI_AlphaBand)("AlphaBand"))
                            ((GCI_HueBand)("HueBand"))
                            ((GCI_SaturationBand)("SaturationBand"))
                            ((GCI_LightnessBand)("LightnessBand"))
                            ((GCI_CyanBand)("CyanBand"))
                            ((GCI_MagentaBand)("MagentaBand"))
                            ((GCI_YellowBand)("YellowBand"))
                            ((GCI_BlackBand)("BlackBand"))
                            ((GCI_YCbCr_YBand)("YBand"))
                            ((GCI_YCbCr_CbBand)("CbBand"))
                            ((GCI_YCbCr_CrBand)("CrBand"))
                            )

#endif // geo_gdal_hpp_included
