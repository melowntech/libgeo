/**
  * @file gdal.hpp
  * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
  *
  * GDAL-specific stuff
  */

#ifndef geo_gdal_hpp_included
#define geo_gdal_hpp_included

#include <boost/lexical_cast.hpp>

#include <gdal/gdal.h>

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

/*! Types of color interpretation for raster bands. */
UTILITY_GENERATE_ENUM_IO_CI(::GDALColorInterp,
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
