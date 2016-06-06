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

UTILITY_GENERATE_ENUM_IO(::GDALDataType,
                         ((GDT_Byte)("byte"))
                         ((GDT_UInt16)("uint16"))
                         ((GDT_Int16)("int16"))
                         ((GDT_UInt32)("uint32"))
                         ((GDT_Int32)("int32"))
                         ((GDT_Float32)("float32"))
                         ((GDT_Float64)("float64"))
                         ((GDT_CInt16)("cint16"))
                         ((GDT_CInt32)("cint32"))
                         ((GDT_CFloat32)("cfloat32"))
                         ((GDT_CFloat64)("cfloat64"))
                         )

#endif // geo_gdal_hpp_included
