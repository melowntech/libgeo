/**
  * @file gdal.hpp
  * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
  *
  * GDAL-specific stuff
  */

#ifndef geo_gdal_hpp_included
#define geo_gdal_hpp_included

#include <boost/lexical_cast.hpp>

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

#endif // geo_gdal_hpp_included
