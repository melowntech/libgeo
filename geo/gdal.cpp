#include <cpl_conv.h>

#include "./gdal.hpp"

namespace geo {

void Gdal::setOption(const std::string &name, const std::string &value
                     , bool thisTreadOnly)
{
    if (thisTreadOnly) {
        ::CPLSetThreadLocalConfigOption(name.c_str(), value.c_str());
    } else {
        ::CPLSetConfigOption(name.c_str(), value.c_str());
    }
}

} // namespace geo
