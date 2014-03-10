define_module(LIBRARY geo DEPENDS cvision imgproc math dbglog
  Boost_FILESYSTEM PROJ GDAL)

set(geo_SOURCES
    dem.hpp dem.cpp
    srsdef.hpp srsdef.cpp
    project.hpp project.cpp
    csconvert.hpp csconvert.cpp

    detail/srs.hpp
)

add_library(geo STATIC ${geo_SOURCES})
buildsys_library(geo)

target_link_libraries(geo ${MODULE_LIBRARIES})
