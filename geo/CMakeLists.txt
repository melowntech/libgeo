define_module(LIBRARY geo DEPENDS cvision imgproc math dbglog
  Boost_FILESYSTEM PROJ GDAL GEOGRAPHICLIB)

set(geo_SOURCES
    dem.hpp dem.cpp
    srsdef.hpp srsdef.cpp
    project.hpp project.cpp
    csconvertor.hpp csconvertor.cpp
    heightconvertor.hpp heightconvertor.cpp
    geodataset.hpp geodataset.cpp
    coordinates.hpp

    detail/srs.hpp
)

add_library(geo STATIC ${geo_SOURCES})
buildsys_library(geo)

target_link_libraries(geo ${MODULE_LIBRARIES})
