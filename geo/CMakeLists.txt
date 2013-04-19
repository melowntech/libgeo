define_module(LIBRARY geo DEPENDS cvision imgproc math dbglog
  Boost_FILESYSTEM GDAL)

set(geo_SOURCES
    dem.hpp dem.cpp
)

add_library(geo STATIC ${geo_SOURCES})
set_target_properties(geo PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(geo ${MODULE_LIBRARIES})
