define_module(LIBRARY geo DEPENDS geometry imgproc math utility dbglog
  Boost_FILESYSTEM PROJ GDAL GEOGRAPHICLIB)

set(geo_SOURCES
    dem.hpp dem.cpp
    srsdef.hpp srsdef.cpp
    srs.hpp
    project.hpp project.cpp
    csconvertor.hpp csconvertor.cpp
    heightconvertor.hpp heightconvertor.cpp
    geodataset.hpp geodataset.cpp
    coordinates.hpp
    io.hpp
    geotransform.hpp geotransform.cpp
    detail/srs.hpp detail/srs.cpp
)

# gdalwrapper.h contains __unused__ attribute in newer versions
set_source_files_properties(geodataset.cpp PROPERTIES
  COMPILE_FLAGS -Wno-attributes)

add_library(geo STATIC ${geo_SOURCES})
buildsys_library(geo)

target_link_libraries(geo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo ${MODULE_DEFINITIONS})

# test stuff
if (COMPILE_TESTS)
  add_subdirectory(test-hconv)
  add_subdirectory(test-srs)
endif()
