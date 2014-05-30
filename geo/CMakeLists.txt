define_module(LIBRARY geo DEPENDS imgproc math dbglog
  Boost_FILESYSTEM PROJ GDAL GEOGRAPHICLIB)

set(geo_SOURCES
    dem.hpp dem.cpp
    srsdef.hpp srsdef.cpp
    project.hpp project.cpp
    csconvertor.hpp csconvertor.cpp
    heightconvertor.hpp heightconvertor.cpp
    geodataset.hpp geodataset.cpp
    coordinates.hpp
    io.hpp

    detail/srs.hpp
)

add_library(geo STATIC ${geo_SOURCES})
buildsys_library(geo)

target_link_libraries(geo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo ${MODULE_DEFINITIONS})

# test stuff
if (COMPILE_TESTS)
  add_subdirectory(test-hconv)
endif()
