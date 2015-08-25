set(geo_EXTRA_DEPENDS)
set(geo_EXTRA_DEFINITIONS)
if(Boost_PROGRAM_OPTIONS_FOUND)
  list(APPEND geo_EXTRA_DEPENDS Boost_PROGRAM_OPTIONS)
  list(APPEND geo_EXTRA_DEFINITIONS GEO_HAS_PROGRAM_OPTIONS=1)
endif()


define_module(LIBRARY geo DEPENDS geometry imgproc math utility dbglog
  Boost_FILESYSTEM PROJ GDAL GEOGRAPHICLIB ${geo_EXTRA_DEPENDS}
  DEFINITIONS ${geo_EXTRA_DEFINITIONS}
  )

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

    srsfactorsfwd.hpp
    srsfactors.hpp srsfactors.cpp
    verticaladjuster.hpp verticaladjuster.cpp

    detail/pjfactors.h detail/pjfactors.c
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

add_subdirectory(test-factors)
