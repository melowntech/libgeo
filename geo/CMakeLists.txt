# bump version here
set(geo_VERSION 1.28)

set(geo_EXTRA_DEPENDS)
set(geo_EXTRA_DEFINITIONS)
if(Boost_PROGRAM_OPTIONS_FOUND)
  list(APPEND geo_EXTRA_DEPENDS Boost_PROGRAM_OPTIONS)
  list(APPEND geo_EXTRA_DEFINITIONS GEO_HAS_PROGRAM_OPTIONS=1)
endif()

set(geo_OPENCV_SOURCES)
if(OpenCV_FOUND)
  list(APPEND geo_EXTRA_DEPENDS OpenCV)
  list(APPEND geo_EXTRA_DEFINITIONS GEO_HAS_OPENCV=1)
  message(STATUS "geo: compiling in OpenCV support")

  # OpenCV sources go here
  set(geo_OPENCV_SOURCES
    geodataset.hpp geodataset.cpp
    dem.hpp dem.cpp
    heightcoding.hpp heightcoding.cpp heightcoding-io.cpp
    featurelayers.hpp featurelayers.cpp
  )
else()
  message(STATUS "geo: compiling without OpenCV support")
endif()


define_module(LIBRARY geo=${geo_VERSION}
  DEPENDS
  jsoncpp>=2.1 geometry>=1.7 imgproc>=1.17 math>=1.2
  utility>=1.11 dbglog>=1.4

  Boost_FILESYSTEM PROJ GDAL GEOGRAPHICLIB ${geo_EXTRA_DEPENDS}
  DEFINITIONS ${geo_EXTRA_DEFINITIONS}
  )

set(geo_SOURCES
    srsdef.hpp srsdef.cpp
    srs.hpp
    enu.hpp enu.cpp
    project.hpp project.cpp
    csconvertor.hpp csconvertor.cpp
    heightconvertor.hpp heightconvertor.cpp
    coordinates.hpp
    io.hpp
    geotransform.hpp geotransform.cpp
    detail/srs.hpp detail/srs.cpp

    srsfactorsfwd.hpp
    srsfactors.hpp srsfactors.cpp
    verticaladjuster.hpp verticaladjuster.cpp

    detail/pjfactors-4.7.h
    detail/pjfactors-4.8.h
    detail/gdalwarpoperation.hpp detail/gdalwarpoperation.cpp
    detail/warpmemorymeter.hpp detail/warpmemorymeter.cpp

    gdal.hpp gdal.cpp

    vectorformat.hpp vectorformat.cpp

    ${geo_OPENCV_SOURCES}
)

# gdalwrapper.h contains __unused__ attribute in newer versions
set_source_files_properties(geodataset.cpp PROPERTIES
  COMPILE_FLAGS -Wno-attributes)

add_library(geo STATIC ${geo_SOURCES})
buildsys_library(geo)

target_link_libraries(geo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo ${MODULE_DEFINITIONS})

# test stuff
if(MODULE_service_FOUND)
  add_subdirectory(test-factors EXCLUDE_FROM_ALL)
  add_subdirectory(test-hconv EXCLUDE_FROM_ALL)
  add_subdirectory(test-srs EXCLUDE_FROM_ALL)
  add_subdirectory(test-geoid EXCLUDE_FROM_ALL)
  add_subdirectory(test-heightmodel EXCLUDE_FROM_ALL)
endif()
