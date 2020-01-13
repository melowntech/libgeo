# bump version here
set(geo_VERSION 1.31)

set(geo_EXTRA_DEPENDS)
set(geo_EXTRA_DEFINITIONS)
if(Boost_PROGRAM_OPTIONS_FOUND)
  list(APPEND geo_EXTRA_DEPENDS Boost_PROGRAM_OPTIONS)
  list(APPEND geo_EXTRA_DEFINITIONS GEO_HAS_PROGRAM_OPTIONS=1)
endif()

set(geo_GDAL_SOURCES)
set(geo_OPENCV_SOURCES)
if(GDAL_FOUND)
  list(APPEND geo_EXTRA_DEPENDS GDAL)
  list(APPEND geo_EXTRA_DEFINITIONS GEO_HAS_GDAL=1)
  message(STATUS "geo: compiling in GDAL support")

  # GDAL sources go here
  set(geo_GDAL_SOURCES
    enu.hpp enu.cpp
    heightconvertor.hpp heightconvertor.cpp
    io.hpp
    srs.hpp
    srsdef.cpp
    csconvertor.cpp
    geotransform.hpp geotransform.cpp
    detail/pjfactors-4.7.h
    detail/pjfactors-4.8.h
    detail/gdalwarpoperation.hpp detail/gdalwarpoperation.cpp
    detail/warpmemorymeter.hpp detail/warpmemorymeter.cpp
    detail/srs.hpp detail/srs.cpp
    gdal.hpp gdal.cpp
    vectorformat.hpp vectorformat.cpp
  )

  if(NOT GDAL_VERSION)
    if(GDAL_CONFIG)
      execute_process(COMMAND "${GDAL_CONFIG}" "--version"
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE GDAL_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )
      if(NOT RESULT STREQUAL "0")
        message(FATAL_ERROR "Cannot get gdal version by running "
          "\"${GDAL_CONFIG} --version\".")
      endif()
    else()
      file(STRINGS "${GDAL_INCLUDE_DIR}/gdal_version.h"
        _gdal_VERSION_HPP_CONTENTS REGEX
        "# *define +GDAL_RELEASE_NAME +")
      string(REGEX REPLACE "^.*\"(.+)\".*$" "\\1"
        GDAL_VERSION "${_gdal_VERSION_HPP_CONTENTS}")
      if(NOT GDAL_VERSION)
        message(FATAL_ERROR "Cannot get gdal version from "
          "\"${GDAL_INCLUDE_DIR}/gdal_version.h\"")
      endif()
    endif()
    set(GDAL_VERSION "${GDAL_VERSION}" PARENT_SCOPE)
    message(STATUS "geo: Using GDAL_VERSION: ${GDAL_VERSION}.")
  endif()

  if(GDAL_VERSION VERSION_LESS 2.2.0)
    list(APPEND geo_GDAL_SOURCES
      detail/ovrdataset.hpp
      detail/ovrdataset.old.cpp)
  else()
    list(APPEND geo_GDAL_SOURCES
      detail/ovrdataset.hpp
      detail/ovrdataset.new.cpp)
  endif()

  # gdalwrapper.h contains __unused__ attribute in newer versions
  set_source_files_properties(geodataset.cpp PROPERTIES
    COMPILE_FLAGS -Wno-attributes)

  # OpenCV
  if(OpenCV_FOUND)
    list(APPEND geo_EXTRA_DEPENDS OpenCV)
    list(APPEND geo_EXTRA_DEFINITIONS GEO_HAS_OPENCV=1)
    message(STATUS "geo: compiling in OpenCV support")

    # OpenCV sources go here
    set(geo_OPENCV_SOURCES
      cv.hpp
      geodataset.hpp geodataset.cpp
      dem.hpp dem.cpp
      heightcoding.hpp heightcoding.cpp heightcoding-io.cpp
      featurelayers.hpp featurelayers.cpp
      gdsblockwriter.hpp gdsblockwriter.cpp
    )
  else()
    message(STATUS "geo: compiling without OpenCV support")
  endif()

else()
  message(STATUS "geo: compiling without GDAL & OpenCV support")
  list(APPEND geo_GDAL_SOURCES
      srsdefWithoutGdal.cpp
      csconvertorWithoutGdal.cpp)
endif()


define_module(LIBRARY geo=${geo_VERSION}
  DEPENDS
  BuildSystem>=1.0
  jsoncpp>=2.1 geometry>=1.7 imgproc>=1.26 math>=1.2
  utility>=1.11 dbglog>=1.4

  Boost_FILESYSTEM PROJ GeographicLib ${geo_EXTRA_DEPENDS}
  DEFINITIONS ${geo_EXTRA_DEFINITIONS}
  )

set(geo_SOURCES
    srsdef.hpp
    project.hpp project.cpp
    csconvertor.hpp
    coordinates.hpp
    srsfactorsfwd.hpp
    verticaladjuster.hpp verticaladjuster.cpp
    srsfactors.hpp srsfactors.cpp

    ${geo_GDAL_SOURCES}
    ${geo_OPENCV_SOURCES}
    )

add_library(geo STATIC ${geo_SOURCES})
buildsys_library(geo)

target_link_libraries(geo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo ${MODULE_DEFINITIONS})

# test stuff
if(GDAL_FOUND AND MODULE_service_FOUND)
  add_subdirectory(test-factors EXCLUDE_FROM_ALL)
  add_subdirectory(test-hconv EXCLUDE_FROM_ALL)
  add_subdirectory(test-srs EXCLUDE_FROM_ALL)
  add_subdirectory(test-geoid EXCLUDE_FROM_ALL)
  add_subdirectory(test-heightmodel EXCLUDE_FROM_ALL)
  add_subdirectory(tools EXCLUDE_FROM_ALL)
endif()
