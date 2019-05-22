define_module(LIBRARY pygeo=${MODULE_geo_VERSION}
  DEPENDS
  geo=${MODULE_geo_VERSION}
  Boost_THREAD Boost_SYSTEM Boost_PYTHON PYTHONLIBS)

set(pygeo_SOURCES
  geomodule.hpp
  geomodule.cpp
  gdsblockwriter.hpp
  )

if(NumPy_FOUND)
  message(STATUS "geo::python: compiling in numpy support")

  list(APPEND pygeo_DEPENDS NumPy)
  list(APPEND pygeo_SOURCES gdsblockwriter.cpp)
else()
  message(STATUS "geo::python: compiling without numpy support")
  list(APPEND pygeo_SOURCES gdsblockwriter.dummy.cpp)
endif()

add_library(pygeo STATIC ${pygeo_SOURCES})
buildsys_library(pygeo)
target_link_libraries(pygeo ${MODULE_LIBRARIES})
