define_module(LIBRARY pygeo=${MODULE_geo_VERSION}
  DEPENDS
  geo=${MODULE_geo_VERSION}
  Boost_THREAD Boost_SYSTEM Boost_PYTHON PYTHONLIBS)

set(pygeo_SOURCES
  geomodule.cpp
  )

add_library(pygeo STATIC ${pygeo_SOURCES})
buildsys_library(pygeo)
target_link_libraries(pygeo ${MODULE_LIBRARIES})
