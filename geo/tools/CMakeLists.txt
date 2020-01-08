define_module(BINARY geo-info
  DEPENDS geo service
)

set(geo-info_SOURCES geoinfo.cpp)
add_executable(geo-info EXCLUDE_FROM_ALL ${geo-info_SOURCES})
target_link_libraries(geo-info ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-info PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-info)
