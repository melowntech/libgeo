define_module(BINARY geo-info
  DEPENDS geo service
)

set(geo-info_SOURCES geoinfo.cpp)
add_executable(geo-info EXCLUDE_FROM_ALL ${geo-info_SOURCES})
target_link_libraries(geo-info ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-info PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-info)

set(geo-asc2gtx_SOURCES asc2gtx.cpp)
add_executable(geo-asc2gtx EXCLUDE_FROM_ALL ${geo-asc2gtx_SOURCES})
target_link_libraries(geo-asc2gtx ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-asc2gtx PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-asc2gtx)

set(geo-srs-asc2gtx_SOURCES srs-asc2gtx.cpp)
add_executable(geo-srs-asc2gtx EXCLUDE_FROM_ALL ${geo-srs-asc2gtx_SOURCES})
target_link_libraries(geo-srs-asc2gtx ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-srs-asc2gtx PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-srs-asc2gtx)
