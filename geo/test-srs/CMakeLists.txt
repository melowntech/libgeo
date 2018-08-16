define_module(BINARY geo-srsconv
  DEPENDS geo service
)

set(geo-srsconv_SOURCES geo-srsconv.cpp)
add_executable(geo-srsconv ${geo-srsconv_SOURCES})
target_link_libraries(geo-srsconv ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-srsconv PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-srsconv)

set(geo-cs2cs_SOURCES geo-cs2cs.cpp)
add_executable(geo-cs2cs ${geo-cs2cs_SOURCES})
target_link_libraries(geo-cs2cs ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-cs2cs PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-cs2cs)

set(geo-srsinfo_SOURCES geo-srsinfo.cpp)
add_executable(geo-srsinfo ${geo-srsinfo_SOURCES})
target_link_libraries(geo-srsinfo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-srsinfo PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-srsinfo)
