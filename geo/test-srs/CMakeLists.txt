define_module(BINARY geo-srsconv
  DEPENDS geo service
)

set(geo-srsconv_SOURCES geo-srsconv.cpp)
add_executable(geo-srsconv ${geo-srsconv_SOURCES})
target_link_libraries(geo-srsconv ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo-srsconv PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-srsconv)
