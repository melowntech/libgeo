define_module(BINARY geo-factors
  DEPENDS geo service
)

set(geo-factors_SOURCES main.cpp)
add_executable(geo-factors EXCLUDE_FROM_ALL ${geo-factors_SOURCES})
target_link_libraries(geo-factors ${MODULE_LIBRARIES})
target_compile_definitions(geo-factors PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-factors)
