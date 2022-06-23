define_module(BINARY geo-geoid
  DEPENDS geo service
)
set(geo-geoid_SOURCES main.cpp)
add_executable(geo-geoid EXCLUDE_FROM_ALL ${geo-geoid_SOURCES})
target_link_libraries(geo-geoid ${MODULE_LIBRARIES})
target_compile_definitions(geo-geoid PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-geoid)
