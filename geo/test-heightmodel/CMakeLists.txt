define_module(BINARY geo-heightmodel
  DEPENDS geo service
)
set(geo-heightmodel_SOURCES main.cpp)
add_executable(geo-heightmodel EXCLUDE_FROM_ALL ${geo-heightmodel_SOURCES})
target_link_libraries(geo-heightmodel ${MODULE_LIBRARIES})
target_compile_definitions(geo-heightmodel PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(geo-heightmodel)
