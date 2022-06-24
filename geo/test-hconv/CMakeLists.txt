define_module(BINARY hconv
  DEPENDS geo service
)

set(hconv_SOURCES hconv.cpp)
add_executable(hconv EXCLUDE_FROM_ALL ${hconv_SOURCES})
target_link_libraries(hconv ${MODULE_LIBRARIES})
target_compile_definitions(hconv PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(hconv)

set(csconv_SOURCES csconv.cpp)
add_executable(csconv EXCLUDE_FROM_ALL ${csconv_SOURCES})
target_link_libraries(csconv ${MODULE_LIBRARIES})
target_compile_definitions(csconv PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(csconv)
