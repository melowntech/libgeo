define_module(BINARY hconv
  DEPENDS geo service
)

set(hconv_SOURCES hconv.cpp)
add_executable(hconv ${hconv_SOURCES})
target_link_libraries(hconv ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(hconv PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(hconv)

set(csconv_SOURCES csconv.cpp)
add_executable(csconv ${csconv_SOURCES})
target_link_libraries(csconv ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(csconv PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(csconv)
