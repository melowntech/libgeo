define_module(BINARY hconv
  DEPENDS geo service
)

set(hconv_SOURCES hconv.cpp)
add_executable(hconv ${hconv_SOURCES})
target_link_libraries(hconv ${MODULE_LIBRARIES})
buildsys_binary(hconv)
