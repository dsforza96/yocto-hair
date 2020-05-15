add_library(yocto_extension yocto_extension.h yocto_extension.cpp)

set_target_properties(yocto_extension PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yocto_extension PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yocto_extension yocto yocto_pathtrace)
