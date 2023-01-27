add_library(yocto_examples
  yocto_grade.h yocto_grade.cpp
  yocto_raytrace.h yocto_raytrace.cpp
)

set_target_properties(yocto_examples PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yocto_examples PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yocto_examples yocto)
