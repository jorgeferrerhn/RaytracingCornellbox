function(add_yapp name)
  add_executable(${name} ${name}.cpp)
  set_target_properties(${name} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
  target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/libs)
  target_link_libraries(${name} PRIVATE yocto yocto_examples)
endfunction()

add_yapp(ygrade)
add_yapp(yraytrace)
