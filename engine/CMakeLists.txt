cmake_minimum_required(VERSION 3.25)
project(engine)
add_library(engine STATIC 
"helpers.h" 
"Debug.h" 
"Debug.cpp" 
 "timer.h"  "serialization.h")
target_include_directories(engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(engine PRIVATE cyclone raylib)
install (TARGETS engine DESTINATION lib)