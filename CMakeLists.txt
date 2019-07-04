CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

project(test_proj_franka_ctrl 
        VERSION 0.1.0 
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_options(/W0)
else()
  add_compile_options(-Wall -Wextra)
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-pipe) #  Avoid temporary files, speeding up builds
  endif()
endif()

option(STRICT "Treat warnings as errors" ON)
if(STRICT)
  if(MSVC)
    add_compile_options(/WX)
  else()
    add_compile_options(-Werror)
  endif()
endif()

add_subdirectory(resources/from_libfranka)
add_subdirectory(source)
add_subdirectory(documentation)
