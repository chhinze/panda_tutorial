project(libfranka_examples_common)

find_package(Franka 0.6 REQUIRED)
add_library(${PROJECT_NAME} examples_common.cpp)
add_library(franka::lib_examples ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} Franka::Franka)