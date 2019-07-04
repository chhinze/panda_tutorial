project(axes_motion)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6 REQUIRED)

add_executable(${PROJECT_NAME} do_axes_motion.cpp)
target_link_libraries(${PROJECT_NAME} Franka::Franka franka::lib_examples)