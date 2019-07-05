project(do_eigen_math)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME} do_eigen3_math.cpp)
