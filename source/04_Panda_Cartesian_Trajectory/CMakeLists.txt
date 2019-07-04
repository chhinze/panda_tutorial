project(cartesian_trajectory)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6 REQUIRED)

add_library(trajectory_planner Trajectory.cpp MotionProfile.cpp Path.cpp)

add_executable(${PROJECT_NAME} do_cartesian_trajectory.cpp)
target_link_libraries(${PROJECT_NAME} trajectory_planner Franka::Franka)