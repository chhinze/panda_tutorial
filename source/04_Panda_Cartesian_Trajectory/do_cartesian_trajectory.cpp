#include "Trajectory.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <iostream>

#include <iostream>

std::string robot_ip = "172.16.0.2";

void setDefaultBehaviour(franka::Robot &robot);

int main() {

  try {
    // Connect to robot
    franka::Robot panda(robot_ip);
    setDefaultBehaviour(panda);

    // read current robot state
    franka::RobotState initial_state = panda.readOnce();
    Eigen::Vector6d initial_pose = homogeneousTfArray2PoseVec(initial_state.O_T_EE_c);

    Eigen::Vector6d targetPose = initial_pose;
    targetPose.head<3>() += Eigen::Vector3d::Constant(0.1);

    auto traj = LinearTrajectory(initial_pose, targetPose, 0.05, 0.5, 1.e-3);
    std::cout << "t_E = " << traj.getTEnd() << " s" << std::endl;
    auto motionIterator = std::make_unique<TrajectoryIteratorCartesianPose>(traj);

    std::cout << "WARNING: The robot will move now. "
              << "Keep around the STOP button." << std::endl
              << "Press ENTER to continue." << std::endl;

    panda.control(
        [&motionIterator](const franka::RobotState &,
                          franka::Duration) -> franka::CartesianVelocities {
          auto cartesianVelDes =
              franka::CartesianVelocities(motionIterator->getCartesianVelocity());
          motionIterator->step();

          if (motionIterator->getCurrentTime() < motionIterator->getEndTime()) {
            return cartesianVelDes;
          } else {
            return franka::MotionFinished(cartesianVelDes);
          }
        },
        /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);

  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
    return -2;
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    return -10;
  }

  std::cout << "Motion finished regularly." << std::endl;
  return 0;
}

void setDefaultBehaviour(franka::Robot &robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}