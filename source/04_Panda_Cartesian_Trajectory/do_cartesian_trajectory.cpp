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

    // 2. read current robot state

    // 3. Calculate target pose

    // 4./5. LinearTrajectory and TrajectoryIteratorCartesianVelocity object creation

    std::cout << "WARNING: The robot will move now. "
              << "Keep around the STOP button." << std::endl
              << "Press ENTER to continue." << std::endl;
    std::cin.ignore();

    // 6. Franka Robot Controller:

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