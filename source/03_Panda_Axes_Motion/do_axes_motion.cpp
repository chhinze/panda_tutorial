#include "examples_common.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <iostream>

std::string pandaIP = "172.16.0.2";
int main(void) {
  try {
    franka::Robot panda(pandaIP);
    setDefaultBehavior(panda);

    // 2. Define goal position and construct MotionGenerator (defined in "examples_common.h")
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.05, q_goal);

    std::cout << "WARNING: The robot will move now. "
              << "Keep around the STOP button." << std::endl
              << "Press ENTER to continue." << std::endl;
    std::cin.ignore();

    // 3. start the control
    panda.control(motion_generator);

    std::cout << "Finished moving to joint configuration : [";
    for (auto e : q_goal) {
      std::cout << e << ",";
    }
    std::cout << "] ." << std::endl;

  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}