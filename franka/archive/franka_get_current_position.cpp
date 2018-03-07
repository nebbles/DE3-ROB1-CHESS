// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include <unistd.h>



/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_cartesian_velocity_motion <robot-hostname>" << std::endl;
    return -1;
  }
  try {

    std::cout.precision(3); 
    
    franka::Robot robot(argv[1]);

    size_t count = 0;
    robot.read([&count](const franka::RobotState& robot_state) {

      auto state_pose = robot_state.O_T_EE_d;
      std::array<double, 16> current_pose = state_pose;

      //std::cout << "current position x y z: " << current_pose[12] << "  " << current_pose[13] << "  " << current_pose[14] << std::endl; 
      std::cout << "[" << current_pose[12] << "," << current_pose[13] << "," << current_pose[14] << "]" << std::endl; 
      //sleep(1); 

      return 0; 
    });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
