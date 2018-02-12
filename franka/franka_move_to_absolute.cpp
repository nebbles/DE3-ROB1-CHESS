// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>


/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: ./generate_cartesian_velocity_motion <robot-hostname> delta_x delta_y delta_z" << std::endl;
    return -1;
  }
  try {

    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    // robot.setJointImpedance({{300, 300, 300, 250, 250, 200, 200}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    double time = 0.0;

    double speed = 0.10; 

    auto initial_pose = robot.readOnce().O_T_EE_d;
    std::array<double, 16> current_pose = initial_pose;

    double target_x = atof(argv[2]); 
    double target_y = atof(argv[3]); 
    double target_z = atof(argv[4]); 


    robot.control([=, &time](const franka::RobotState& robot_state,
                             franka::Duration time_step) -> franka::CartesianVelocities {

      double vel_x = 0.0; 
      double vel_y = 0.0; 
      double vel_z = 0.0; 

      static double old_vel_x = 0.0; 
      static double old_vel_y = 0.0; 
      static double old_vel_z = 0.0; 

      time += time_step.toSec(); 

      auto state_pose = robot_state.O_T_EE_d;
      std::array<double, 16> current_pose = state_pose;


      double cur_x = current_pose[12]; 
      double cur_y = current_pose[13];
      double cur_z = current_pose[14]; 

      double vec_x = target_x - cur_x;
      double vec_y = target_y - cur_y;
      double vec_z = target_z - cur_z; 

      double l2_norm = sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z); 

      if (l2_norm < 0.02) {
          vel_x = 0.9*old_vel_x;
          vel_y = 0.9*old_vel_y; 
          vel_z = 0.9*old_vel_z; 
      }
      else {
        vel_x = speed*(vec_x / l2_norm);
        vel_y = speed*(vec_y / l2_norm);
        vel_z = speed*(vec_z / l2_norm); 
      }

      vel_x = 0.99*old_vel_x + 0.01*vel_x;
      vel_y = 0.99*old_vel_y + 0.01*vel_y;
      vel_z = 0.99*old_vel_z + 0.01*vel_z;
      
      old_vel_x = vel_x;
      old_vel_y = vel_y;
      old_vel_z = vel_z;

      franka::CartesianVelocities output = {{vel_x, vel_y, vel_z, 0.0, 0.0, 0.0}};

      double vel_norm = sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z); 
      if (vel_norm < 0.001) {
        // stop program when target reached
        std::cout << std::endl << "Finished motion, shutting down..." << std::endl << std::flush;
        franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        return franka::MotionFinished(output);
      }
      // franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
/*      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }*/
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
