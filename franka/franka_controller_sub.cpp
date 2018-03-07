/*
Authors: 
Fabian Falck
Petar Kormushev
*/

#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include <cmath>  // for isnan()
#include <limits> // for quiet_NaN

// for subscribers
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <thread>

using namespace std; 


// declaration of global variables
double target_x = std::numeric_limits<double>::quiet_NaN(); 
double target_y = std::numeric_limits<double>::quiet_NaN(); 
double target_z = std::numeric_limits<double>::quiet_NaN(); 
double speed = 0.0; 
double target_gripper_width = 1.0;  
double target_gripper_speed = 0.1; 
double target_gripper_force = 60; 

franka::Gripper* gripper = nullptr;  // just for the sake of creating a global object, will be changed later 

/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


// parameters to set
double speed_limit = 0.10;  // was tested to work properly


bool speed_below_limit(double target_speed) {
  if (target_speed <= speed_limit) {
    return true;
  }
  else {
    return false; 
  }
}


void motionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ROS_INFO("Motion callback received: [%lf, %lf, %lf, %lf]", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);  // 


  target_x = msg->data[0]; 
  target_y = msg->data[1]; 
  target_z = msg->data[2]; 
  speed = msg->data[3]; 

  // new target coordinates and speed now from within the control loop
  // TODO modify x,y,z here

  // only if the wanted speed is below the speed limit is the speed adjusted  
  if (speed_below_limit(speed)) {
    // speed = sub_speed; 
  }
}

void graspCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ROS_INFO("Gripper callback received: width, speed, force: [%lf, %lf, %lf]", msg->data[0], msg->data[1], msg->data[2]);  // 
  target_gripper_width = msg->data[0]; 
  target_gripper_speed = msg->data[1];
  target_gripper_force = msg->data[2];

  // bool moved = gripper->move(target_gripper_width, target_gripper_speed);  //  , target_gripper_force
  bool grasped = gripper->grasp(target_gripper_width, target_gripper_speed, target_gripper_force);  //  
  cout << "grasped " << grasped << endl;
  // bool moved = gripper->move(target_gripper_width+0.02, target_gripper_speed);  //  , target_gripper_force
  // cout << "moved " << moved << endl;

  // if(gripper->grasp(target_gripper_width, target_gripper_speed, target_gripper_force)) {
  //   bool stopped = gripper->stop();
  //   cout << stopped << endl;  
  // }
  // gripper->stop();
  // if () {
  //   gripper->stop(); 
  // }  // 
  // sleep(1); 
  // gripper->stop();  // releasing the object

  // ros::Rate loop_rate(2000);  // 2 kHz
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   loop_rate.sleep(); 
  // }

  // new target coordinates and speed now from within the control loop
  // TODO modify x,y,z here

  // only if the wanted speed is below the speed limit is the speed adjusted  
}

void moveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ROS_INFO("Gripper callback received: width, speed, force: [%lf, %lf, %lf]", msg->data[0], msg->data[1], msg->data[2]);  // 
  target_gripper_width = msg->data[0]; 
  target_gripper_speed = msg->data[1];
  // nove force here

  // bool moved = gripper->move(target_gripper_width, target_gripper_speed);  //  , target_gripper_force
  bool moved = gripper->move(target_gripper_width, target_gripper_speed);  //  
  cout << "moved " << moved << endl;
  // bool moved = gripper->move(target_gripper_width+0.02, target_gripper_speed);  //  , target_gripper_force
  // cout << "moved " << moved << endl;
}


// subscriber for absolute coordinates and speed
void subscribe_target_motion() {
  // subscription stuff
  ros::NodeHandle node_handle;
  ros::Subscriber sub_motion = node_handle.subscribe("franka_move_to", 1, motionCallback);  // 1: buffer size of message queue
  ros::Subscriber sub_grasp = node_handle.subscribe("franka_gripper_grasp", 1, graspCallback);
  ros::Subscriber sub_move = node_handle.subscribe("franka_gripper_move", 1, moveCallback);

  ros::Rate loop_rate(2000);  // 2 kHz
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep(); 
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");

  if (argc != 2) {
    std::cerr << "Usage: ./franka_gripper_control <robot-hostname>" << std::endl;
    return -1;
  }
  try {

    franka::Robot robot(argv[1]);
    gripper = new franka::Gripper(argv[1]); 

    cout << "gripper created" << endl; 

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

    auto initial_pose = robot.readOnce().O_T_EE_d;
    std::array<double, 16> current_pose = initial_pose;

    // call the subscription thread
    
    std::thread t1(subscribe_target_motion);

    cout << "before control loop" << endl; 

    robot.control([=, &time](const franka::RobotState& robot_state,
                             franka::Duration time_step) -> franka::CartesianVelocities {

      // cout << "within control loop" << endl; 

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

      // cout << cur_x << " " << cur_y << " " << cur << endl; 

      // initially, the robot moves to its current position (-> no motion)
      if (isnan(target_x)) {
        target_x = cur_x; 
        target_y = cur_y; 
        target_z = cur_z; 
      }

      // target coordinates

      // subscriber in separate thread
      
      // cout << target_x; 
      

      // computing the motion

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




      // stopping the motino properly
      // if (vel_norm < 0.001) {
      //   // stop program when target reached
      //   std::cout << std::endl << "Finished motion, shutting down..." << std::endl << std::flush;
      //   franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      //   return franka::MotionFinished(output);
      // }




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


/*
TODO: 
topics: 
- speed ->
- coordinates -> 
- motion finished <- 
*/

// 
