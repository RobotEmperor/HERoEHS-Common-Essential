/*
 * heroehs_main_manager.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: robotemperor
 */
/*
 * aki_main_manager.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: robotemperor
 */

#include "robotis_controller/robotis_controller.h"
#include "heroehs_base_module/heroehs_base_module_node.h"
#include "action_module/action_module.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Heroehs_main_manager");
  ros::NodeHandle nh;
  ROS_INFO("heroehs_main_manager->init !! start!");
  robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

  controller->DEBUG_PRINT = true;
  /* Load ROS Parameter */
  std::string offset_file = nh.param<std::string>("offset_table", "");
  std::string robot_file  = nh.param<std::string>("robot_file_path", "");
  std::string init_file   = nh.param<std::string>("init_file_path", "");

  /* gazebo simulation */
  controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
  if (controller->gazebo_mode_ == true)
  {
    std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;

    ROS_WARN("GAZEBO_MODE!!!!!!!!!!!!!!!");
  }
  base_module::BaseModule::getInstance()->gazebo_check = controller->gazebo_mode_;
  action_module::ActionModule::getInstance()->gazebo_check = controller->gazebo_mode_;

  if (robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  if (controller->initialize(robot_file, init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  if (offset_file != "")
    controller->loadOffset(offset_file);

  sleep(1);

  /* Add Motion Module */
  controller->addMotionModule((robotis_framework::MotionModule*) base_module::BaseModule::getInstance());
  controller->addMotionModule((robotis_framework::MotionModule*) action_module::ActionModule::getInstance());


  controller->startTimer();

  while (ros::ok())
  {
    usleep(1000*1000);
  }

  return 0;
}



