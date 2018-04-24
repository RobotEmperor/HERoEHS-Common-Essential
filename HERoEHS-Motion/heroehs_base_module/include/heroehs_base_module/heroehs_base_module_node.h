/*
 * heroehs_base_module_node.h
 *
 *  Created on: Mar 7, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ROBOCUP_HEROEHS_MOTION_HEROEHS_BASE_MODULE_INCLUDE_HEROEHS_BASE_MODULE_HEROEHS_BASE_MODULE_NODE_H_
#define HEROEHS_ROBOCUP_HEROEHS_MOTION_HEROEHS_BASE_MODULE_INCLUDE_HEROEHS_BASE_MODULE_HEROEHS_BASE_MODULE_NODE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_math/robotis_math.h"

#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"

namespace base_module
{
class BaseModuleState
{
public:
  BaseModuleState();
  ~BaseModuleState();

  bool is_moving_state;
  double mov_time_state;
  int MAX_JOINT_ID_STATE;

  Eigen::MatrixXd joint_ini_pose_state;
  Eigen::MatrixXd joint_ini_pose_goal;

};

class BaseModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<BaseModule>
{
public:
	BaseModule();
	virtual ~BaseModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;

	/* ROS Topic Callback Functions */

	void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
	void go_to_init_pose(std::string data);


	BaseModuleState *base_module_state;
	heroehs_math::FifthOrderTrajectory *motion_trajectory[26];
	bool is_moving;

private:
	void queueThread();
	void parse_init_pose_data_(const std::string &path);
	void parse_init_offset_pose_data_(const std::string &path, const std::string &data);
	bool running_;

	int new_count_;
	int control_cycle_msec_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

};



}



#endif /* HEROEHS_ROBOCUP_HEROEHS_MOTION_HEROEHS_BASE_MODULE_INCLUDE_HEROEHS_BASE_MODULE_HEROEHS_BASE_MODULE_NODE_H_ */
