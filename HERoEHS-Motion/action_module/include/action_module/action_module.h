/*
 * action_module.h
 *
 *  Created on: Mar 11, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ROBOCUP_HEROEHS_MOTION_ACTION_MODULE_INCLUDE_ACTION_MODULE_ACTION_MODULE_H_
#define HEROEHS_ROBOCUP_HEROEHS_MOTION_ACTION_MODULE_INCLUDE_ACTION_MODULE_ACTION_MODULE_H_

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

namespace action_module
{
class ActionModuleState
{
public:
	ActionModuleState();
	~ActionModuleState();

	bool is_moving_state;
	std::map<int, double> mov_time_state;
	int MAX_JOINT_ID_STATE;

	Eigen::MatrixXd joint_ini_pose_state;
	Eigen::MatrixXd joint_ini_pose_goal;

};
class ActionData
{
public:
	ActionData();
	~ActionData();

	void initialize();

	//data parse variables
	std::map<std::string, double>            pose_data;
	std::map< int, std::map<std::string, double> >  pose_data_array;
	std::map<int, double>                    pose_time;
	std::string                              pose_key;
	std::string                              pose_key_time;


	//motion player variables
	double motion_player_total_time;
	double motion_player_time;
	int motion_sequence;
	int motion_sequence_all;
	int parse_check;

private:

};


class ActionModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
public:
	ActionModule();
	virtual ~ActionModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void initPose(std::string command);
	void ActionDataParse(std::string action_file_name);
	void ActionDataInitialize();
	void MotionPlayer(ActionData *data, std::string motion_file_name, double start_time);

	void GoalPositionParse(std::map< int, std::map<std::string, double> > pose_key_data_array, std::map<int, double> pose_key_time, int num);
	void initPositionParse(std::map< int, std::map<std::string, double> > pose_key_data_array, std::map<int, double> pose_key_time, int num);

    void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg); //

	void stop();
	bool isRunning();

	bool gazebo_check;

	/* ROS Topic Callback Functions */
	ros::Subscriber action_command_sub;
	ros::Subscriber ini_pose_msg_sub;

	void ActionCommandMsgCallback(const std_msgs::String::ConstPtr& msg);




	ActionModuleState *action_module_state;
	heroehs_math::FifthOrderTrajectory *motion_trajectory[26];
	bool is_moving;
	bool is_playing;

	std::string motion_file_name_temp;
	std::string action_file_name_temp;
	double start_time_temp;

	//action data
	ActionData *action_data_common;
	ActionData *action_data_part[4];



	//int motion_sequence; /////////////////////
	//int motion_sequence_all;
	//double motion_player_time;
	//int parse_check;
	std::string action_command;

private:
	void queueThread();
	void parse_init_pose_data_(const std::string &path);
	bool running_;

	int new_count_;
	int joint_dh_change;
	int control_cycle_msec_;

	boost::thread queue_thread_;
	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;
	std::map<int, std::string> action_data;
	std::map<int, double> action_data_time;

	std::string joint_command;


};
}




#endif /* HEROEHS_ROBOCUP_HEROEHS_MOTION_ACTION_MODULE_INCLUDE_ACTION_MODULE_ACTION_MODULE_H_ */
