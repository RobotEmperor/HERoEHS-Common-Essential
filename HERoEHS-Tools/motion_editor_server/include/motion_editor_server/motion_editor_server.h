/*
 * motion_editor_server.h
 *
 *  Created on: Mar 11, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ROBOCUP_HEROEHS_TOOLS_MOTION_EDITOR_SERVER_INCLUDE_MOTION_EDITOR_SERVER_MOTION_EDITOR_SERVER_H_
#define HEROEHS_ROBOCUP_HEROEHS_TOOLS_MOTION_EDITOR_SERVER_INCLUDE_MOTION_EDITOR_SERVER_MOTION_EDITOR_SERVER_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <stdlib.h>
#include <errno.h>
#include <ros/package.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>

#include "robotis_controller/robotis_controller.h"
#include "offset_tuner_msgs/JointOffsetState.h"
#include "offset_tuner_msgs/JointTorqueOnOff.h"
#include "offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "offset_tuner_msgs/PresentJointStateArray.h"
#include "motion_editor_msgs/EditorCommand.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "action_module/action_module.h"

typedef struct _finddata_t  FILE_SEARCH;

namespace motion_editor
{

class JointOffsetData
{
public:
	double joint_offset_rad_;
	double joint_init_offset_rad_;
	double joint_init_pos_rad_;
	int p_gain_;
	int i_gain_;
	int d_gain_;
	JointOffsetData()
	{
		joint_offset_rad_ = 0;
		joint_init_offset_rad_ = 0;
		joint_init_pos_rad_ = 0;
		p_gain_ = 32;
		i_gain_ = 0;
		d_gain_ = 0;

	}
	~JointOffsetData()
	{

	}
private:

};

class MotionEditorServer: public robotis_framework::Singleton<MotionEditorServer>
{
public:
	MotionEditorServer();
	~MotionEditorServer();

	bool initialize();
	//void moveToInitPose();


private:
	robotis_framework::RobotisController* controller;

	//ros communication
	ros::Subscriber command_state_sub;
	ros::Subscriber joint_goal_state_sub;
	ros::Publisher moving_state_pub;

	ros::ServiceServer joint_torque_on_off_ser;
	ros::ServiceServer present_joint_state_array_ser;
	ros::ServiceServer editor_command_ser;

	//message
	std_msgs::Bool moving_state_msg;


	std::string init_file;
	std::string robot_file;
	std::string offset_file;
	std::string motion_data_path;   //로스 패키지에서 YAML파일의 경로를 읽어온다
	std::string motion_file_name;   //로스 패키지에서 YAML파일의 경로를 읽어온다
	std::string action_file_name;   //로스 패키지에서 YAML파일의 경로를 읽어온다

	std::map<std::string, bool>              robot_torque_enable_data;
	std::map<std::string, double>            pose_data;
	vector< std::map<std::string, double> >  pose_data_array;
	vector<std::string>                      pose_key;
	vector<std::string>                      pose_key_time;
	vector<double>                           pose_time;
	bool                                     pose_save_check;

	//variables
	int pose_num;
	std::string operating_command;
	pid_t g_play_pid;

	void setCtrlModule(std::string module);
	void moveToPose();
	void CommandStateMsgsCallBack(const std_msgs::String::ConstPtr& msg);
	void JointGoalStateMsgsCallBack(const offset_tuner_msgs::JointOffsetState::ConstPtr& msg);

	bool JointTorqueOnOffCallBack(offset_tuner_msgs::JointTorqueOnOff::Request &req,offset_tuner_msgs::JointTorqueOnOff::Response &res);
    bool PresentJointStateArrayCallBack(offset_tuner_msgs::PresentJointStateArray::Request &req,offset_tuner_msgs::PresentJointStateArray::Response &res);
    bool EditorCommandCallBack(motion_editor_msgs::EditorCommand::Request &req, motion_editor_msgs::EditorCommand::Response &res);
    void playSound(std::string file_name);
};

}



#endif /* HEROEHS_ROBOCUP_HEROEHS_TOOLS_MOTION_EDITOR_SERVER_INCLUDE_MOTION_EDITOR_SERVER_MOTION_EDITOR_SERVER_H_ */
