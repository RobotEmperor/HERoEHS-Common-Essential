/*
 * action_module.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: robotemperor
 */
#include <stdio.h>
#include "action_module/action_module.h"
using namespace action_module;

ActionModuleState::ActionModuleState()
{
	is_moving_state= false;
	MAX_JOINT_ID_STATE = 0;

	joint_ini_pose_state  = Eigen::MatrixXd::Zero( MAX_JOINT_ID_STATE + 3, 1);
	joint_ini_pose_goal  = Eigen::MatrixXd::Zero( MAX_JOINT_ID_STATE + 3, 1);

}
ActionModuleState::~ActionModuleState()
{
}
ActionData::ActionData()
{
	motion_player_total_time = 0;
	motion_sequence = 0;
	motion_sequence_all = 0;
	motion_player_time = 0;
	parse_check = 0;

	pose_data.clear();
	pose_data_array.clear();
	pose_time.clear();
	pose_key        = "";
	pose_key_time   = "";

}
ActionData::~ActionData()
{
}
void ActionData::initialize()
{
	motion_player_total_time = 0;
	motion_sequence = 0;
	motion_sequence_all = 0;
	motion_player_time = 0;
	parse_check = 0;

	pose_data.clear();
	pose_data_array.clear();
	pose_time.clear();
	pose_key        = "";
	pose_key_time   = "";
}
ActionModule::ActionModule()
: control_cycle_msec_(8)
{
	running_ = false;
	enable_       = false;
	gazebo_check  = false;
	module_name_  = "action_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3

	result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
	result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

	result_["l_shoulder_yaw"]    = new robotis_framework::DynamixelState();  // joint 7
	result_["r_shoulder_yaw"]    = new robotis_framework::DynamixelState();  // joint 8

	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
//	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

/*
	result_["l_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 13
	result_["l_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 22
*/

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23
	result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 24
	result_["head_roll"]        = new robotis_framework::DynamixelState();  // joint 25


	// TEST
	/*
	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5

	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23
	 */
	//result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	//result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18


	new_count_ = 1;
	///////////////////////////

	for(int i=1;i<26;i++)
	{
		motion_trajectory[i]= new heroehs_math::FifthOrderTrajectory();
	}

	action_module_state  = new ActionModuleState();
	action_data_common = new ActionData();
	for(int i=0; i<5;i++)
	{
		action_data_part[i] = new ActionData();
	}

	is_moving = false;
	is_playing = false;
	start_time_temp= 0;
	joint_dh_change = 1;
}
ActionModule::~ActionModule()
{
	queue_thread_.join();
}
void ActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&ActionModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
		action_module_state->mov_time_state[dxl_info->id_] = 4.0;

		action_module_state->MAX_JOINT_ID_STATE ++; // joint 개수 확인
	}
	action_module_state -> joint_ini_pose_state.resize(26,1);
	action_module_state -> joint_ini_pose_goal.resize(26,1);
	action_module_state -> joint_ini_pose_state.fill(0);
	action_module_state -> joint_ini_pose_goal.fill(0);

	ROS_INFO("< -------  Initialize Module : Action Module !!  ------->");
}
void ActionModule::parse_init_pose_data_(const std::string &path)
{
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	double mov_time_ = 0.0;

	mov_time_= doc["mov_time"].as<double>(); // YAML 에 string "mov_time"을 읽어온다.

	for (std::map<int, std::string>::iterator it = joint_id_to_name_.begin();
			it != joint_id_to_name_.end(); it++)
	{
		action_module_state->mov_time_state[it->first] = mov_time_;
	}

	YAML::Node tar_pose_node = doc["tar_pose"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int id;
		double value;
		// 한 줄에서 int 와 double 을 분리한다.
		id = it->first.as<int>();
		value = it->second.as<double>();

		action_module_state->joint_ini_pose_goal.coeffRef(id, 0) = value * DEGREE2RADIAN; // YAML에 로드된 초기 포즈 값을 라디안으로 바꾸고, eigen matrix 에 id 개수만큼 열을 생성한다.
	}
}
void ActionModule::initPose(std::string command) // GUI 에서 init pose topic을 sub 받아 실
{
	joint_dh_change = 1;
	for(int id=9 ; id<26 ; id++)
	{
		motion_trajectory[id]->current_time = 0;
	}
	// 팔 다이나믹셀 초기화
	for(int id=1 ; id<9 ; id++)
	{
		motion_trajectory[id]->current_time = 0;
	}
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	if(command.compare("init_pose") == 0)
	{
		init_pose_path = ros::package::getPath("action_module") + "/data/init_pose.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
		ROS_INFO("FILE LOAD  ::  init_pose");
		parse_init_pose_data_(init_pose_path); // YAML 파일 로드
	}
	else
		return;
	new_count_ = 1;
	action_module_state->is_moving_state = true;
	is_moving = true;
	joint_command = "init_pose";
	action_command = "init_pose";
	ROS_INFO("FILE LOAD complete");
}
void ActionModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg) //
{
	initPose("init_pose");
	joint_command = "init_pose";
	action_command = "";
}
void ActionModule::MotionPlayer(ActionData *data, std::string motion_file_name, double start_time)
{
	data->motion_player_time = data->motion_player_time + 0.008;
	if(data->parse_check == 0)
	{
		std::string file_path = ros::package::getPath("action_module") + "/motion_data/"+ motion_file_name + ".yaml";
		YAML::Node doc;
		try
		{
			// load yaml
			doc = YAML::LoadFile(file_path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

		}catch(const std::exception& e) // 에러 점검
		{
			ROS_ERROR("Fail to load yaml file!");
			return;
		}

		for (YAML::iterator it = doc.begin(); it != doc.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
		{
			int pose_number = 0;
			data->pose_key = it -> first.as<std::string>();

			if (data->pose_key.find("pose_time") == -1)// pose_key
			{
				YAML::Node data_node = doc[data->pose_key];
				pose_number = atoi(data->pose_key.erase(0,5).c_str());
				for (YAML::iterator it_pose_key = data_node.begin(); it_pose_key != data_node.end(); ++it_pose_key) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
				{
					data->pose_data[it_pose_key->first.as<std::string>()] = it_pose_key->second.as<double>();
					data->pose_data_array[pose_number] = data->pose_data;
				}
			}
			else// pose_time
			{
				pose_number = atoi(data->pose_key.erase(0,10).c_str());
				data->pose_key = it -> first.as<std::string>();
				data->pose_time[pose_number] = doc[data->pose_key].as<double>();
				data->motion_sequence_all ++;
				data->motion_player_total_time = data->motion_player_total_time  + data->pose_time[pose_number];
			}
		}
		data->parse_check ++;
	}
	if(data->parse_check == 1)
	{
		if(data->motion_player_time < start_time)
		{
			return;
		}
		else
		{
			data->motion_player_time = 0;
			data->parse_check ++;
		}
	}

	if(data->motion_sequence == data->motion_sequence_all)
	{
		return;
	}

	// motion play
	for(int num = 0; num < data->motion_sequence_all; num++)
	{
		if(data->motion_sequence == num)
		{
			if(data->motion_player_time > data->pose_time[num])
			{
				data->motion_sequence ++;
				data->motion_player_time = 0;

				if(data->motion_sequence == data->motion_sequence_all)
				{
					return;
				}
				else
				{
					initPositionParse(data->pose_data_array, data->pose_time, data->motion_sequence);
				}
			}

			if(data->motion_player_time == 0.008)
			{
				GoalPositionParse(data->pose_data_array, data->pose_time, data->motion_sequence);
				action_module_state->is_moving_state = true;
				is_moving = true;
				printf("END \n");
			}
		}
	}

	printf("motion_sequence :: %d \n", data->motion_sequence);

}
void ActionModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);
	/* subscribe topics */
	// for gui
	ini_pose_msg_sub = ros_node.subscribe("/heroehs/init_pose", 5, &ActionModule::initPoseMsgCallback, this);
	action_command_sub = ros_node.subscribe("/heroehs/action_command", 5, &ActionModule::ActionCommandMsgCallback, this);
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);

	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
bool ActionModule::isRunning()
{
	return running_;
}
void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if(action_command.compare("init_pose"))
	{
		// 팔 다이나믹셀 초기화
		if(joint_dh_change == 1)
		{

			for(int id=23 ; id<26 ; id++)
			{
				motion_trajectory[id]->current_time = 0;
			}
			// 팔 다이나믹셀 초기화
			for(int id=1 ; id<9 ; id++)
			{
				motion_trajectory[id]->current_time = 0;
			}

			for(int id=1 ; id< 10 ; id++)
			{
				ROS_INFO("motion play  id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);

				if(id == 1 || id == 3 || id == 5 || id == 4 || id == 6 || id == 8 || id == 9) // 방향 반대인 다이나믹셀
				{
					action_module_state->joint_ini_pose_goal(id,0) = -action_module_state->joint_ini_pose_goal(id,0); // 방향이 반대인 다이나믹셀만 초기화
				}
			}
			action_module_state->joint_ini_pose_goal(23,0) = -action_module_state->joint_ini_pose_goal(23,0); // 방향이 반대인 다이나믹셀만 초기화
			action_module_state->joint_ini_pose_goal(24,0) = -action_module_state->joint_ini_pose_goal(24,0); // 방향이 반대인 다이나믹셀만 초기화
			action_module_state->joint_ini_pose_goal(25,0) = -action_module_state->joint_ini_pose_goal(25,0); // 방향이 반대인 다이나믹셀만 초기화

			joint_dh_change = 0;
		}

		if(!action_command.compare("motion_play"))
		{
			MotionPlayer(action_data_common, motion_file_name_temp, 0);
		}
		else
		{
			for(int motion_num = 0; motion_num < action_data_time.size(); motion_num ++)
			{
				MotionPlayer(action_data_part[motion_num], action_data[motion_num], action_data_time[motion_num]);
			}
		}
		joint_command = "motion";
	}

	if(action_command.compare("pause") == 0)
	{
		is_playing = false;
		action_command = "";
	}

	if (enable_ == false)
	{
		return;
	}

	//// read current position ////
	if(new_count_ == 1)
	{
		ROS_INFO("Action_Process Start!");

		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(gazebo_check == true)
				action_module_state->joint_ini_pose_state.coeffRef(joint_name_to_id_[joint_name], 0) = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			else
			{
				result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
				action_module_state->joint_ini_pose_state.coeffRef(joint_name_to_id_[joint_name], 0) = dxls[joint_name]->dxl_state_->present_position_; // 초기위치 저장
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Action module :: Read position and Send goal position");

		new_count_ = 0;
	}
	// trajectory is working joint space control
	if(action_module_state->is_moving_state == false)
	{
		ROS_INFO("Action Stay");
		is_moving = false;
	}
	else
	{
		ROS_INFO("Action Trajectory Start");
		if(joint_command.compare("init_pose"))
		{
			for(int id=23 ; id<26 ; id++)
			{
				result_[joint_id_to_name_[id]]->goal_position_ = motion_trajectory[id]->fifth_order_traj_gen(action_module_state->joint_ini_pose_state(id,0),
						action_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,action_module_state->mov_time_state[id]);
				if(gazebo_check == true)
					result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
				//ROS_INFO("motion play  id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
			}
			for(int id=1 ; id<10 ; id++)
			{
				result_[joint_id_to_name_[id]]->goal_position_ = motion_trajectory[id]->fifth_order_traj_gen(action_module_state->joint_ini_pose_state(id,0),
						action_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,action_module_state->mov_time_state[id]);
				if(gazebo_check == true)
					result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
			}

			if(     motion_trajectory[1]->is_moving_traj  || motion_trajectory[2]->is_moving_traj  || motion_trajectory[3]->is_moving_traj ||
					motion_trajectory[4]->is_moving_traj  || motion_trajectory[5]->is_moving_traj  || motion_trajectory[6]->is_moving_traj ||
					motion_trajectory[7]->is_moving_traj  || motion_trajectory[8]->is_moving_traj  || motion_trajectory[9]->is_moving_traj ||
					motion_trajectory[23]->is_moving_traj || motion_trajectory[24]->is_moving_traj || motion_trajectory[25]->is_moving_traj )
			{
				is_moving = true;
				action_module_state->is_moving_state = true;// action time 으로 제한
			}
			else
			{
				is_moving = false;
				action_module_state->is_moving_state = false;// action time 으로 제한
				is_playing = false;
			}
		}
		else
		{
			for(int id=23 ; id<26 ; id++)
			{
				if(id == 23 || id == 24 || id == 25) // 방향 반대인 다이나믹셀
				{
					result_[joint_id_to_name_[id]]->goal_position_ = - motion_trajectory[id]->fifth_order_traj_gen(-action_module_state->joint_ini_pose_state(id,0),
							action_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,action_module_state->mov_time_state[id]);
					if(gazebo_check == true)
						result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
					//ROS_INFO("id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
				}
			}
			// 팔 다이나믹셀 초기화
			for(int id=1 ; id<10 ; id++)
			{
				if(id == 1 || id == 3 || id == 5 || id == 4 || id == 6 || id == 8 || id == 9) // 방향 반대인 다이나믹셀
				{
					result_[joint_id_to_name_[id]]->goal_position_ = - motion_trajectory[id]->fifth_order_traj_gen(- action_module_state->joint_ini_pose_state(id,0),
							action_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,action_module_state->mov_time_state[id]);
					if(gazebo_check == true)
						result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
					//ROS_INFO("id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
				}
				else
				{
					result_[joint_id_to_name_[id]]->goal_position_ =  motion_trajectory[id]->fifth_order_traj_gen(action_module_state->joint_ini_pose_state(id,0),
							action_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,action_module_state->mov_time_state[id]);
					if(gazebo_check == true)
						result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
					//ROS_INFO("id :: %d , value %f", id ,  result_[joint_id_to_name_[id]]->goal_position_);
					//ROS_INFO("GOAL id :: %d , value %f", id ,  action_module_state->joint_ini_pose_goal(id,0));
				}
			}
			action_module_state->is_moving_state = motion_trajectory[1]->is_moving_traj;// action time 으로 제한
			is_moving = motion_trajectory[1]->is_moving_traj;
			is_playing = motion_trajectory[1]->is_moving_traj;
		}
	}
}
void ActionModule::stop()
{
	return;
}
void ActionModule::GoalPositionParse(std::map< int, std::map<std::string, double> > pose_key_data_array, std::map<int, double> pose_key_time, int num)
{
	for (std::map< int, std::map<std::string, double> >::iterator it_pose_key = pose_key_data_array.begin(); it_pose_key != pose_key_data_array.end(); ++it_pose_key) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		if(it_pose_key->first == num)
		{
			for (std::map<std::string, double>::iterator it_pose_joint = it_pose_key->second.begin(); it_pose_joint != it_pose_key->second.end(); ++ it_pose_joint)
			{
				motion_trajectory[joint_name_to_id_[it_pose_joint->first]]->current_time = 0;
				//printf("before   %d  :: value  ::  %f", joint_name_to_id_[it_pose_joint->first], action_module_state->joint_ini_pose_goal.coeffRef(joint_name_to_id_[it_pose_joint->first],0));
				action_module_state->joint_ini_pose_goal.coeffRef(joint_name_to_id_[it_pose_joint->first], 0) = pose_key_data_array[it_pose_key->first][it_pose_joint->first]*DEGREE2RADIAN;
				//printf("after    %d  :: value  ::  %f", joint_name_to_id_[it_pose_joint->first], action_module_state->joint_ini_pose_goal.coeffRef(joint_name_to_id_[it_pose_joint->first],0));
				action_module_state->mov_time_state[joint_name_to_id_[it_pose_joint->first]]=pose_key_time[it_pose_key->first];
			}
		}
	}
}
void ActionModule::initPositionParse(std::map< int, std::map<std::string, double> > pose_key_data_array, std::map<int, double> pose_key_time, int num)
{
	for (std::map< int, std::map<std::string, double> >::iterator it_pose_key = pose_key_data_array.begin(); it_pose_key != pose_key_data_array.end(); ++it_pose_key) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		if(it_pose_key->first == num)
		{
			for (std::map<std::string, double>::iterator it_pose_joint = it_pose_key->second.begin(); it_pose_joint != it_pose_key->second.end(); ++ it_pose_joint)
			{
				motion_trajectory[joint_name_to_id_[it_pose_joint->first]]->current_time = 0;
				action_module_state->joint_ini_pose_state.coeffRef(joint_name_to_id_[it_pose_joint->first], 0) = action_module_state->joint_ini_pose_goal.coeffRef(joint_name_to_id_[it_pose_joint->first], 0);
			}
		}
	}

}

void ActionModule::ActionDataParse(std::string action_file_name)
{
	//ROS_INFO("data  :: %s \n",action_file_name.c_str());

	std::string file_path = ros::package::getPath("action_module") + "/motion_data/action_data/"+ action_file_name + ".yaml";
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(file_path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	YAML::Node action_data_node = doc["action"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = action_data_node.begin(); it != action_data_node.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int temp_int;
		std::string temp;
		temp_int = it->first.as<int>();
		temp =  it->second.as<std::string>();
		action_data[temp_int] = temp;

		//printf("num  %d   ::   data  :: %s \n",temp_int , action_data[temp_int].c_str());
	}

	action_data_node = doc["time"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = action_data_node.begin(); it != action_data_node.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int temp_time_num;
		double temp_time_value;
		temp_time_num = it->first.as<int>();
		temp_time_value =  it->second.as<double>();
		action_data_time[temp_time_num] = temp_time_value;

		//printf("time_num  %d   ::   time_data  :: %f \n",temp_time_num  , action_data_time[temp_time_num]);
	}

}
void ActionModule::ActionDataInitialize()
{
        action_data.clear();
	action_data_time.clear();
	if(!action_command.compare("action_play"))
		ActionDataParse(action_file_name_temp);

	new_count_ = 1;
	//joint_dh_change = 1;
	action_data_common ->initialize();

	for(int i=0; i<4;i++)
	{
		action_data_part[i] -> initialize();
	}
}
//gui action && motion server
void ActionModule::ActionCommandMsgCallback(const std_msgs::String::ConstPtr& msg)
{
	/*action_command  = msg->data;
	if(msg->data == "init_pose")
	{
		joint_dh_change = 1;
		is_moving = true;
		return;
	}
	else
	{
		ActionDataInitialize();
		is_moving = true;
	}*/
}


// motion player



