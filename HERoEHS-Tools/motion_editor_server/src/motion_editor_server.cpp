/*
 * motion_editor_server.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: robotemperor
 */
#include "motion_editor_server/motion_editor_server.h"


#define OFFSET_ROSPARAM_KEY "offset"
#define OFFSET_INIT_POS_ROSPARAM_KEY "init_pose_for_offset_tuner"

using namespace motion_editor;
using namespace action_module;

MotionEditorServer::MotionEditorServer()
: controller(0),
  init_file(""),
  robot_file(""),
  offset_file("")
{
	pose_num = 0;
	g_play_pid = -1;
	pose_save_check = false;
}

MotionEditorServer::~MotionEditorServer()
{
}

void MotionEditorServer::setCtrlModule(std::string module)
{
	robotis_controller_msgs::JointCtrlModule control_msg;

	std::map<std::string, robotis_framework::DynamixelState *>::iterator joint_iter;
	ros::NodeHandle nh;
	ros::Publisher set_ctrl_module_pub = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);

	ActionModule* action_module = ActionModule::getInstance();

	for (joint_iter = action_module->result_.begin(); joint_iter != action_module->result_.end(); ++joint_iter)
	{
		control_msg.joint_name.push_back(joint_iter->first);
		control_msg.module_name.push_back(module);
	}

	set_ctrl_module_pub.publish(control_msg);
}


bool MotionEditorServer::initialize()
{
	controller = robotis_framework::RobotisController::getInstance();
	ros::NodeHandle n;

	offset_file = n.param<std::string>("offset_file_path","");
	robot_file = n.param<std::string>("robot_file_path","");
	init_file = n.param<std::string>("init_file_path","");

	/* gazebo simulation */
	controller->gazebo_mode_ = n.param<bool>("gazebo", false);
	if (controller->gazebo_mode_ == true)
	{
		std::string robot_name = n.param<std::string>("gazebo_robot_name", "");
		if (robot_name != "")
			controller->gazebo_robot_name_ = robot_name;

		ROS_WARN("GAZEBO_MODE!!!!!!!!!!!!!!!");
	}
	action_module::ActionModule::getInstance()->gazebo_check = controller->gazebo_mode_;

	if ((offset_file == "") || (robot_file == ""))
	{
		ROS_ERROR("Failed to get file path");
		return -1;
	}
	//
	//Controller Initialize with robot file info
	if (controller->initialize(robot_file, init_file) == false)
	{
		ROS_ERROR("ROBOTIS Controller Initialize Fail!");
		return -1;
	}
	//controller->loadOffset(offset_file);
	controller->addMotionModule((robotis_framework::MotionModule*) ActionModule::getInstance());

	//Initialize RobotOffsetData
	for (std::map<std::string, robotis_framework::Dynamixel *>::iterator robot_it = controller->robot_->dxls_.begin();
			robot_it != controller->robot_->dxls_.end(); robot_it++)
	{
		std::string joint_name = robot_it->first;
		//robot_offset_data[joint_name] = new JointOffsetData();
		robot_torque_enable_data[joint_name] = true;
	}
	//Add here Communication
	command_state_sub = n.subscribe("/heroehs/command_state", 10, &MotionEditorServer::CommandStateMsgsCallBack, this);
	joint_goal_state_sub =  n.subscribe("/heroehs/joint_offset_state", 10, &MotionEditorServer::JointGoalStateMsgsCallBack, this);
	moving_state_pub = n.advertise<std_msgs::Bool>("/heroehs/moving_state", 10);

	joint_torque_on_off_ser = n.advertiseService("/heroehs/joint_torque_on_off", &MotionEditorServer::JointTorqueOnOffCallBack, this);
	present_joint_state_array_ser = n.advertiseService("/heroehs/present_joint_state_array", &MotionEditorServer::PresentJointStateArrayCallBack, this);
	editor_command_ser = n.advertiseService("/heroehs/editor_command", &MotionEditorServer::EditorCommandCallBack, this);
	return true;
}
void MotionEditorServer::CommandStateMsgsCallBack(const std_msgs::String::ConstPtr& msg)
{
	operating_command = msg->data;
	if(msg->data == "init_pose")
	{
		moveToPose();
	}
	else if(msg->data == "motion_play")
	{
		moveToPose();
	}
	else if(msg->data == "action_play")
	{
		moveToPose();
	}
	else
		ROS_INFO_STREAM("Invalid Command : " << msg->data);
}
void MotionEditorServer::JointGoalStateMsgsCallBack(const offset_tuner_msgs::JointOffsetState::ConstPtr& msg)
{
	if (controller->isTimerRunning())
	{
		ROS_ERROR("Timer is running now");
		return;
	}

	//goal position
	ROS_INFO_STREAM(msg->joint_name << " " << msg->joint_goal_value << " " );

	std::map<std::string, bool>::iterator it;
	it = robot_torque_enable_data.find(msg->joint_name);
	if (it == robot_torque_enable_data.end())
	{
		ROS_ERROR("Invalid Joint Name");
		return;
	}

	if (robot_torque_enable_data[msg->joint_name] == false)
	{
		ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
		return;
	}

	double  goal_pose_rad   = msg->joint_goal_value*DEGREE2RADIAN;
	int32_t goal_pose_value = controller->robot_->dxls_[msg->joint_name]->convertRadian2Value(goal_pose_rad);
	uint8_t dxl_error       = 0;
	int32_t comm_result     = COMM_SUCCESS;

	comm_result = controller->writeCtrlItem(msg->joint_name,
			controller->robot_->dxls_[msg->joint_name]->goal_position_item_->item_name_,
			goal_pose_value, &dxl_error);
	if (comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("Failed to write goal position");
		return;
	}
	if (dxl_error != 0)
	{
		ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error " << (int) dxl_error);
	}

	// robot_offset_data_[msg->joint_name]->p_gain_ = msg->p_gain;
	// robot_offset_data_[msg->joint_name]->i_gain_ = msg->i_gain;
	// robot_offset_data_[msg->joint_name]->d_gain_ = msg->d_gain;

}
bool MotionEditorServer::JointTorqueOnOffCallBack(offset_tuner_msgs::JointTorqueOnOff::Request &req, offset_tuner_msgs::JointTorqueOnOff::Response &res)
{
	std::string joint_name = req.torque_command.joint_name;
	bool torque_enable = req.torque_command.joint_torque_on_off;

	int32_t comm_result = COMM_SUCCESS;
	uint8_t dxl_error = 0;
	uint8_t torque_enable_value = 0;

	if (torque_enable)
		torque_enable_value = 1;
	else
		torque_enable_value = 0;

	comm_result = controller->writeCtrlItem(joint_name,
			controller->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
			torque_enable_value, &dxl_error);
	if (comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("Failed to write goal position");
	}
	else
	{
		robot_torque_enable_data[joint_name] = torque_enable;
	}

	if (dxl_error != 0)
	{
		ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error " << (int) dxl_error);
	}

	return true;
}
bool MotionEditorServer::PresentJointStateArrayCallBack(offset_tuner_msgs::PresentJointStateArray::Request &req, offset_tuner_msgs::PresentJointStateArray::Response &res)
{
	ROS_INFO("GetPresentJointOffsetDataService Called");
	res.joint_data.clear();

	for (std::map<std::string, bool>::iterator it = robot_torque_enable_data.begin();
			it != robot_torque_enable_data.end(); it++)
	{
		std::string       joint_name = it->first;
		bool joint_data = it->second;

		offset_tuner_msgs::PresentJointStateData joint_state_pos;
		int32_t torque_enable = 0;
		int32_t present_pos_value = 0;
		uint8_t dxl_error         = 0;
		int     comm_result       = COMM_SUCCESS;

		comm_result = controller->readCtrlItem(joint_name,
				controller->robot_->dxls_[joint_name]->present_position_item_->item_name_,
				(uint32_t*) &present_pos_value,
				&dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			ROS_ERROR("Failed to read present pos");
			return false;
		}

		usleep(10*1000);

		comm_result = controller->readCtrlItem(joint_name,
				controller->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
				(uint32_t *) &torque_enable,
				&dxl_error);

		if (comm_result != COMM_SUCCESS)
		{
			ROS_ERROR("Failed to read present pos");
			return false;
		}
		else
		{
			if (dxl_error != 0)
			{
				ROS_ERROR_STREAM(joint_name << "  has error " << (int) dxl_error);
			}

			joint_state_pos.joint_name  = joint_name;
			joint_state_pos.torque_state  = torque_enable;
			joint_state_pos.present_position_value = controller->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value)*RADIAN2DEGREE;

			res.joint_data.push_back(joint_state_pos);
		}
	}
	return true;
}
bool MotionEditorServer::EditorCommandCallBack(motion_editor_msgs::EditorCommand::Request &req, motion_editor_msgs::EditorCommand::Response &res)
{
	motion_data_path = ros::package::getPath("action_module") + "/motion_data/"+ req.file_name + ".yaml";
	if (!req.command.compare("stop_save") && pose_save_check == true)
	{
		YAML::Emitter out;
		out << YAML::BeginMap;
		for(int it = 0; it < pose_num ; it++)
		{
			out << YAML::Key << pose_key[it] <<YAML::Value << pose_data_array[it];
		}
		for(int it = 0; it < pose_num ; it++)
		{
			out << YAML::Key << pose_key_time[it] <<YAML::Value << pose_time[it];
		}
		out << YAML::EndMap;
		std::ofstream fout(motion_data_path.c_str());
		fout << out.c_str();

		pose_key.clear();
		pose_data.clear();
		pose_data_array.clear();
		pose_key_time.clear();
		pose_time.clear();
		pose_num = 0;
	}
	else if(!req.command.compare("record"))
	{
		for (std::map<std::string, bool>::iterator it = robot_torque_enable_data.begin();
				it != robot_torque_enable_data.end(); it++)
		{
			std::string       joint_name = it->first;
			int32_t present_pos_value = 0;
			uint8_t dxl_error         = 0;
			int     comm_result       = COMM_SUCCESS;

			comm_result = controller->readCtrlItem(joint_name,
					controller->robot_->dxls_[joint_name]->present_position_item_->item_name_,
					(uint32_t*) &present_pos_value,
					&dxl_error);
			if (comm_result != COMM_SUCCESS)
			{
				ROS_ERROR("Failed to read present pos");
				return false;
			}

			comm_result = controller->readCtrlItem(joint_name,
					controller->robot_->dxls_[joint_name]->present_position_item_->item_name_,
					(uint32_t *) &present_pos_value,
					&dxl_error);

			usleep(10*1000);
			if (comm_result != COMM_SUCCESS)
			{
				ROS_ERROR("Failed to read present pos");
				return false;
			}
			else
			{
				if (dxl_error != 0)
				{
					ROS_ERROR_STREAM(joint_name << "  has error " << (int) dxl_error);
				}
				for (int check_num = 0; check_num < req.part.size();check_num++)
				{
					if(!req.part[check_num].compare("arm_left"))
					{
						if(!joint_name.compare("l_shoulder_pitch")|| !joint_name.compare("l_shoulder_roll")|| !joint_name.compare("l_shoulder_yaw")|| !joint_name.compare("l_elbow_pitch"))
						{
							pose_data[joint_name] = controller->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value)*RADIAN2DEGREE;
							pose_save_check = true;
						}
					}
					if(!req.part[check_num].compare("arm_right"))
					{

						if(!joint_name.compare("r_shoulder_pitch")|| !joint_name.compare("r_shoulder_roll")|| !joint_name.compare("r_shoulder_yaw")|| !joint_name.compare("r_elbow_pitch"))
						{
							pose_data[joint_name] = controller->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value)*RADIAN2DEGREE;
							pose_save_check = true;
						}
					}
					if(!req.part[check_num].compare("head"))
					{
						if(!joint_name.compare("head_pitch")|| !joint_name.compare("head_yaw")|| !joint_name.compare("head_roll"))
						{
							pose_data[joint_name] = controller->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value)*RADIAN2DEGREE;
							pose_save_check = true;
						}
					}
					if(!req.part[check_num].compare("waist"))
					{
						if(!joint_name.compare("waist_yaw")|| !joint_name.compare("waist_roll"))
						{
							pose_data[joint_name] = controller->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value)*RADIAN2DEGREE;
							pose_save_check = true;
						}
					}
					if(pose_save_check == false)
					{
						res.result = "Nothing to record";
						//return true;
					}
					else
						res.result = "Recording";
				}
			}
		}
		char s1[30];
		sprintf(s1, "%d", pose_num);    // %d를 지정하여 정수를 문자열로 저장
		string str_temp(s1);

		pose_data_array.push_back(pose_data);
		pose_key.push_back("pose_" + str_temp);
		pose_key_time.push_back("pose_time_" + str_temp);
		pose_time.push_back(req.time);
		pose_num ++;
	}
	else if(!req.command.compare("pause"))
	{
		return true;
		//	pose_num --;
	}
	else if(!req.command.compare("motion_play") || !req.command.compare("action_play"))
	{
		motion_file_name = req.file_name;
		action_file_name = req.file_name;
	}
	else
	{
		ROS_INFO_STREAM("Nothing to record");
		res.result = "Nothing to record";
	}
	return true;
}
void MotionEditorServer::moveToPose()
{
	moving_state_msg.data = 1;
	moving_state_pub.publish(moving_state_msg);
	ActionModule *action_module = ActionModule::getInstance();

	action_module->is_playing = true;
	action_module->action_command = operating_command;

	if(!operating_command.compare("init_pose"))
	{
		action_module->initPose(operating_command);

	}
	else if(!operating_command.compare("motion_play"))
	{
		action_module->motion_file_name_temp = motion_file_name;
		action_module->ActionDataInitialize();

	}
	else if(!operating_command.compare("action_play"))
	{
		action_module->action_file_name_temp = action_file_name;
		//playSound("test1.WAV");
		action_module->ActionDataInitialize();
	}
	controller->startTimer();

	while (action_module->is_playing)
	{
usleep(100);
		if(!ros::ok())
		{
			controller->stopTimer();
			setCtrlModule("none");
		}
	}
	//usleep(1000*1000);

	controller->stopTimer();

	moving_state_msg.data = 0;
	moving_state_pub.publish(moving_state_msg);

	while (controller->isTimerRunning())
		usleep(10 * 1000);

	if (controller->isTimerRunning())
	{
		ROS_INFO("Timer Running");
	}

	setCtrlModule("none");
}
void MotionEditorServer::playSound(std::string file_name)
{
    //char *temp_char;
	std::string temp_str = ros::package::getPath("action_module") + "/sound_data/"+ file_name;

  //srand((unsigned int)time(NULL));
/*  int randomVal = rand();
  switch(msg->data)
  {
    case 1:
      temp = idle[randomVal%5];
      break;
    case 2:
      temp = pickup[randomVal%4];
      break;
    case 3:
      temp = touched[randomVal%4];
      break;
    case 4:
      temp = tracing[randomVal%5];
      break;
  }*/

  if(g_play_pid != -1)
    kill(g_play_pid, SIGKILL);

  g_play_pid = fork();

  switch(g_play_pid)
  {
  case -1:
    fprintf(stderr, "Fork Failed!! \n");
    break;
  case 0:
    execl("/usr/bin/aplay", "aplay", temp_str.c_str(), "", (char*)0);
    break;
  default:
    break;
  }
}



