/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <stdio.h>
#include "../include/offset_tuner_operation/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace offset_tuner_operation {

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent)
, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	qnode.init();
	setWindowIcon(QIcon(":/images/icon.png"));

	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/****************************
	 ** Connect
	 ****************************/
	// offset tab
	QObject::connect(ui.goal_spin_box, SIGNAL(valueChanged(QString)), this, SLOT(offset_goal_value_changed_function()));
	QObject::connect(ui.torque_state_check_box, SIGNAL(clicked()), this, SLOT(offset_torque_state_changed_function()));
	QObject::connect(ui.change_button, SIGNAL(clicked()), this, SLOT(offset_change_button()));

	// motion editor
	QObject::connect(ui.motion_editor_goal_spin_box, SIGNAL(valueChanged(QString)), this, SLOT(motion_editor_goal_value_changed_function()));
	QObject::connect(ui.motion_editor_torque_state_check_box, SIGNAL(clicked()), this, SLOT(motion_editor_torque_state_changed_function()));
	QObject::connect(ui.motion_editor_update_button, SIGNAL(clicked()), this, SLOT(on_update_button_clicked()));
	QObject::connect(ui.motion_editor_change_button, SIGNAL(clicked()), this, SLOT(motion_editor_change_button()));


	/*********************
	 ** Logging
	 **********************/
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

	/****************************
	 ** Initialize  load joint data
	 ****************************/
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("offset_tuner_operation") + "/config/joint_data.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	joint_data_parse(init_pose_path);
	ui.id_line_edit->setText("0");
	ui.motion_editor_id_line_edit->setText("0");
	pose_num = 0;
	action_num = 0;
	action_num_str = "";
}
MainWindow::~MainWindow() {}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
	close();
}
/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/
void MainWindow::on_update_button_clicked()
{
	qnode.present_joint_state_array_srv.request.update_check = true;
	if(qnode.present_joint_state_array_cl.call(qnode.present_joint_state_array_srv))
	{
		for(int num = 0; num < qnode.present_joint_state_array_srv.response.joint_data.size(); num ++)
		{
			for (std::map<int, std::string>::iterator it = joint_id_to_name.begin(); it != joint_id_to_name.end(); ++it) //
			{
				std::string joint_name;
				joint_name = it->second;
				if(!qnode.present_joint_state_array_srv.response.joint_data[num].joint_name.compare(joint_name))
				{
					joint_name_to_torque_state[joint_name]  = qnode.present_joint_state_array_srv.response.joint_data[num].torque_state;
					joint_name_to_present_value[joint_name] = qnode.present_joint_state_array_srv.response.joint_data[num].present_position_value;
					joint_name_to_offset_data[joint_name]   = qnode.present_joint_state_array_srv.response.joint_data[num].offset_data;
					joint_id_to_goal[joint_name_to_id[joint_name]] = qnode.present_joint_state_array_srv.response.joint_data[num].present_position_value;

				}
			}
		}
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
}
void MainWindow::on_all_torque_on_button_clicked()
{
	torque_state(1);
	on_update_button_clicked();
	usleep(10*1000);
	change_button(ui.id_line_edit->text(), "offset");

}
void MainWindow::on_all_torque_off_button_clicked()
{
	torque_state(0);
	on_update_button_clicked();
	usleep(10*1000);
	change_button(ui.id_line_edit->text(), "offset");
}
void MainWindow::on_initial_pose_button_clicked()
{
	qnode.log(qnode.Info, "zero_offset_pose");
	qnode.command_state_msg.data = "init_offset_pose_zero";
	qnode.command_state_pub.publish(qnode.command_state_msg);

}
void MainWindow::on_pre_offset_pose_button_clicked()
{
	qnode.log(qnode.Info, "pre_offset_pose");
	qnode.command_state_msg.data = "init_offset_pose";
	qnode.command_state_pub.publish(qnode.command_state_msg);

}
void MainWindow::on_save_button_clicked()
{
	qnode.command_state_msg.data = "save";
	qnode.command_state_pub.publish(qnode.command_state_msg);
}
void MainWindow::on_base_module_button_clicked()
{
	qnode.enable_module_msg.data = "base_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::torque_state(bool state)
{
	qnode.joint_torque_on_off_array_srv.request.torque_command.clear();

	for (std::map<int, std::string>::iterator it = joint_id_to_name.begin(); it != joint_id_to_name.end(); ++it) //
	{
		std::string joint_name;
		joint_name = it->second;

		qnode.joint_torque_on_off_data.joint_name = joint_name;
		qnode.joint_torque_on_off_data.joint_torque_on_off = state;
		qnode.joint_torque_on_off_array_srv.request.torque_command.push_back(qnode.joint_torque_on_off_data);
	}
	if(qnode.joint_torque_on_off_array_cl.call(qnode.joint_torque_on_off_array_srv))
	{
		qnode.log(qnode.Info, "Send command to sever complete [All Torque]");
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
}
// offset
void MainWindow::offset_torque_state_changed_function()
{
	torque_state_changed_function(ui.id_line_edit->text(), ui.torque_state_check_box->isChecked());
}
void MainWindow::offset_goal_value_changed_function()
{
	goal_value_changed_function(ui.id_line_edit->text(), ui.goal_spin_box->value());
}
void MainWindow::offset_change_button()
{
	on_update_button_clicked();
	usleep(10*1000);
	ui.motion_editor_id_line_edit->setText(ui.id_line_edit->text());
	change_button(ui.id_line_edit->text(),"offset");
}
//motion
void MainWindow::motion_editor_torque_state_changed_function()
{
	torque_state_changed_function(ui.motion_editor_id_line_edit->text(), ui.motion_editor_torque_state_check_box->isChecked());
}

void MainWindow::motion_editor_goal_value_changed_function()
{
	goal_value_changed_function(ui.motion_editor_id_line_edit->text(), ui.motion_editor_goal_spin_box->value());
}
void MainWindow::motion_editor_change_button()
{
	on_update_button_clicked();
	usleep(10*1000);
	ui.id_line_edit->setText(ui.motion_editor_id_line_edit->text());
	change_button(ui.motion_editor_id_line_edit->text(), "motion_editor");
}
void MainWindow::on_motion_editor_record_button_clicked()
{
	qnode.editor_command_srv.request.part.clear();
	qnode.editor_command_srv.request.file_name = ui.motion_editor_file_name_line_edit -> text().toStdString();
	qnode.editor_command_srv.request.time = ui.motion_editor_time_line_edit -> text().toDouble();

	if(ui.motion_editor_arm_left_check_box->isChecked() || ui.motion_editor_arm_right_check_box->isChecked() ||
			ui.motion_editor_head_check_box->isChecked()     || ui.motion_editor_waist_check_box->isChecked())
	{
		if(ui.motion_editor_arm_left_check_box->isChecked())
			qnode.editor_command_srv.request.part.push_back("arm_left");
		if(ui.motion_editor_arm_right_check_box->isChecked())
			qnode.editor_command_srv.request.part.push_back("arm_right");
		if(ui.motion_editor_head_check_box->isChecked())
			qnode.editor_command_srv.request.part.push_back("head");
		if(ui.motion_editor_waist_check_box->isChecked())
			qnode.editor_command_srv.request.part.push_back("waist");
	}
	else
	{
		qnode.log(qnode.Error, "Please Check Part <SEND FAIL>");
		return;
	}
	qnode.editor_command_srv.request.command = "record";
	if(qnode.editor_command_cl.call(qnode.editor_command_srv))
	{
		if(qnode.editor_command_srv.response.result.compare("Nothing to record"))
		{
			qnode.log(qnode.Info, "Record complete!");
			pose_num ++;
		}
		else
		{
			qnode.log(qnode.Info, "Nothing to record! Check your joint name!");
		}
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}

	ui.motion_editor_pose_num_line_edit->setText(QString::number(pose_num));
}
void MainWindow::on_motion_editor_stop_save_button_clicked()
{
	pose_num = 0;
	qnode.editor_command_srv.request.file_name = ui.motion_editor_file_name_line_edit -> text().toStdString();
	qnode.editor_command_srv.request.command = "stop_save";
	if(qnode.editor_command_cl.call(qnode.editor_command_srv))
	{
		if(qnode.editor_command_srv.response.result.compare("Nothing to record"))
		{
			qnode.log(qnode.Info, "SAVE complete!");
		}
		else
		{
			qnode.log(qnode.Info, "Nothing to Save! Check your joint name!");
		}
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}

	ui.motion_editor_pose_num_line_edit->setText(QString::number(pose_num));
}
void MainWindow::on_motion_editor_pause_button_clicked()
{
	qnode.editor_command_srv.request.file_name = ui.motion_editor_file_name_line_edit -> text().toStdString();
	qnode.editor_command_srv.request.command = "pause";
	if(qnode.editor_command_cl.call(qnode.editor_command_srv))
	{
		qnode.log(qnode.Info, "Pause");
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
}
void MainWindow::on_motion_editor_play_button_clicked()
{
	pose_num = 0;
	qnode.editor_command_srv.request.file_name = ui.motion_editor_file_name_line_edit -> text().toStdString();
	qnode.editor_command_srv.request.command = "motion_play";
	if(qnode.editor_command_cl.call(qnode.editor_command_srv))
	{
		qnode.log(qnode.Info, "Motion Play command sended");
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
	qnode.log(qnode.Info, "motion_play");
	qnode.command_state_msg.data = "motion_play";
	qnode.command_state_pub.publish(qnode.command_state_msg);

	ui.motion_editor_pose_num_line_edit->setText(QString::number(pose_num));
}
void MainWindow::on_motion_editor_module_button_clicked()
{
	qnode.enable_module_msg.data = "action_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::on_motion_editor_initial_pose_button_clicked()
{
	qnode.log(qnode.Info, "init_pose");
	qnode.command_state_msg.data = "init_pose";
	qnode.command_state_pub.publish(qnode.command_state_msg);
}

void MainWindow::on_action_1_button_clicked()
{
	send_action_command("action1");
}
void MainWindow::on_action_2_button_clicked()
{
	send_action_command("action2");
}
void MainWindow::on_action_3_button_clicked()
{
	send_action_command("action3");
}
void MainWindow::on_action_4_button_clicked()
{
	send_action_command("action4");
}
void MainWindow::on_action_5_button_clicked()
{
	send_action_command("action5");
}
void MainWindow::on_action_6_button_clicked()
{
	send_action_command("action6");
}
void MainWindow::on_action_7_button_clicked()
{
	send_action_command("action7");
}
void MainWindow::on_action_play_decrease_button_clicked()
{
	action_num = ui.action_number_line_edit -> text().toDouble();
	action_num --;
	if(action_num < 0)
		action_num = 0;

	ui.action_number_line_edit->setText(QString::number(action_num));
}
void MainWindow::on_action_play_increase_button_clicked()
{
	action_num = ui.action_number_line_edit -> text().toDouble();

	action_num ++;

	ui.action_number_line_edit->setText(QString::number(action_num));
}
void MainWindow::on_action_start_button_clicked()
{
	action_num = ui.action_number_line_edit -> text().toDouble();
	ui.action_number_line_edit->setText(QString::number(action_num));

	std::stringstream temp_str;
	temp_str << action_num;
	action_num_str = "action" + temp_str.str();

	printf("%s", action_num_str.c_str());

	send_action_command(action_num_str);
}
/*void MainWindow::on_action_init_pose_button_clicked()
{
	qnode.init_pose_msg.data ="init_pose";
	qnode.init_pose_pub.publish(qnode.init_pose_msg);
}*/
void MainWindow::on_editor_pause_button_2_clicked()
{
	qnode.action_command_msg.data ="pause";
	qnode.action_command_pub.publish(qnode.action_command_msg);
}
void MainWindow::on_action_play_button_clicked()
{
	pose_num = 0;
	qnode.editor_command_srv.request.file_name = ui.motion_editor_file_name_line_edit -> text().toStdString();
	qnode.editor_command_srv.request.command = "action_play";
	if(qnode.editor_command_cl.call(qnode.editor_command_srv))
	{
		qnode.log(qnode.Info, "Action Play command sended");
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
	qnode.log(qnode.Info, "action_play");
	qnode.command_state_msg.data = "action_play";
	qnode.command_state_pub.publish(qnode.command_state_msg);

	qnode.sound_msg.data = ui.motion_editor_file_name_line_edit -> text().toStdString();
	qnode.sound_pub.publish(qnode.sound_msg);

	ui.motion_editor_pose_num_line_edit->setText(QString::number(pose_num));

}
/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/
void MainWindow::updateLoggingView() {
	ui.view_logging->scrollToBottom();
}
void MainWindow::goal_value_changed_function(QString id_string, double spin_value)
{
	int id_int;

	id_int = id_string.toInt();

	if(joint_id_to_name.count(id_int))
	{
		joint_id_to_goal[id_int] =  spin_value;
		qnode.joint_offset_state_msg.joint_name = joint_id_to_name[id_int];
		qnode.joint_offset_state_msg.joint_goal_value =  spin_value;
		qnode.joint_offset_state_pub.publish(qnode.joint_offset_state_msg);
		qnode.log(qnode.Info, "Id confirm and Send Goal value!!");
	}
	else
	{
		qnode.log(qnode.Error, "Check ID number <SEND FAIL>");
	}
}
void MainWindow::torque_state_changed_function(QString id_string, bool check)
{
	int id_int;
	id_int = id_string.toInt();

	if(joint_id_to_name.count(id_int))
	{
		qnode.joint_torque_on_off_srv.request.torque_command.joint_name = joint_id_to_name[id_int];
		qnode.joint_torque_on_off_srv.request.torque_command.joint_torque_on_off = check;
		if(qnode.joint_torque_on_off_cl.call(qnode.joint_torque_on_off_srv))
		{
			qnode.log(qnode.Info, "Send command to sever complete [One Torque]");
		}
		else
		{
			qnode.log(qnode.Error, "Check communication <SEND FAIL>");
		}
	}
	else
	{
		qnode.log(qnode.Error, "Check ID number <SEND FAIL>");
	}

}
void MainWindow::joint_data_parse(const std::string &path)
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

	YAML::Node joint_data = doc["joint_data"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = joint_data.begin(); it != joint_data.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int id;
		std::string joint_name;
		// 한 줄에서 int 와 double 을 분리한다.
		id = it->first.as<int>();
		joint_name = it->second.as<std::string>();

		//joint 정보 저장
		joint_name_to_id[joint_name] = id;
		joint_id_to_name[id] = joint_name;
		joint_id_to_goal[id] = 0;
	}
}
void MainWindow::change_button(QString id_string, std::string type)
{
	int id_int;
	id_int = id_string.toInt();
	if(joint_id_to_name.count(id_int))
	{
		//offset
		if(!type.compare("offset"))
		{
			ui.goal_spin_box->setValue(joint_id_to_goal[id_int]);
			ui.link_name_line_edit->setText(QString::fromStdString(joint_id_to_name[id_int]));
			ui.torque_state_check_box->setChecked(joint_name_to_torque_state[joint_id_to_name[id_int]]);
			ui.offset_line_edit->setText(QString::number(joint_name_to_offset_data[joint_id_to_name[id_int]]));
			ui.present_line_edit->setText(QString::number(joint_name_to_present_value[joint_id_to_name[id_int]]));
		}
		else if(!type.compare("motion_editor"))
		{
			//motion
			ui.motion_editor_goal_spin_box->setValue(joint_id_to_goal[id_int]);
			ui.motion_editor_present_line_edit->setText(QString::number(joint_name_to_present_value[joint_id_to_name[id_int]]));
			ui.motion_editor_joint_name_line_edit->setText(QString::fromStdString(joint_id_to_name[id_int]));
			ui.motion_editor_torque_state_check_box->setChecked(joint_name_to_torque_state[joint_id_to_name[id_int]]);
		}
		else
			return;
	}
	else
	{
		qnode.log(qnode.Error, "Check ID number <SEND FAIL>");
	}
}
void MainWindow::send_action_command(std::string action_command_str)
{
	pose_num = 0;
	qnode.editor_command_srv.request.file_name =  action_command_str;
	qnode.editor_command_srv.request.command = "action_play";
	if(qnode.editor_command_cl.call(qnode.editor_command_srv))
	{
		qnode.log(qnode.Info, "Action Play command sended");
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
	qnode.log(qnode.Info, "action_play");
	qnode.command_state_msg.data = "action_play";
	qnode.command_state_pub.publish(qnode.command_state_msg);

	qnode.sound_msg.data =  action_command_str;
	qnode.sound_pub.publish(qnode.sound_msg);

	ui.motion_editor_pose_num_line_edit->setText(QString::number(pose_num));

}
void MainWindow::closeEvent(QCloseEvent *event)
{
	//WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace offset_tuner_operation

