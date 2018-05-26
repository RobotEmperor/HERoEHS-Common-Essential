/**
 * @file /include/offset_tuner_operation/main_window.hpp
 *
 * @brief Qt based gui for offset_tuner_operation.
 *
 * @date November 2010
 **/
#ifndef offset_tuner_operation_MAIN_WINDOW_H
#define offset_tuner_operation_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <QString>
#include <sstream>
#include "ui_main_window.h"
#include "qnode.hpp"
#endif

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace offset_tuner_operation {

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void closeEvent(QCloseEvent *event);
	void showNoMasterMessage();

	public Q_SLOTS:
	void updateLoggingView(); // no idea why this can't connect automatically
	//offset tabButton
	void on_update_button_clicked();
	void on_all_torque_on_button_clicked();
	void on_all_torque_off_button_clicked();

	void on_initial_pose_button_clicked();
	void on_pre_offset_pose_button_clicked();
	void on_save_button_clicked();
	void on_base_module_button_clicked();

	void offset_change_button();
	void offset_goal_value_changed_function();
	void offset_torque_state_changed_function();


	//motion editor button
	void motion_editor_goal_value_changed_function();
	void motion_editor_torque_state_changed_function();
	void motion_editor_change_button();
	void on_motion_editor_record_button_clicked();
	void on_motion_editor_stop_save_button_clicked();
	void on_motion_editor_pause_button_clicked();
	void on_motion_editor_play_button_clicked();

	void on_motion_editor_module_button_clicked();
	void on_motion_editor_initial_pose_button_clicked();


	//common
	void change_button(QString id_string, std::string type);
	void goal_value_changed_function(QString id_string, double spin_value);
	void torque_state_changed_function(QString id_string, bool check);
	void torque_state(bool state);


	//action player
	void on_action_1_button_clicked();
	void on_action_2_button_clicked();
	void on_action_3_button_clicked();
	void on_action_4_button_clicked();
	void on_action_5_button_clicked();
	void on_action_6_button_clicked();
	void on_action_7_button_clicked();

	void on_action_play_decrease_button_clicked();
	void on_action_play_increase_button_clicked();
	void on_action_start_button_clicked();


	//void on_action_init_pose_button_clicked();

	void on_editor_pause_button_2_clicked();
	void on_action_play_button_clicked();

	void send_action_command(std::string action_command_str);


	private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	void joint_data_parse(const std::string &path);

	std::map<std::string, int> joint_name_to_id;
	std::map<int, double>      joint_id_to_goal;
	std::map<int, std::string> joint_id_to_name;

	std::map<std::string, bool>   joint_name_to_torque_state;
	std::map<std::string, double> joint_name_to_present_value;
	std::map<std::string, double> joint_name_to_offset_data;

	int pose_num;
	int action_num;
	std::string action_num_str;
};

}  // namespace offset_tuner_operation

#endif // offset_tuner_operation_MAIN_WINDOW_H
