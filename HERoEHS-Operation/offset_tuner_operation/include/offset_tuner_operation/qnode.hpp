/**
 * @file /include/offset_tuner_operation/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef offset_tuner_operation_QNODE_HPP_
#define offset_tuner_operation_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
/*****************************************************************************
** Message
*****************************************************************************/
#include "offset_tuner_msgs/JointOffsetState.h"
#include "offset_tuner_msgs/JointTorqueOnOffData.h"
#include "offset_tuner_msgs/JointTorqueOnOff.h"
#include "offset_tuner_msgs/JointTorqueOnOffArray.h"

#include "offset_tuner_msgs/PresentJointStateData.h"
#include "offset_tuner_msgs/PresentJointStateArray.h"

#include "motion_editor_msgs/EditorCommand.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace offset_tuner_operation {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	void MovingStateMsgsCallBack(const std_msgs::Bool::ConstPtr& msg);


	//publisher
	ros::Publisher joint_offset_state_pub;
	ros::Publisher command_state_pub;
	ros::Publisher enable_module_pub;
	ros::Publisher action_command_pub;
	ros::Publisher init_pose_pub;
	ros::Publisher sound_pub;



	//subscriber
	ros::Subscriber moving_state_sub;

	//service client
    ros::ServiceClient joint_torque_on_off_cl;
    ros::ServiceClient joint_torque_on_off_array_cl;
    ros::ServiceClient present_joint_state_array_cl;

    //motion editor
    ros::ServiceClient editor_command_cl;


	//message
	offset_tuner_msgs::JointOffsetState      joint_offset_state_msg;
	offset_tuner_msgs::JointTorqueOnOffData  joint_torque_on_off_data;
	offset_tuner_msgs::JointTorqueOnOff      joint_torque_on_off_srv;
	offset_tuner_msgs::JointTorqueOnOffArray joint_torque_on_off_array_srv;

	offset_tuner_msgs::PresentJointStateArray present_joint_state_array_srv;
	offset_tuner_msgs::PresentJointStateData  present_joint_state_data;

	std_msgs::String command_state_msg;
	std_msgs::String enable_module_msg;
	std_msgs::String action_command_msg;
	std_msgs::String init_pose_msg;
	std_msgs::String sound_msg;

	//motion
	motion_editor_msgs::EditorCommand editor_command_srv;

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

}  // namespace offset_tuner_operation

#endif /* offset_tuner_operation_QNODE_HPP_ */
