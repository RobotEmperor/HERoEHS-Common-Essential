/*
 * main.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: robotemperor
 */
#include <ros/ros.h>

#include "motion_editor_server/motion_editor_server.h"

using namespace motion_editor;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offset_tuner_server_node");

	MotionEditorServer* server = MotionEditorServer::getInstance();

	server->initialize();


	while(ros::ok())
{
usleep(100);
ros::spinOnce();
}


	return 0;
}




