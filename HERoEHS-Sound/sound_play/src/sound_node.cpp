/*
 * sound_node.cpp
 *
 *  Created on: Mar 19, 2018
 *      Author: robotemperor
 */
#include <ros/ros.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <fstream>
#include <stdio.h>

int state = 0;
pid_t g_play_pid = -1;

void playSound(const std_msgs::String::ConstPtr& msg)
{
	//std::string temp_str = "";
    //char *temp_char;
	std::string temp_str = ros::package::getPath("sound_play") + "/data/"+ msg->data +".WAV";

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_node");
  ros::NodeHandle nh;
  ros::Subscriber sound_sub = nh.subscribe<std_msgs::String>("/heroehs/sound", 0, playSound);

  while(ros::ok())
  {
	  ros::spinOnce();
	  usleep(1000);
  }
  return 0;
}




