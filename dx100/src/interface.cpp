/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Yaskawa America, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*       * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Yaskawa America, Inc., nor the names 
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/ 

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "moto_socket.h"
#include "utils.h"
#include "p_var_q.h"
#include "definitions.h"
#include <fstream>

using std::cout;
using std::cin;
using std::endl;
using std::ofstream;

int main(int argc, char** argv)
// Connects to robot, subscribes to joint_cmd topic to receive trajectories and execute them on the roboot
{
  char buff[1024] = "192.168.10.20"; // Robot IP address

  ros::init(argc, argv, "motion_interface");
  ros::NodeHandle n;

  MotoSocket sock(buff, MOTION_PORT);
  ofstream* log_file = new ofstream;
  log_file->open("comm_log.txt");
  PVarQ pvq(&sock, log_file);

  ros::Subscriber sub = n.subscribe("joint_cmd", 1000, &PVarQ::addTraj, &pvq);
  cout << "subscription started" << endl;

  while (ros::ok())
  {
    ros::spinOnce();
    pvq.run();
  }

  log_file->close();
  delete log_file;
  return 0;
}
