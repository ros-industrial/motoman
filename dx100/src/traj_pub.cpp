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
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <sstream>

using std::cout;
using std::cin;
using std::endl;

int main(int argc, char **argv)
// Publishes trajectory to joint_cmd topic to be received by interface node
{
  const int NUMPOINTS = 100;
  int traj_points[NUMPOINTS][8] = {{0, 0, -90000, 0, 0, 0, 0, 0},
{200, 200, -90200, 200, 200, 200, 200, 0},
{400, 400, -90400, 400, 400, 400, 400, 0},
{600, 600, -90600, 600, 600, 600, 600, 0},
{800, 800, -90800, 800, 800, 800, 800, 0},
{1000, 1000, -91000, 1000, 1000, 1000, 1000, 0},
{1200, 1200, -91200, 1200, 1200, 1200, 1200, 0},
{1400, 1400, -91400, 1400, 1400, 1400, 1400, 0},
{1600, 1600, -91600, 1600, 1600, 1600, 1600, 0},
{1800, 1800, -91800, 1800, 1800, 1800, 1800, 0},
{2000, 2000, -92000, 2000, 2000, 2000, 2000, 0},
{2200, 2200, -92200, 2200, 2200, 2200, 2200, 0},
{2400, 2400, -92400, 2400, 2400, 2400, 2400, 0},
{2600, 2600, -92600, 2600, 2600, 2600, 2600, 0},
{2800, 2800, -92800, 2800, 2800, 2800, 2800, 0},
{3000, 3000, -93000, 3000, 3000, 3000, 3000, 0},
{3200, 3200, -93200, 3200, 3200, 3200, 3200, 0},
{3400, 3400, -93400, 3400, 3400, 3400, 3400, 0},
{3600, 3600, -93600, 3600, 3600, 3600, 3600, 0},
{3800, 3800, -93800, 3800, 3800, 3800, 3800, 0},
{4000, 4000, -94000, 4000, 4000, 4000, 4000, 0},
{4200, 4200, -94200, 4200, 4200, 4200, 4200, 0},
{4400, 4400, -94400, 4400, 4400, 4400, 4400, 0},
{4600, 4600, -94600, 4600, 4600, 4600, 4600, 0},
{4800, 4800, -94800, 4800, 4800, 4800, 4800, 0},
{5000, 5000, -95000, 5000, 5000, 5000, 5000, 0},
{5200, 5200, -95200, 5200, 5200, 5200, 5200, 0},
{5400, 5400, -95400, 5400, 5400, 5400, 5400, 0},
{5600, 5600, -95600, 5600, 5600, 5600, 5600, 0},
{5800, 5800, -95800, 5800, 5800, 5800, 5800, 0},
{6000, 6000, -96000, 6000, 6000, 6000, 6000, 0},
{6200, 6200, -96200, 6200, 6200, 6200, 6200, 0},
{6400, 6400, -96400, 6400, 6400, 6400, 6400, 0},
{6600, 6600, -96600, 6600, 6600, 6600, 6600, 0},
{6800, 6800, -96800, 6800, 6800, 6800, 6800, 0},
{7000, 7000, -97000, 7000, 7000, 7000, 7000, 0},
{7200, 7200, -97200, 7200, 7200, 7200, 7200, 0},
{7400, 7400, -97400, 7400, 7400, 7400, 7400, 0},
{7600, 7600, -97600, 7600, 7600, 7600, 7600, 0},
{7800, 7800, -97800, 7800, 7800, 7800, 7800, 0},
{8000, 8000, -98000, 8000, 8000, 8000, 8000, 0},
{8200, 8200, -98200, 8200, 8200, 8200, 8200, 0},
{8400, 8400, -98400, 8400, 8400, 8400, 8400, 0},
{8600, 8600, -98600, 8600, 8600, 8600, 8600, 0},
{8800, 8800, -98800, 8800, 8800, 8800, 8800, 0},
{9000, 9000, -99000, 9000, 9000, 9000, 9000, 0},
{9200, 9200, -99200, 9200, 9200, 9200, 9200, 0},
{9400, 9400, -99400, 9400, 9400, 9400, 9400, 0},
{9600, 9600, -99600, 9600, 9600, 9600, 9600, 0},
{9800, 9800, -99800, 9800, 9800, 9800, 9800, 0},
{10000, 10000, -100000, 10000, 10000, 10000, 10000, 0},
{10200, 10200, -100200, 10200, 10200, 10200, 10200, 0},
{10400, 10400, -100400, 10400, 10400, 10400, 10400, 0},
{10600, 10600, -100600, 10600, 10600, 10600, 10600, 0},
{10800, 10800, -100800, 10800, 10800, 10800, 10800, 0},
{11000, 11000, -101000, 11000, 11000, 11000, 11000, 0},
{11200, 11200, -101200, 11200, 11200, 11200, 11200, 0},
{11400, 11400, -101400, 11400, 11400, 11400, 11400, 0},
{11600, 11600, -101600, 11600, 11600, 11600, 11600, 0},
{11800, 11800, -101800, 11800, 11800, 11800, 11800, 0},
{12000, 12000, -102000, 12000, 12000, 12000, 12000, 0},
{12200, 12200, -102200, 12200, 12200, 12200, 12200, 0},
{12400, 12400, -102400, 12400, 12400, 12400, 12400, 0},
{12600, 12600, -102600, 12600, 12600, 12600, 12600, 0},
{12800, 12800, -102800, 12800, 12800, 12800, 12800, 0},
{13000, 13000, -103000, 13000, 13000, 13000, 13000, 0},
{13200, 13200, -103200, 13200, 13200, 13200, 13200, 0},
{13400, 13400, -103400, 13400, 13400, 13400, 13400, 0},
{13600, 13600, -103600, 13600, 13600, 13600, 13600, 0},
{13800, 13800, -103800, 13800, 13800, 13800, 13800, 0},
{14000, 14000, -104000, 14000, 14000, 14000, 14000, 0},
{14200, 14200, -104200, 14200, 14200, 14200, 14200, 0},
{14400, 14400, -104400, 14400, 14400, 14400, 14400, 0},
{14600, 14600, -104600, 14600, 14600, 14600, 14600, 0},
{14800, 14800, -104800, 14800, 14800, 14800, 14800, 0},
{15000, 15000, -105000, 15000, 15000, 15000, 15000, 0},
{15200, 15200, -105200, 15200, 15200, 15200, 15200, 0},
{15400, 15400, -105400, 15400, 15400, 15400, 15400, 0},
{15600, 15600, -105600, 15600, 15600, 15600, 15600, 0},
{15800, 15800, -105800, 15800, 15800, 15800, 15800, 0},
{16000, 16000, -106000, 16000, 16000, 16000, 16000, 0},
{16200, 16200, -106200, 16200, 16200, 16200, 16200, 0},
{16400, 16400, -106400, 16400, 16400, 16400, 16400, 0},
{16600, 16600, -106600, 16600, 16600, 16600, 16600, 0},
{16800, 16800, -106800, 16800, 16800, 16800, 16800, 0},
{17000, 17000, -107000, 17000, 17000, 17000, 17000, 0},
{17200, 17200, -107200, 17200, 17200, 17200, 17200, 0},
{17400, 17400, -107400, 17400, 17400, 17400, 17400, 0},
{17600, 17600, -107600, 17600, 17600, 17600, 17600, 0},
{17800, 17800, -107800, 17800, 17800, 17800, 17800, 0},
{18000, 18000, -108000, 18000, 18000, 18000, 18000, 0},
{18200, 18200, -108200, 18200, 18200, 18200, 18200, 0},
{18400, 18400, -108400, 18400, 18400, 18400, 18400, 0},
{18600, 18600, -108600, 18600, 18600, 18600, 18600, 0},
{18800, 18800, -108800, 18800, 18800, 18800, 18800, 0},
{19000, 19000, -109000, 19000, 19000, 19000, 19000, 0},
{19200, 19200, -109200, 19200, 19200, 19200, 19200, 0},
{19400, 19400, -109400, 19400, 19400, 19400, 19400, 0},
{19600, 19600, -109600, 19600, 19600, 19600, 19600, 0},
{19800, 19800, -109800, 19800, 19800, 19800, 19800, 0}};

  ros::init(argc, argv, "traj_pub");
  ros::NodeHandle n;
  ros::Publisher joint_cmd_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_cmd", 1000);
  ros::Rate loop_rate(.5); // Publish trajectory at 1 Hz

  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("S");
  traj.joint_names.push_back("L");
  traj.joint_names.push_back("U");
  traj.joint_names.push_back("R");
  traj.joint_names.push_back("B");
  traj.joint_names.push_back("T");
  traj.joint_names.push_back("E");
  traj.joint_names.push_back("8th axis");

  traj.header.stamp = ros::Time::now() + ros::Duration(1.0);

  traj.points.resize(NUMPOINTS);
  
  for (int i = 0; i < NUMPOINTS; i++)
  {
    traj.points[i].positions.resize(8);
    for (short j = 0; j < 8; j++)
    {
      traj.points[i].positions[j] = traj_points[i][j];
    }
    traj.points[i].velocities.resize(1);
    traj.points[i].velocities[0] = 2000;
    traj.points[i].time_from_start = ros::Duration(i);
  }

  /*
  traj.points.resize(2);
  int i = 0;

  traj.header.stamp = ros::Time::now() + ros::Duration(1.0);

  traj.points[i].positions.resize(8);
  traj.points[i].positions[0] = 0;
  traj.points[i].positions[1] = 0;
  traj.points[i].positions[2] = -90000;
  traj.points[i].positions[3] = 0;
  traj.points[i].positions[4] = 0;
  traj.points[i].positions[5] = 0;
  traj.points[i].positions[6] = 0;
  traj.points[i].positions[7] = 0;

  traj.points[i].velocities.resize(1);
  traj.points[i].velocities[0] = 0;

  traj.points[i].time_from_start = ros::Duration(1.0);

  i++;

  traj.points[i].positions.resize(8);
  traj.points[i].positions[0] = 200;
  traj.points[i].positions[1] = 200;
  traj.points[i].positions[2] = -90200;
  traj.points[i].positions[3] = 200;
  traj.points[i].positions[4] = 200;
  traj.points[i].positions[5] = 200;
  traj.points[i].positions[6] = 200;
  traj.points[i].positions[7] = 200;

  traj.points[i].velocities.resize(1);
  traj.points[i].velocities[0] = 0;

  traj.points[i].time_from_start = ros::Duration(2.0);
  */
  
  while (ros::ok())
  {
    joint_cmd_pub.publish(traj);
    cout << "Trajectory sent" << endl;
    loop_rate.sleep();
  }
  
  return 0;
}
