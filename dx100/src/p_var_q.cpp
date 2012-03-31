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

#include "p_var_q.h"

using std::cout;
using std::cin;
using std::endl;
using std::ofstream;

PVarQ::PVarQ(MotoSocket *sock, ofstream* log_file)
// Constructor for position variable queue object
// Adds initialization message to queue and stores socket
{
  this->sock = sock;
  ack_received = true;
  num_traj_recv = 0;
  num_traj_send = 0;
  this->log_file = log_file;
}

PVarQ::~PVarQ(void)
// Destruct for position variable queue object
{
}

void PVarQ::addTraj(const trajectory_msgs::JointTrajectory& traj)
// Adds received trajectory to messages queue, taking care to add necessary commands
{
  cout << "traj received" << endl;
  const int NUMPOINTS = traj.points.size();
  message pvq_message;

  if (messages.empty())
    addMessage(CMD_INIT_PVQ);
  else if (messages.back().contents[0] == CMD_END_PVQ)
    addMessage(CMD_INIT_PVQ);

  memset(pvq_message.contents, UNUSED, sizeof(pvq_message.contents));
  pvq_message.contents[0] = CMD_ADD_POINT_PVQ;
  
  for (int i = 0; i < NUMPOINTS; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      pvq_message.contents[j+2] = traj.points[i].positions[j];
    }
    pvq_message.contents[10] = traj.points[i].velocities[0];
    messages.push(pvq_message);
  }

  if (NUMPOINTS >= QSIZE) // Send trajectory if it has enough points for the robot to move
  {
    /*
    Comment out the following line before if you don't want to end the trajectory QSIZE points early, and you'd rather
    send the CMD_END_PVQ command manually. If you don't send it before the robot runs out of new points, though, it
    will oscillate in the job along the most recent Position Variables in the loop.
    */
    addMessage(CMD_END_PVQ); // Add on PVQ end message
    num_traj_recv++;
    //sendTraj(); // Uncomment if using motion_interface instead of interface
  }
}

void PVarQ::sendTraj()
// Talks to motoros_server in MotoPlus via UDP. Initiates PVQ motion, sends trajectory, stops motion. Blocks during motion.
{
  message send_message;
  message recv_message;
  int bytes_send, bytes_recv;

  while (!messages.empty())
  {
    send_message = messages.front();
    for (short i = 0; i < 11; i++)
    {
      cout << send_message.contents[i] << " ";
    }
    cout << endl;
    bytes_send = sock->sendMessage(send_message.contents, true);
    if (bytes_send == -1)
      break;
    bytes_recv = sock->recvMessage(recv_message.contents, true);
    if (bytes_recv == -1)
      break;
    for (short i = 0; i < 11; i++)
    {
      cout << recv_message.contents[i] << " ";
    }
    cout << endl;
    messages.pop();
  }
  addMessage(CMD_INIT_PVQ);
}

void PVarQ::addMessage(int command)
// Adds message to messages queue
{
  message new_message;
  memset(new_message.contents, UNUSED, sizeof(new_message.contents));
  new_message.contents[0] = command;
  messages.push(new_message);
}

int PVarQ::sendMessage()
// Performs non-blocking send on next message in queue. If successful, logs message and sets "ack_received" flag to "false."
{
  int bytes_send = sock->sendMessage(messages.front().contents, false);
  if (bytes_send > 0)
  {
    ack_received = false;
    logMessage(messages.front(), true);
    messages.pop();
  }
  return bytes_send;
}

int PVarQ::recvMessage()
// Receives (non-blocking) message from robot. If successful, logs message and sets "ack_received" flag to "true."
{
  message recv_message;
  memset(recv_message.contents, 0, 44);
  int bytes_recv = sock->recvMessage(recv_message.contents, false);
  if (bytes_recv > 0)
  {
    ack_received = true;
    logMessage(recv_message, false);
    if (recv_message.contents[0] == CMD_END_PVQ)
      num_traj_send++;
  }
  return bytes_recv;
}

void PVarQ::logMessage(message log_message, bool is_send)
// Logs and displays message
{
  if (is_send)
  {
    cout << "sent: ";
    *log_file << "sent: ";
  }
  else
  {
    cout << "recv: ";
    *log_file << "recv: ";
  }
  for (short i = 0; i < 11; i++)
  {
    cout << log_message.contents[i] << " ";
    *log_file << log_message.contents[i] << " ";
  }
  cout << endl;
  *log_file << endl;
}

void PVarQ::run()
// Enables single non-blocking send or receive depending on communication status and if there are trajectory points left to process
{
  if (num_traj_recv > num_traj_send) // Trajectory points remaining to proceses
  {
    if (ack_received) // Ack has been received, so send
      sendMessage();
    if (!ack_received) // Waiting for ack, so receive
      recvMessage();
  }
}
