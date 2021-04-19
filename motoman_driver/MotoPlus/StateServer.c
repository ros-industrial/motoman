// StateServer.c
//
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Yaskawa America, Inc.
* All rights reserved.
*
* Redistribution and use in binary form, with or without modification,
* is permitted provided that the following conditions are met:
*
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

#include "MotoROS.h"

//-----------------------
// Function Declarations
//-----------------------
void Ros_StateServer_SendState(Controller* controller, int connectionIndex);
BOOL Ros_StateServer_SendMsgToAllClient(Controller* controller, int connectionIndex, SimpleMsg* sendMsg, int msgSize);
void Ros_StateServer_StopConnection(Controller* controller, int connectionIndex);

//-----------------------
// Function implementation
//-----------------------

//-----------------------------------------------------------------------
// Start the task for a new state server connection:
// - Ros_StateServer_SendState: Task that broadcasts controller & robot state to the connected client
//-----------------------------------------------------------------------
void Ros_StateServer_StartNewConnection(Controller* controller, int sd)
{
	int connectionIndex;
	int sockOpt;

	printf("Starting new connection to the State Server\r\n");
	
ATTEMPT_STATE_CONNECTION:
	//look for next available connection slot
	for (connectionIndex = 0; connectionIndex < MAX_STATE_CONNECTIONS; connectionIndex++)
	{
		if (controller->sdStateConnections[connectionIndex] == INVALID_SOCKET)
		{
			controller->sdStateConnections[connectionIndex] = sd;			
			break;
		}
	}
	
	if (connectionIndex == MAX_STATE_CONNECTIONS)
	{
		if (Ros_MotionServer_HasDataInQueue(controller)) //another client is actively controlling motion; likely monitoring the state also
		{
			puts("Too many State server connections... not accepting last attempt.");
			mpClose(sd);
			return;
		}
		else
		{
			puts("Too many State server connections... closing old connection.");
			Ros_StateServer_StopConnection(controller, 0); //close socket, cleanup resources, and delete tasks
			goto ATTEMPT_STATE_CONNECTION;
		}
	}

	sockOpt = 1;
	mpSetsockopt(sd, SOL_SOCKET, SO_KEEPALIVE, (char*)&sockOpt, sizeof(sockOpt));

	//start task that will send the controller state
	controller->tidStateSendState[connectionIndex] = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
																	(FUNCPTR)Ros_StateServer_SendState,
																	(int)controller, connectionIndex, 0, 0, 0, 0, 0, 0, 0, 0);

	//set feedback signal
	if (controller->tidStateSendState[connectionIndex] != INVALID_TASK)
		Ros_Controller_SetIOState(IO_FEEDBACK_STATESERVERCONNECTED, TRUE);
	else
	{
		mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 3);
		Ros_StateServer_StopConnection(controller, connectionIndex);
	}
}

void Ros_StateServer_StopConnection(Controller* controller, int connectionIndex)
{
	int tid, i;
	BOOL bAtLeastOne;

	//close this connection
	mpClose(controller->sdStateConnections[connectionIndex]);
	//mark connection as invalid
	controller->sdStateConnections[connectionIndex] = INVALID_SOCKET;

	// Stop message receiption task
	tid = controller->tidStateSendState[connectionIndex];
	controller->tidStateSendState[connectionIndex] = INVALID_TASK;

	//update I/O bit that indicates if any state connections are active
	bAtLeastOne = FALSE;
	for (i = 0; i < MAX_STATE_CONNECTIONS; i++)
	{
		if (controller->sdStateConnections[i] != INVALID_SOCKET)
			bAtLeastOne = TRUE;
	}
	Ros_Controller_SetIOState(IO_FEEDBACK_STATESERVERCONNECTED, bAtLeastOne);

	mpDeleteTask(tid);
}


//-----------------------------------------------------------------------
// Send state (robot position and controller status) as long as there is
// an active connection
//-----------------------------------------------------------------------
void Ros_StateServer_SendState(Controller* controller, int connectionIndex)
{
	int groupNo;
	SimpleMsg sendMsg;
	SimpleMsg sendMsgFEx;
	int msgSize, fexMsgSize = 0;
	BOOL bOkToSendExFeedback;
	BOOL bSuccesfulSend;
	
	printf("Starting State Server Send State task\r\n");
	printf("Controller number of group = %d\r\n", controller->numGroup);
	
	while(TRUE) //loop will break when there is a transmission error
	{
		Ros_SimpleMsg_JointFeedbackEx_Init(controller->numGroup, &sendMsgFEx);
		bOkToSendExFeedback = TRUE;

		// Send feedback position for each control group
		for(groupNo=0; groupNo < controller->numGroup; groupNo++)
		{
			msgSize = Ros_SimpleMsg_JointFeedback(controller->ctrlGroups[groupNo], &sendMsg);
			fexMsgSize = Ros_SimpleMsg_JointFeedbackEx_Build(groupNo, &sendMsg, &sendMsgFEx);
			if(msgSize > 0)
			{
				bSuccesfulSend = Ros_StateServer_SendMsgToAllClient(controller, connectionIndex, &sendMsg, msgSize);
				if (!bSuccesfulSend)
					break;
			}
			else
			{
				printf("Ros_SimpleMsg_JointFeedback returned a message size of 0\r\n");
				bOkToSendExFeedback = FALSE;
			}
		}

		if (controller->numGroup < 2) //only send the ROS_MSG_MOTO_JOINT_FEEDBACK_EX message if we have multiple control groups
			bOkToSendExFeedback = FALSE;

		if (bOkToSendExFeedback) //send extended-feedback message
		{
			bSuccesfulSend = Ros_StateServer_SendMsgToAllClient(controller, connectionIndex, &sendMsgFEx, fexMsgSize);
			if (!bSuccesfulSend)
				break;
		}

		// Send controller/robot status
		msgSize = Ros_Controller_StatusToMsg(controller, &sendMsg);
		if(msgSize > 0)
		{
			bSuccesfulSend = Ros_StateServer_SendMsgToAllClient(controller, connectionIndex, &sendMsg, msgSize);
			if (!bSuccesfulSend)
				break;
		}
		Ros_Sleep(STATE_UPDATE_MIN_PERIOD);
	}
	
	printf("State Server Send State task was terminated\r\n");
	Ros_StateServer_StopConnection(controller, connectionIndex);
}


//-----------------------------------------------------------------------
// Send state message to all active connections
// return TRUE if message was send to at least one client
//-----------------------------------------------------------------------
BOOL Ros_StateServer_SendMsgToAllClient(Controller* controller, int connectionIndex, SimpleMsg* sendMsg, int msgSize)
{
	int ret;
	
	ret = mpSend(controller->sdStateConnections[connectionIndex], (char*)(sendMsg), msgSize, 0);
	if(ret <= 0)
	{
		printf("StateServer Send failure.  Closing state server connection.\r\n");
		return FALSE;
	}

	return TRUE;
}
