// MotionServer.c
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
#include "debug.h"

//-----------------------
// Function Declarations
//-----------------------
// Main Task: 
void Ros_MotionServer_StartNewConnection(Controller* controller, int sd);
void Ros_MotionServer_StopConnection(Controller* controller, int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_MotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex);
BOOL Ros_MotionServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_MotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
BOOL Ros_MotionServer_StopMotion(Controller* controller);
BOOL Ros_MotionServer_ServoPower(Controller* controller, int servoOnOff);
BOOL Ros_MotionServer_ResetAlarm(Controller* controller);
BOOL Ros_MotionServer_StartTrajMode(Controller* controller);
BOOL Ros_MotionServer_StopTrajMode(Controller* controller);
int Ros_MotionServer_JointTrajDataProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_MotionServer_InitTrajPointFull(CtrlGroup* ctrlGroup, SmBodyJointTrajPtFull* jointTrajData);
int Ros_MotionServer_InitTrajPointFullEx(CtrlGroup* ctrlGroup, SmBodyJointTrajPtExData* jointTrajDataEx, int sequence);
int Ros_MotionServer_AddTrajPointFull(CtrlGroup* ctrlGroup, SmBodyJointTrajPtFull* jointTrajData);
int Ros_MotionServer_AddTrajPointFullEx(CtrlGroup* ctrlGroup, SmBodyJointTrajPtExData* jointTrajDataEx, int sequence);
int Ros_MotionServer_JointTrajPtFullExProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_MotionServer_GetDhParameters(Controller* controller, SimpleMsg* replyMsg);
int Ros_MotionServer_SetSelectedTool(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
void Ros_MotionServer_EnsureEcoModeIsDisabled(Controller* controller);

// AddToIncQueue Task:
void Ros_MotionServer_AddToIncQueueProcess(Controller* controller, int groupNo);
void Ros_MotionServer_JointTrajDataToIncQueue(Controller* controller, int groupNo);
BOOL Ros_MotionServer_AddPulseIncPointToQ(Controller* controller, int groupNo, Incremental_data* dataToEnQ);
BOOL Ros_MotionServer_ClearQ_All(Controller* controller);
BOOL Ros_MotionServer_HasDataInQueue(Controller* controller);
int Ros_MotionServer_GetQueueCnt(Controller* controller, int groupNo);
void Ros_MotionServer_IncMoveLoopStart(Controller* controller);

// Utility functions:
void Ros_MotionServer_ConvertToJointMotionData(SmBodyJointTrajPtFull* jointTrajData, JointMotionData* jointMotionData);
STATUS Ros_MotionServer_DisableEcoMode(Controller* controller);
void Ros_MotionServer_PrintError(USHORT err_no, char* msgPrefix);

//-----------------------
// Function implementation
//-----------------------

//-----------------------------------------------------------------------
// Start the tasks for a new motion server connection:
// - WaitForSimpleMsg: Task that waits to receive new SimpleMessage
// - AddToIncQueueProcess: Task that take data from a message and generate Incmove  
//-----------------------------------------------------------------------
void Ros_MotionServer_StartNewConnection(Controller* controller, int sd)
{
	int groupNo;
	int connectionIndex;
	int sockOpt;
	
	printf("Starting new connection to the Motion Server\r\n");

ATTEMPT_MOTION_CONNECTION:
	//look for next available connection slot
	for (connectionIndex = 0; connectionIndex < MAX_MOTION_CONNECTIONS; connectionIndex++)
	{
		if (controller->sdMotionConnections[connectionIndex] == INVALID_SOCKET)
		{
			controller->sdMotionConnections[connectionIndex] = sd;
			break;
		}
	}
	
	if (connectionIndex == MAX_MOTION_CONNECTIONS)
	{
		if (Ros_MotionServer_HasDataInQueue(controller)) //another client is actively controlling motion
		{
			puts("Motion server already connected... not accepting last attempt.");
			mpClose(sd);
			return;
		}
		else
		{
			puts("Motion server already connected... closing old connection.");
			Ros_MotionServer_StopConnection(controller, 0); //close socket, cleanup resources, and delete tasks
			goto ATTEMPT_MOTION_CONNECTION; //goto is sometimes useful... don't judge me
		}
	}

	//This timeout detection takes two hours. So, it's not terribly useful. But, it still serves a purpose.
	sockOpt = 1;
	mpSetsockopt(sd, SOL_SOCKET, SO_KEEPALIVE, (char*)&sockOpt, sizeof(sockOpt));
	
	// If not started, start the IncMoveTask (there should be only one instance of this thread)
	if(controller->tidIncMoveThread == INVALID_TASK)
	{
		puts("Creating new task: IncMoveTask");
		
		controller->tidIncMoveThread = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE, 
													(FUNCPTR)Ros_MotionServer_IncMoveLoopStart,
													(int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		if (controller->tidIncMoveThread == ERROR)
		{
			puts("Failed to create task for incremental-motion.  Check robot parameters.");
			mpClose(sd);
			controller->tidIncMoveThread = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);

			return;
		}
	}
	
	// If not started, start the AddToIncQueueProcess for each control group
	for(groupNo = 0; groupNo < controller->numGroup; groupNo++)
	{
		if (controller->ctrlGroups[groupNo]->tidAddToIncQueue == INVALID_TASK)
		{
			printf("Creating new task: tidAddToIncQueue (groupNo = %d)\n", groupNo);
			
			controller->ctrlGroups[groupNo]->tidAddToIncQueue = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
																			(FUNCPTR)Ros_MotionServer_AddToIncQueueProcess,
																			(int)controller, groupNo, 0, 0, 0, 0, 0, 0, 0, 0); 
			if (controller->ctrlGroups[groupNo]->tidAddToIncQueue == ERROR)
			{
				puts("Failed to create task for parsing motion increments.  Check robot parameters.");
				mpClose(sd);
				controller->ctrlGroups[groupNo]->tidAddToIncQueue = INVALID_TASK;
				Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
				mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 5);
				return;
			}
		}
	}
	

	if (controller->tidMotionConnections[connectionIndex] == INVALID_TASK)
	{
		printf("Creating new task: tidMotionConnections (connectionIndex = %d)\n", connectionIndex);
		
			
		//start new task for this specific connection
		controller->tidMotionConnections[connectionIndex] = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
																		(FUNCPTR)Ros_MotionServer_WaitForSimpleMsg,
																		(int)controller, connectionIndex, 0, 0, 0, 0, 0, 0, 0, 0);
	
		if (controller->tidMotionConnections[connectionIndex] != ERROR)
		{
			Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, TRUE); //set feedback signal indicating success
		}
		else
		{
			puts("Could not create new task in the motion server.  Check robot parameters.");
			mpClose(sd);
			controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;
			controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 6);
			return;
		}
	}
}


//-----------------------------------------------------------------------
// Close a connection along with all its associated task
//-----------------------------------------------------------------------
void Ros_MotionServer_StopConnection(Controller* controller, int connectionIndex)
{   
	int i;
	int tid;
	BOOL bDeleteIncMovTask;
	
	printf("Closing Motion Server Connection\r\n");
	
	//close this connection
	mpClose(controller->sdMotionConnections[connectionIndex]);
	//mark connection as invalid
	controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;

	// Check if there are still some valid connection
	bDeleteIncMovTask = TRUE;
	for(i=0; i<MAX_MOTION_CONNECTIONS; i++)
	{
		if(controller->sdMotionConnections[connectionIndex] != INVALID_SOCKET)
		{
			bDeleteIncMovTask = FALSE;
			break;
		}
	}
	
	// If there is no more connection, stop the inc_move task
	if(bDeleteIncMovTask)
	{
		//set feedback signal
		Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, FALSE);

		// Stop adding increment to queue (for each ctrlGroup
		for(i=0; i < controller->numGroup; i++)
		{
			controller->ctrlGroups[i]->hasDataToProcess = FALSE;
			tid = controller->ctrlGroups[i]->tidAddToIncQueue;
			controller->ctrlGroups[i]->tidAddToIncQueue = INVALID_TASK;
			mpDeleteTask(tid);
		}
		
		// terminate the inc_move task
		tid = controller->tidIncMoveThread;
		controller->tidIncMoveThread = INVALID_TASK;
		mpDeleteTask(tid);
	}
		
	// Stop message receiption task
	tid = controller->tidMotionConnections[connectionIndex];
	controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
	printf("Motion Server Connection Closed\r\n");
	
	mpDeleteTask(tid);
}

int Ros_MotionServer_GetExpectedByteSizeForMessageType(SimpleMsg* receiveMsg, int recvByteSize)
{
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;

	switch (receiveMsg->header.msgType)
	{
	case ROS_MSG_PING:
		expectedSize = minSize;
		break;
	case ROS_MSG_ROBOT_STATUS:
		expectedSize = minSize + sizeof(SmBodyRobotStatus);
		break;
	case ROS_MSG_JOINT_TRAJ_PT_FULL:
		expectedSize = minSize + sizeof(SmBodyJointTrajPtFull);
		break;
	case ROS_MSG_JOINT_FEEDBACK:
		expectedSize = minSize + sizeof(SmBodyJointFeedback);
		break;
	case ROS_MSG_MOTO_MOTION_CTRL:
		expectedSize = minSize + sizeof(SmBodyMotoMotionCtrl);
		break;
	case ROS_MSG_MOTO_MOTION_REPLY:
		expectedSize = minSize + sizeof(SmBodyMotoMotionReply);
		break;
	case ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX:
		//Don't require the user to send data for non-existant control groups
		if (recvByteSize >= (int)(minSize + sizeof(int))) //make sure I can at least get to [numberOfGroups] field
		{
			expectedSize = minSize + (sizeof(int) * 2);
			expectedSize += (sizeof(SmBodyJointTrajPtExData) * receiveMsg->body.jointTrajDataEx.numberOfValidGroups); //check the number of groups to determine size of data
		}
		else
			expectedSize = minSize + sizeof(SmBodyJointTrajPtFullEx);
		break;
	case ROS_MSG_MOTO_JOINT_FEEDBACK_EX:
		expectedSize = minSize + sizeof(SmBodyJointFeedbackEx);
		break;
	case ROS_MSG_MOTO_READ_IO_BIT:
		expectedSize = minSize + sizeof(SmBodyMotoReadIOBit);
		break;
	case ROS_MSG_MOTO_WRITE_IO_BIT:
		expectedSize = minSize + sizeof(SmBodyMotoWriteIOBit);
		break;
	case ROS_MSG_MOTO_READ_IO_GROUP:
		expectedSize = minSize + sizeof(SmBodyMotoReadIOGroup);
		break;
	case ROS_MSG_MOTO_WRITE_IO_GROUP:
		expectedSize = minSize + sizeof(SmBodyMotoWriteIOGroup);
		break;
	case ROS_MSG_MOTO_GET_DH_PARAMETERS:
		expectedSize = minSize; //no additional data on the request
		break;
	case ROS_MSG_MOTO_SELECT_TOOL:
		expectedSize = minSize + sizeof(SmBodySelectTool);
		break;
	default: //invalid message type
		return -1;
	}
	return expectedSize;
}

//-----------------------------------------------------------------------
// Task that waits to receive new SimpleMessage and then processes it
//-----------------------------------------------------------------------
void Ros_MotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex)
{
	SimpleMsg receiveMsg;
	SimpleMsg replyMsg;
	int byteSize = 0, byteSizeResponse = 0;
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;
	int ret = 0;
	int partialMsgByteCount = 0;
	BOOL bSkipNetworkRecv = FALSE;

	while (TRUE) //keep accepting messages until connection closes
	{
		Ros_Sleep(0);	//give it some time to breathe, if needed

		if (!bSkipNetworkRecv) //if I don't already have an extra complete packet buffered from the previous recv
		{
			//Receive message from the PC
			memset((&receiveMsg) + partialMsgByteCount, 0x00, sizeof(SimpleMsg) - partialMsgByteCount);
			byteSize = mpRecv(controller->sdMotionConnections[connectionIndex], (char*)((&receiveMsg) + partialMsgByteCount), sizeof(SimpleMsg) - partialMsgByteCount, 0);
			if (byteSize <= 0)
				break; //end connection

			byteSize += partialMsgByteCount;
			partialMsgByteCount = 0;
		}
		else
		{
			byteSize = partialMsgByteCount;
			partialMsgByteCount = 0;
			bSkipNetworkRecv = FALSE;
		}

		// Determine the expected size of the message
		expectedSize = -1;
		if(byteSize >= minSize)
		{
			expectedSize = Ros_MotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, byteSize);

			if (expectedSize == -1)
			{
				printf("Unknown Message Received (%d)\r\n", receiveMsg.header.msgType);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGTYPE, &replyMsg, 0);
			}			
			else if (byteSize >= expectedSize) // Check message size
			{
				// Process the simple message
				ret = Ros_MotionServer_SimpleMsgProcess(controller, &receiveMsg, &replyMsg);
				if (ret != OK) //error during processing
				{
					break; //disconnect
				}
				else if (byteSize > expectedSize) // Received extra data in single message
				{
					//Special case where ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX message could have different lengths
					if (receiveMsg.header.msgType == ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX &&
						byteSize == (int)(minSize + sizeof(SmBodyJointTrajPtFullEx)))
					{
						// All good
						partialMsgByteCount = 0;
					}
					else
					{
						// Preserve the remaining bytes and treat them as the start of a new message
						Db_Print("MessageReceived(%d bytes): expectedSize=%d, processing rest of bytes (%d, %d, %d)\r\n", byteSize, expectedSize, sizeof(receiveMsg), receiveMsg.body.jointTrajData.sequence, ((int*)((char*)&receiveMsg + expectedSize))[5]);
						partialMsgByteCount = byteSize - expectedSize;
						memmove(&receiveMsg, (char*)&receiveMsg + expectedSize, partialMsgByteCount);

						//Did I receive multiple full messages at once that all need to be processed before listening for new data?
						if (partialMsgByteCount >= minSize)
						{
							expectedSize = Ros_MotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, partialMsgByteCount);
							bSkipNetworkRecv = (partialMsgByteCount >= expectedSize); //does my modified receiveMsg buffer contain a full message to process?
						}
					}
				}
				else // All good
					partialMsgByteCount = 0;
			}
			else // Not enough data to process the command
			{
				Db_Print("MessageReceived(%d bytes): expectedSize=%d\r\n", byteSize, expectedSize);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
			}
		}
		else // Didn't even receive a command ID
		{
			Db_Print("Unknown Data Received (%d bytes)\r\n", byteSize);
			Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
		}

		if (receiveMsg.header.commType != ROS_COMM_SERVICE_REPLY) //don't send a reply to a reply from the pc
		{
			//Send reply message
			byteSizeResponse = mpSend(controller->sdMotionConnections[connectionIndex], (char*)(&replyMsg), replyMsg.prefix.length + sizeof(SmPrefix), 0);
			if (byteSizeResponse <= 0)
				break;	// Close the connection
		}
	}
	
	Ros_Sleep(50);	// Just in case other associated task need time to clean-up.
	
	//close this connection
	Ros_MotionServer_StopConnection(controller, connectionIndex);
}


//-----------------------------------------------------------------------
// Checks the type of message and processes it accordingly
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_MotionServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int ret = ERROR;
	int invalidSubcode = 0;
	
	switch(receiveMsg->header.msgType)
	{
	case ROS_MSG_PING:
		memset(replyMsg, 0x00, sizeof(SimpleMsg));

		replyMsg->prefix.length = sizeof(SmHeader);
		replyMsg->header.msgType = ROS_MSG_PING;
		replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
		replyMsg->header.replyType = ROS_REPLY_SUCCESS;
		ret = OK;
		break;

	case ROS_MSG_JOINT_TRAJ_PT_FULL:
		ret = Ros_MotionServer_JointTrajDataProcess(controller, receiveMsg, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_MOTION_CTRL:
		ret = Ros_MotionServer_MotionCtrlProcess(controller, receiveMsg, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX:
		ret = Ros_MotionServer_JointTrajPtFullExProcess(controller, receiveMsg, replyMsg);
		break;


//Maintain backward compatibility for users who are sending I/O over motion-server
	//-----------------------
	case ROS_MSG_MOTO_READ_IO_BIT:
		ret = Ros_IoServer_ReadIOBit(receiveMsg, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_WRITE_IO_BIT:
		ret = Ros_IoServer_WriteIOBit(receiveMsg, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_READ_IO_GROUP:
		ret = Ros_IoServer_ReadIOGroup(receiveMsg, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_WRITE_IO_GROUP:
		ret = Ros_IoServer_WriteIOGroup(receiveMsg, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_GET_DH_PARAMETERS:
		ret = Ros_MotionServer_GetDhParameters(controller, replyMsg);
		break;

	//-----------------------
	case ROS_MSG_MOTO_SELECT_TOOL:
		ret = Ros_MotionServer_SetSelectedTool(controller, receiveMsg, replyMsg);
		break;

	//-----------------------
	default:
		printf("Invalid message type: %d\n", receiveMsg->header.msgType);
		invalidSubcode = ROS_RESULT_INVALID_MSGTYPE;
		break;
	}
	
	// Check Invalid Case
	if(invalidSubcode != 0)
	{
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, invalidSubcode, replyMsg, 0);
		ret = ERROR;
	}
		
	return ret;
}


//-----------------------------------------------------------------------
// Processes message of type: ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_MotionServer_JointTrajPtFullExProcess(Controller* controller, SimpleMsg* receiveMsg, 
											  SimpleMsg* replyMsg)
{
	SmBodyJointTrajPtFullEx* msgBody;	
	CtrlGroup* ctrlGroup;
	int ret, i;
	FlagsValidFields validationFlags;

	msgBody = &receiveMsg->body.jointTrajDataEx;

	// Check if controller is able to receive incremental move and if the incremental move thread is running
	if(!Ros_Controller_IsMotionReady(controller))
	{
		int subcode = Ros_Controller_GetNotReadySubcode(controller);
		printf("ERROR: Controller is not ready (code: %d).  Can't process ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX.\r\n", subcode);
		for (i = 0; i < msgBody->numberOfValidGroups; i += 1)
		{
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_NOT_READY, subcode, replyMsg, msgBody->jointTrajPtData[i].groupNo);
		}
		return 0;
	}

	// Pre-check to ensure no groups are busy
	for (i = 0; i < msgBody->numberOfValidGroups; i += 1)
	{
		if (Ros_Controller_IsValidGroupNo(controller, msgBody->jointTrajPtData[i].groupNo))
		{
			ctrlGroup = controller->ctrlGroups[msgBody->jointTrajPtData[i].groupNo];
			if (ctrlGroup->hasDataToProcess)
			{
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_BUSY, 0, replyMsg, msgBody->jointTrajPtData[i].groupNo);
				return 0;
			}
		}
		else
		{
			printf("ERROR: GroupNo %d is not valid\n", msgBody->jointTrajPtData[i].groupNo);
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_GROUPNO, replyMsg, msgBody->jointTrajPtData[i].groupNo);
			return 0;
		}
			
		// Check that minimum information (time, position, velocity) is valid
		validationFlags = Valid_Time | Valid_Position | Valid_Velocity;
		if( (msgBody->jointTrajPtData[i].validFields & validationFlags) != validationFlags)
		{
			printf("ERROR: Validfields = %d\r\n", msgBody->jointTrajPtData[i].validFields);
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_DATA_INSUFFICIENT, replyMsg, msgBody->jointTrajPtData[i].groupNo);
			return 0;
		}
	}

	for (i = 0; i < msgBody->numberOfValidGroups; i += 1)
	{
		ctrlGroup = controller->ctrlGroups[msgBody->jointTrajPtData[i].groupNo];
		
		// Check the trajectory sequence code
		if(msgBody->sequence == 0) // First trajectory point
		{
            //It's possible for energy-saving mode to activate between /robot_enable and the first trajectory point.
            Ros_MotionServer_EnsureEcoModeIsDisabled(controller); //make sure that Ros_Controller_IsMotionReady gets called above before calling this

			// Initialize first point variables
			ret = Ros_MotionServer_InitTrajPointFullEx(ctrlGroup, &msgBody->jointTrajPtData[i], msgBody->sequence);
		
			// set reply
			if(ret == 0)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, msgBody->jointTrajPtData[i].groupNo);
			else
			{
				printf("ERROR: Ros_MotionServer_InitTrajPointFullEx returned %d\n", ret);
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ret, replyMsg, msgBody->jointTrajPtData[i].groupNo);
				return 0; //stop processing other groups in this loop
			}
		}
		else if(msgBody->sequence > 0)// Subsequent trajectory points
		{
			// Add the point to the trajectory
			ret = Ros_MotionServer_AddTrajPointFullEx(ctrlGroup, &msgBody->jointTrajPtData[i], msgBody->sequence);
		
			// ser reply
			if(ret == 0)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, msgBody->jointTrajPtData[i].groupNo);
			else if(ret == 1)
			{
				printf("ERROR: Ros_MotionServer_AddTrajPointFullEx returned %d\n", ret);
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_BUSY, 0, replyMsg, msgBody->jointTrajPtData[i].groupNo);
				return 0; //stop processing other groups in this loop
			}
			else
			{
				printf("ERROR: Ros_MotionServer_AddTrajPointFullEx returned %d\n", ret);
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ret, replyMsg, msgBody->jointTrajPtData[i].groupNo);
				return 0; //stop processing other groups in this loop
			}
		}
		else
		{
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_SEQUENCE, replyMsg, msgBody->jointTrajPtData[i].groupNo);
			return 0; //stop processing other groups in this loop
		}
	}

	return 0;
}


//-----------------------------------------------------------------------
// Processes message of type: ROS_MSG_MOTO_MOTION_CTRL
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_MotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, 
										SimpleMsg* replyMsg)
{
	SmBodyMotoMotionCtrl* motionCtrl;

	//printf("In MotionCtrlProcess\r\n");

	// Check the command code
	motionCtrl = &receiveMsg->body.motionCtrl;
	switch(motionCtrl->command)
	{
		case ROS_CMD_CHECK_MOTION_READY: 
		{
			if(Ros_Controller_IsMotionReady(controller))
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_TRUE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FALSE, Ros_Controller_GetNotReadySubcode(controller), replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_CHECK_QUEUE_CNT:
		{
			int count = Ros_MotionServer_GetQueueCnt(controller, motionCtrl->groupNo);
			if(count >= 0)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_TRUE, count, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, count, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_STOP_MOTION:
		{
			// Stop Motion
			BOOL bRet = Ros_MotionServer_StopMotion(controller);
			
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else 
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_START_SERVOS:
		{
			// Stop Motion
			BOOL bRet = Ros_MotionServer_ServoPower(controller, ON);
			
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else 
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_STOP_SERVOS:
		{
			// Stop Motion
			BOOL bRet = Ros_MotionServer_ServoPower(controller, OFF);
			
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else 
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_RESET_ALARM:
		{
			// Stop Motion
			BOOL bRet = Ros_MotionServer_ResetAlarm(controller);
			
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else 
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_START_TRAJ_MODE:
		{
			// Start Trajectory mode by starting the INIT_ROS job on the controller
			BOOL bRet = Ros_MotionServer_StartTrajMode(controller);
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_NOT_READY, 
						Ros_Controller_GetNotReadySubcode(controller), replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_STOP_TRAJ_MODE:
		case ROS_CMD_DISCONNECT:
		{
			BOOL bRet = Ros_MotionServer_StopTrajMode(controller);
			if(bRet)
			{
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
				if(motionCtrl->command == ROS_CMD_DISCONNECT)
					return 1;
			}
			else
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
	}

	return 0;
}


//-----------------------------------------------------------------------
// Stop motion by stopping message processing and clearing the queue
//-----------------------------------------------------------------------
BOOL Ros_MotionServer_StopMotion(Controller* controller)
{
	// NOTE: for the time being, stop motion will stop all motion for all control group 
	BOOL bRet;
	BOOL bStopped;
	int checkCnt;
	int groupNo;
		
	// Stop any motion from being processed further
	controller->bStopMotion = TRUE;
	
	// Check that background processing of message has been stopped
	for(checkCnt=0; checkCnt<MOTION_STOP_TIMEOUT; checkCnt++) 
	{
		bStopped = TRUE;
		for(groupNo=0; groupNo<controller->numGroup; groupNo++)
			bStopped &= !controller->ctrlGroups[groupNo]->hasDataToProcess;
		if(bStopped)
			break;
		else
			Ros_Sleep(1);
	}
	
	// Clear queues
	bRet = Ros_MotionServer_ClearQ_All(controller);
	
	// All motion should be stopped at this point, so turn of the flag
	controller->bStopMotion = FALSE;
	
	if (checkCnt >= MOTION_STOP_TIMEOUT)
		printf("WARNING: Message processing not stopped before clearing queue\r\n");

	return(bStopped && bRet);
}


//-----------------------------------------------------------------------
// Sets servo power to ON or OFF
//-----------------------------------------------------------------------
BOOL Ros_MotionServer_ServoPower(Controller* controller, int servoOnOff)
{
	MP_SERVO_POWER_SEND_DATA sServoData;
	MP_STD_RSP_DATA stdRespData;
	int ret;
	STATUS status;
	
#ifdef DUMMY_SERVO_MODE
	return TRUE;
#endif
	
	if (servoOnOff == OFF)
		Ros_MotionServer_StopMotion(controller);

	if (servoOnOff == ON)
	{
		status = Ros_MotionServer_DisableEcoMode(controller);
		if (status == NG)
		{
			Ros_Controller_StatusUpdate(controller);
			return Ros_Controller_IsServoOn(controller) == servoOnOff;
		}
	}

	printf("Setting servo power: %d\n", servoOnOff);
	memset(&sServoData, 0x00, sizeof(sServoData));
	memset(&stdRespData, 0x00, sizeof(stdRespData));
	sServoData.sServoPower = servoOnOff;

	ret = mpSetServoPower(&sServoData, &stdRespData);
	if( (ret == 0) && (stdRespData.err_no == 0) )
	{
		// wait for confirmation
		int checkCount;
		for(checkCount=0; checkCount<MOTION_START_TIMEOUT; checkCount+=MOTION_START_CHECK_PERIOD)
		{
			// Update status
			Ros_Controller_StatusUpdate(controller);
		
			if(Ros_Controller_IsServoOn(controller) == servoOnOff)
				break;
			
			Ros_Sleep(MOTION_START_CHECK_PERIOD);
		}		
	}
	else
	{
		Ros_MotionServer_PrintError(stdRespData.err_no, "Can't turn off servo because:");
	}
	
	// Update status
	Ros_Controller_StatusUpdate(controller);
	return Ros_Controller_IsServoOn(controller) == servoOnOff;
}

BOOL Ros_MotionServer_ResetAlarm(Controller* controller)
{
	int ret, i;
	BOOL returnBoolean;
	MP_ALARM_STATUS_RSP_DATA alarmstatus;
	MP_STD_RSP_DATA responseData;
	
	returnBoolean = TRUE;

	ret = mpGetAlarmStatus(&alarmstatus);
	if( ret != 0 ) 
	{
		printf("Could not get alarm status\n");
		//Ignore this error.  Continue to try and clear the alarm.
	}
    
	if (alarmstatus.sIsAlarm & MASK_ISALARM_ACTIVEALARM) //alarm is active
	{
		MP_ALARM_CODE_RSP_DATA alarmcode;
		ret = mpGetAlarmCode(&alarmcode);
		if( ret != 0 ) 
		{
			printf("Could not get alarm code\n");
			//Ignore this error.  Continue to try and clear the alarm.
		}
		else
		{
			for (i = 0; i < alarmcode.usAlarmNum; i += 1)
				printf("Has alarm: %d[%d], resetting...\n", alarmcode.AlarmData.usAlarmNo[i], alarmcode.AlarmData.usAlarmData[i]);
		}

		ret = mpResetAlarm(&responseData);
		if( ret != 0 ) 
		{
			printf("Could not reset the alarm, failure code: %d\n", responseData.err_no);
			returnBoolean = FALSE;
		}
	}

	if (alarmstatus.sIsAlarm & MASK_ISALARM_ACTIVEERROR) //error is active
	{
		MP_ALARM_CODE_RSP_DATA alarmcode;
		ret = mpGetAlarmCode(&alarmcode);
		if (ret != 0)
		{
			printf("Could not get error code\n");
			//Ignore this problem.  Continue to try and clear the error.
		}
		else
		{
			printf("Has error: %d[%d], resetting...\n", alarmcode.usErrorNo, alarmcode.usErrorData);
		}

		ret = mpCancelError(&responseData);
		if (ret != 0)
		{
			printf("Could not cancel the error, failure code: %d\n", responseData.err_no);
			returnBoolean = FALSE;
		}
	}

	Ros_Controller_StatusUpdate(controller);
	return returnBoolean;
}

//-----------------------------------------------------------------------
// Attempts to start playback of a job to put the controller in RosMotion mode
//-----------------------------------------------------------------------
BOOL Ros_MotionServer_StartTrajMode(Controller* controller)
{
	int ret;
	MP_STD_RSP_DATA rData;
	MP_START_JOB_SEND_DATA sStartData;
	int checkCount;
	int grpNo;
	STATUS status;

	printf("In StartTrajMode\r\n");

	// Update status
	Ros_Controller_StatusUpdate(controller);

	// Reset PFL Activation Flag
	if(controller->bPFLduringRosMove)
		controller->bPFLduringRosMove = FALSE;

	// Reset Inc Move Error
	if (controller->bMpIncMoveError)
		controller->bMpIncMoveError = FALSE;

	// Check if already in the proper mode
	if(Ros_Controller_IsMotionReady(controller))
		return TRUE;

	// Check if currently in operation, we don't want to interrupt current operation
	if(Ros_Controller_IsOperating(controller))
		return FALSE;
		
#ifndef DUMMY_SERVO_MODE
	// Check for condition that need operator manual intervention	
	if(Ros_Controller_IsEStop(controller)
		|| Ros_Controller_IsHold(controller)
		|| !Ros_Controller_IsRemote(controller))
		return FALSE;
#endif
		
	// Check for condition that can be fixed remotely
	if(Ros_Controller_IsError(controller))
	{
		// Cancel error
		memset(&rData, 0x00, sizeof(rData));
		ret = mpCancelError(&rData);
		if(ret != 0)
			goto updateStatus;
	}

	// Check for condition that can be fixed remotely
	if(Ros_Controller_IsAlarm(controller))
	{
		// Reset alarm
		memset(&rData, 0x00, sizeof(rData));
		ret = mpResetAlarm(&rData);
		if(ret == 0)
		{
			// wait for the Alarm reset confirmation
			int checkCount;
			for(checkCount=0; checkCount<MOTION_START_TIMEOUT; checkCount+=MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);
		
				if(Ros_Controller_IsAlarm(controller) == FALSE)
					continue;
			
				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
			if(Ros_Controller_IsAlarm(controller))
				goto updateStatus;
		}
		else
			goto updateStatus;
	}
	

#ifndef DUMMY_SERVO_MODE
	// Servo On
	if(Ros_Controller_IsServoOn(controller) == FALSE)
	{
		MP_SERVO_POWER_SEND_DATA sServoData;
		memset(&sServoData, 0x00, sizeof(sServoData));

		status = Ros_MotionServer_DisableEcoMode(controller);
		if (status == NG)
		{
			goto updateStatus;
		}

		sServoData.sServoPower = 1;  // ON
		memset(&rData, 0x00, sizeof(rData));
		ret = mpSetServoPower(&sServoData, &rData);
		if( (ret == 0) && (rData.err_no ==0) )
		{
			// wait for the Servo On confirmation
			int checkCount;
			for(checkCount=0; checkCount<MOTION_START_TIMEOUT; checkCount+=MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);
		
				if (Ros_Controller_IsServoOn(controller) == TRUE)
					break;
			
				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
			if(Ros_Controller_IsServoOn(controller) == FALSE)
				goto updateStatus;			
		}
		else
		{
			Ros_MotionServer_PrintError(rData.err_no, "Can't turn on servo because:");
			goto updateStatus;			
		}
	}
#endif
	// make sure that there is no data in the queues
	if (Ros_MotionServer_HasDataInQueue(controller)) {
#ifdef DEBUG
		printf("StartTrajMode clearing leftover data in queue\r\n");
#endif
		Ros_MotionServer_ClearQ_All(controller);

		if (Ros_MotionServer_HasDataInQueue(controller))
			printf("WARNING: StartTrajMode has data in queue\r\n");
	}

	// have to initialize the prevPulsePos that will be used when interpolating the traj
	for(grpNo = 0; grpNo < MP_GRP_NUM; ++grpNo)
	{
		if(controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
		}
	}

	// Start Job
	memset(&rData, 0x00, sizeof(rData));
	memset(&sStartData, 0x00, sizeof(sStartData));
	sStartData.sTaskNo = 0;
	memcpy(sStartData.cJobName, MOTION_INIT_ROS_JOB, MAX_JOB_NAME_LEN);
	ret = mpStartJob(&sStartData, &rData);
	if( (ret != 0) || (rData.err_no !=0) )
	{
		Ros_MotionServer_PrintError(rData.err_no, "Can't start job because:");
		goto updateStatus;		
	}
	
	// wait for the Motion Ready
	for(checkCount=0; checkCount<MOTION_START_TIMEOUT; checkCount+=MOTION_START_CHECK_PERIOD)
	{
		// Update status
		Ros_Controller_StatusUpdate(controller);
		
		if(Ros_Controller_IsMotionReady(controller))
			return(TRUE);
			
		Ros_Sleep(MOTION_START_CHECK_PERIOD);
	}
	
updateStatus:	
	// Update status
	Ros_Controller_StatusUpdate(controller);
	
	return (Ros_Controller_IsMotionReady(controller));
}



//-----------------------------------------------------------------------
// Set I/O signal matching the WAIT instruction to allow the controller 
// to resume job execution
//-----------------------------------------------------------------------
BOOL Ros_MotionServer_StopTrajMode(Controller* controller)
{
	// Don't change mode if queue is not empty
	if(Ros_MotionServer_HasDataInQueue(controller))
	{
		//printf("Failed: Ros_MotionServer_HasDataInQueue is true\r\n");
		return FALSE;
	}
		
	// Stop motion
	if(!Ros_MotionServer_StopMotion(controller))
	{
		//printf("Failed: Ros_MotionServer_StopMotion is false\r\n");
		return FALSE;
	}
	
	// Set I/O signal
	Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, TRUE);
	
	return TRUE;
}


//-----------------------------------------------------------------------
// Processes message of type: ROS_MSG_JOINT_TRAJ_PT_FULL
// Return: 0=Success; -1=Failure
//-----------------------------------------------------------------------
int Ros_MotionServer_JointTrajDataProcess(Controller* controller, SimpleMsg* receiveMsg, 
											SimpleMsg* replyMsg)
{
	SmBodyJointTrajPtFull* trajData;
	CtrlGroup* ctrlGroup;
	int ret;
	FlagsValidFields validationFlags;

	// Check if controller is able to receive incremental move and if the incremental move thread is running
	if(!Ros_Controller_IsMotionReady(controller))
	{
		int subcode = Ros_Controller_GetNotReadySubcode(controller);
		printf("ERROR: Controller is not ready (code: %d).  Can't process ROS_MSG_JOINT_TRAJ_PT_FULL.\r\n", subcode);
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_NOT_READY, subcode, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		return 0;
	}

	// Set pointer reference
	trajData = &receiveMsg->body.jointTrajData;
	
	// Check group number valid
	if(Ros_Controller_IsValidGroupNo(controller, trajData->groupNo))
	{
		ctrlGroup = controller->ctrlGroups[trajData->groupNo];
	}
	else
	{
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_GROUPNO, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		return 0;
	}
	
	// Check that minimum information (time, position, velocity) is valid
	validationFlags = Valid_Time | Valid_Position | Valid_Velocity;
	if( (trajData->validFields & validationFlags) != validationFlags)
	{
		printf("ERROR: Validfields = %d\r\n", trajData->validFields);
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_DATA_INSUFFICIENT, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		return 0;
	}

	//Due to data loss when converting from double (ROS PC) to float (Simple Message serialization), we cannot accept
	//a single trjectory that lasts more than 4 hours.
	if (trajData->time >= MAX_TRAJECTORY_TIME_LENGTH)
	{
		printf("ERROR: Trajectory time (%.4f) > MAX_TRAJECTORY_TIME_LENGTH (%.4f)\r\n", trajData->time, MAX_TRAJECTORY_TIME_LENGTH);
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_DATA_TIME, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		return 0;
	}

	// Check the trajectory sequence code
	if(trajData->sequence == 0) // First trajectory point
	{
        //It's possible for energy-saving mode to activate between /robot_enable and the first trajectory point.
        Ros_MotionServer_EnsureEcoModeIsDisabled(controller); //make sure that Ros_Controller_IsMotionReady gets called above before calling this

		// Initialize first point variables
		ret = Ros_MotionServer_InitTrajPointFull(ctrlGroup, trajData);
		
		// set reply
		if(ret == 0)
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		else
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ret, replyMsg, receiveMsg->body.jointTrajData.groupNo);
	}
	else if(trajData->sequence > 0)// Subsequent trajectory points
	{
		// Add the point to the trajectory
		ret = Ros_MotionServer_AddTrajPointFull(ctrlGroup, trajData);
		
		// ser reply
		if(ret == 0)
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		else if(ret == 1)
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_BUSY, 0, replyMsg, receiveMsg->body.jointTrajData.groupNo);
		else
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ret, replyMsg, receiveMsg->body.jointTrajData.groupNo);	
	}
	else
	{
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_SEQUENCE, replyMsg, receiveMsg->body.jointTrajData.groupNo);
	}

	return 0;
}

//-----------------------------------------------------------------------
// Convert SmBodyMotoJointTrajPtExData data to SmBodyJointTrajPtFull
//-----------------------------------------------------------------------
int Ros_MotionServer_InitTrajPointFullEx(CtrlGroup* ctrlGroup, SmBodyJointTrajPtExData* jointTrajDataEx, int sequence)
{
	SmBodyJointTrajPtFull jointTrajData;

	//convert SmBodyMotoJointTrajPtExData data to SmBodyJointTrajPtFull
	jointTrajData.groupNo = jointTrajDataEx->groupNo;
	jointTrajData.sequence = sequence;
	jointTrajData.validFields = jointTrajDataEx->validFields;
	jointTrajData.time = jointTrajDataEx->time;
	memcpy(jointTrajData.pos, jointTrajDataEx->pos, sizeof(float)*ROS_MAX_JOINT);
	memcpy(jointTrajData.vel, jointTrajDataEx->vel, sizeof(float)*ROS_MAX_JOINT);
	memcpy(jointTrajData.acc, jointTrajDataEx->acc, sizeof(float)*ROS_MAX_JOINT);

	return Ros_MotionServer_InitTrajPointFull(ctrlGroup, &jointTrajData);
}

//-----------------------------------------------------------------------
// Setup the first point of a trajectory
//-----------------------------------------------------------------------
int Ros_MotionServer_InitTrajPointFull(CtrlGroup* ctrlGroup, SmBodyJointTrajPtFull* jointTrajData)
{
	long pulsePos[MAX_PULSE_AXES];
	long curPos[MAX_PULSE_AXES];
	int i;

	if(ctrlGroup->groupNo == jointTrajData->groupNo)
	{
		// Assign start position
		Ros_MotionServer_ConvertToJointMotionData(jointTrajData, &ctrlGroup->jointMotionData);
		ctrlGroup->timeLeftover_ms = 0;
		ctrlGroup->q_time = ctrlGroup->jointMotionData.time;
		
		// For MPL80/100 robot type (SLUBT): Controller automatically moves the B-axis
		// to maintain orientation as other axes are moved.
		if (ctrlGroup->bIsBaxisSlave)
		{
			//ROS joint order
			ctrlGroup->jointMotionData.pos[3] += -ctrlGroup->jointMotionData.pos[1] + ctrlGroup->jointMotionData.pos[2];
			ctrlGroup->jointMotionData.vel[3] += -ctrlGroup->jointMotionData.vel[1] + ctrlGroup->jointMotionData.vel[2];
		}

		// Convert start position to pulse format
		Ros_CtrlGroup_ConvertToMotoPos(ctrlGroup, ctrlGroup->jointMotionData.pos, pulsePos);
		Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, curPos);

		// Initialize prevPulsePos to the current position
		Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, ctrlGroup->prevPulsePos);
		
		// Check for each axis
		for(i=0; i<MAX_PULSE_AXES; i++)
		{
			// Check if position matches current command position
			if(abs(pulsePos[i] - curPos[i]) > START_MAX_PULSE_DEVIATION)
			{
				printf("ERROR: Trajectory start position doesn't match current position (MOTO joint order).\r\n");
				printf(" - Requested start: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
					pulsePos[0], pulsePos[1], pulsePos[2],
					pulsePos[3], pulsePos[4], pulsePos[5],
					pulsePos[6], pulsePos[7]);
				printf(" - Current pos: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
					curPos[0], curPos[1], curPos[2],
					curPos[3], curPos[4], curPos[5],
					curPos[6], curPos[7]);
				printf(" - ctrlGroup->prevPulsePos: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
					ctrlGroup->prevPulsePos[0], ctrlGroup->prevPulsePos[1], ctrlGroup->prevPulsePos[2],
					ctrlGroup->prevPulsePos[3], ctrlGroup->prevPulsePos[4], ctrlGroup->prevPulsePos[5],
					ctrlGroup->prevPulsePos[6], ctrlGroup->prevPulsePos[7]);

				return ROS_RESULT_INVALID_DATA_START_POS;
			}
			
			// Check maximum velocity limit
			if(abs(ctrlGroup->jointMotionData.vel[i]) > ctrlGroup->maxSpeed[i])
			{
				// excessive speed
				return ROS_RESULT_INVALID_DATA_SPEED;
			}
		}		

		//printf("Trajectory Start Initialized\r\n");
		// Return success
		return 0;
	}
	
	return ROS_RESULT_INVALID_GROUPNO;
}

//-----------------------------------------------------------------------
// Convert SmBodyMotoJointTrajPtExData data to SmBodyJointTrajPtFull
//-----------------------------------------------------------------------
int Ros_MotionServer_AddTrajPointFullEx(CtrlGroup* ctrlGroup, SmBodyJointTrajPtExData* jointTrajDataEx, int sequence)
{
	SmBodyJointTrajPtFull jointTrajData;

	//convert SmBodyMotoJointTrajPtExData data to SmBodyJointTrajPtFull
	jointTrajData.groupNo = jointTrajDataEx->groupNo;
	jointTrajData.sequence = sequence;
	jointTrajData.validFields = jointTrajDataEx->validFields;
	jointTrajData.time = jointTrajDataEx->time;
	memcpy(jointTrajData.pos, jointTrajDataEx->pos, sizeof(float)*ROS_MAX_JOINT);
	memcpy(jointTrajData.vel, jointTrajDataEx->vel, sizeof(float)*ROS_MAX_JOINT);
	memcpy(jointTrajData.acc, jointTrajDataEx->acc, sizeof(float)*ROS_MAX_JOINT);

	return Ros_MotionServer_AddTrajPointFull(ctrlGroup, &jointTrajData);
}


//-----------------------------------------------------------------------
// Setup the subsequent point of a trajectory
//-----------------------------------------------------------------------
int Ros_MotionServer_AddTrajPointFull(CtrlGroup* ctrlGroup, SmBodyJointTrajPtFull* jointTrajData)
{
	int i;
	JointMotionData jointData;

	// Check that there isn't data current being processed
	if(ctrlGroup->hasDataToProcess)
	{
		// Busy
		return ROS_RESULT_BUSY;
	}
	
	// Convert message data to a jointMotionData
	Ros_MotionServer_ConvertToJointMotionData(jointTrajData, &jointData);
			
	// Check that incoming data is valid
	for(i=0; i<ctrlGroup->numAxes; i++)
	{
		// Check position softlimit
		// TODO? Note need to add function to Parameter Extraction Library
		
		// Velocity check
		if(abs(jointData.vel[i]) > ctrlGroup->maxSpeed[i])
		{
			// excessive speed
			printf("ERROR: Invalid speed in message TrajPointFull data: \r\n  axis: %d, speed: %f, limit: %f\r\n", 
				i, jointData.vel[i], ctrlGroup->maxSpeed[i]);
				
			#ifdef DEBUG
				Ros_SimpleMsg_DumpTrajPtFull(jointTrajData);
			#endif
	
			return ROS_RESULT_INVALID_DATA_SPEED;
		}
	}			

	// Store of the message trajectory data to the control group for processing 
	memcpy(&ctrlGroup->jointMotionDataToProcess, &jointData, sizeof(JointMotionData));
	ctrlGroup->hasDataToProcess = TRUE;

	return 0;
}


//-----------------------------------------------------------------------
// Task that handles in the background messages that may have long processing
// time so that they don't block other message from being processed.
// Checks the type of message and processes it accordingly. 
//-----------------------------------------------------------------------
void Ros_MotionServer_AddToIncQueueProcess(Controller* controller, int groupNo)
{
	int interpolPeriod;
	CtrlGroup* ctrlGroup = controller->ctrlGroups[groupNo];

	// Initialization of pointers and memory
	interpolPeriod = controller->interpolPeriod; 
	ctrlGroup->hasDataToProcess = FALSE;

	FOREVER
	{
		// if there is no message to process delay and try agsain
		if(ctrlGroup->hasDataToProcess)
		{
			// Interpolate increment move to reach position data
			Ros_MotionServer_JointTrajDataToIncQueue(controller, groupNo);
			
			// Mark message as processed 
			ctrlGroup->hasDataToProcess = FALSE;
		}
		
		Ros_Sleep(interpolPeriod);
	}		
}


//-----------------------------------------------------------------------
// Decompose the message type: ROS_MSG_JOINT_TRAJ_PT_FULL into incremental
// moves to be added to the inc move queue.
// Interpolation is based on position, velocity and time
// Acceleration is modeled by a linear equation acc = accCoef1 + accCoef2 * time
//-----------------------------------------------------------------------
void Ros_MotionServer_JointTrajDataToIncQueue(Controller* controller, int groupNo)
{
	int interpolPeriod = controller->interpolPeriod; 
	CtrlGroup* ctrlGroup = controller->ctrlGroups[groupNo];
	int i; 
	JointMotionData _startTrajData;
	JointMotionData* startTrajData;
	JointMotionData* endTrajData;
	JointMotionData* curTrajData;
	float interval;						// Time between startTime and the new data time
	float accCoef1[MP_GRP_AXES_NUM];    // Acceleration coefficient 1
	float accCoef2[MP_GRP_AXES_NUM];    // Acceleration coefficient 2
	int timeInc_ms;						// time increment in millisecond
	int calculationTime_ms;				// time in ms at which the interpolation takes place
	float interpolTime;      			// time increment in second
	long newPulsePos[MP_GRP_AXES_NUM];
	Incremental_data incData;

	//printf("Starting JointTrajDataProcess\r\n");	

	// Initialization of pointers and memory
	curTrajData = &ctrlGroup->jointMotionData;
	endTrajData = &ctrlGroup->jointMotionDataToProcess;
	startTrajData = &_startTrajData;
	// Set the start of the trajectory interpolation as the current position (which should be the end of last interpolation)
	memcpy(startTrajData, curTrajData, sizeof(JointMotionData));

	// For MPL80/100 robot type (SLUBT): Controller automatically moves the B-axis
	// to maintain orientation as other axes are moved.
	if (ctrlGroup->bIsBaxisSlave)
	{
		//ROS joint order
		endTrajData->pos[3] += -endTrajData->pos[1] + endTrajData->pos[2];
		endTrajData->vel[3] += -endTrajData->vel[1] + endTrajData->vel[2];
	}

	memset(newPulsePos, 0x00, sizeof(newPulsePos));
	memset(&incData, 0x00, sizeof(incData));
	incData.frame = MP_INC_PULSE_DTYPE;
	// also prevents ROS_MSG_MOTO_SELECT_TOOL from changing the active tool while processing
	incData.tool = ctrlGroup->tool;
	
	// Calculate an acceleration coefficients
	memset(&accCoef1, 0x00, sizeof(accCoef1));
	memset(&accCoef2, 0x00, sizeof(accCoef2));
	interval = (endTrajData->time - startTrajData->time) / 1000.0f;  // time difference in sec
	if (interval > 0.0)
	{
		for (i = 0; i < ctrlGroup->numAxes; i++)
		{	
			//Calculate acceleration coefficient (convert interval to seconds
			accCoef1[i] = ( 6 * (endTrajData->pos[i] - startTrajData->pos[i]) / (interval * interval) )
						- ( 2 * (endTrajData->vel[i] + 2 * startTrajData->vel[i]) / interval);
			accCoef2[i] = ( -12 * (endTrajData->pos[i] - startTrajData->pos[i]) / (interval * interval * interval))
						+ ( 6 * (endTrajData->vel[i] + startTrajData->vel[i]) / (interval * interval) );
		}
	}
	else
	{
		printf("Warning: Group %d - Time difference between endTrajData (%d) and startTrajData (%d) is 0 or less.\r\n", groupNo, endTrajData->time, startTrajData->time);
	}
	
	// Initialize calculation variable before entering while loop
	calculationTime_ms = startTrajData->time;
	if(ctrlGroup->timeLeftover_ms == 0)
		timeInc_ms = interpolPeriod;
	else
		timeInc_ms = ctrlGroup->timeLeftover_ms;
		
	// While interpolation time is smaller than new ROS point time
	while( (curTrajData->time < endTrajData->time) && Ros_Controller_IsMotionReady(controller))
	{
		// Increment calculation time by next time increment
		calculationTime_ms += timeInc_ms;
		interpolTime = (calculationTime_ms - startTrajData->time) / 1000.0f;
			
		if( calculationTime_ms < endTrajData->time )  // Make calculation for full interpolation clock
		{	   
			// Set new interpolation time to calculation time
			curTrajData->time = calculationTime_ms;
				
			// For each axis calculate the new position at the interpolation time
			for (i = 0; i < ctrlGroup->numAxes; i++)
			{
				// Add position change for new interpolation time 
				curTrajData->pos[i] = startTrajData->pos[i] 						// initial position component
					+ startTrajData->vel[i] * interpolTime  						// initial velocity component
					+ accCoef1[i] * interpolTime * interpolTime / 2 				// accCoef1 component
					+ accCoef2[i] * interpolTime * interpolTime * interpolTime / 6;	// accCoef2 component
	
				// Add velocity change for new interpolation time
				curTrajData->vel[i] = startTrajData->vel[i]   						// initial velocity component
					+ accCoef1[i] * interpolTime 									// accCoef1 component
					+ accCoef2[i] * interpolTime * interpolTime / 2;				// accCoef2 component
			}
	
			// Reset the timeInc_ms for the next interpolation cycle
			if(timeInc_ms < interpolPeriod)
			{
				timeInc_ms = interpolPeriod;
				ctrlGroup->timeLeftover_ms = 0;
			}
		}
		else  // Make calculation for partial interpolation cycle
		{
			// Set the current trajectory data equal to the end trajectory
			memcpy(curTrajData, endTrajData, sizeof(JointMotionData));
	
			// Set the next interpolation increment to the the remainder to reach the next interpolation cycle  
			if(calculationTime_ms > endTrajData->time)
			{
				ctrlGroup->timeLeftover_ms = calculationTime_ms - endTrajData->time;
			} 
		}
	
		// Convert position in motoman pulse joint
		Ros_CtrlGroup_ConvertToMotoPos(ctrlGroup, curTrajData->pos, newPulsePos);
		
		// Calculate the increment
		incData.time = curTrajData->time;
		for (i = 0; i < MP_GRP_AXES_NUM; i++)
		{
			if (ctrlGroup->axisType.type[i] != AXIS_INVALID)
				incData.inc[i] = (newPulsePos[i] - ctrlGroup->prevPulsePos[i]);
			else
				incData.inc[i] = 0;
		}
		
		// Add the increment to the queue
		if(!Ros_MotionServer_AddPulseIncPointToQ(controller, groupNo, &incData))
			break;
			
		// Copy data to the previous pulse position for next iteration
		memcpy(ctrlGroup->prevPulsePos, newPulsePos, sizeof(ctrlGroup->prevPulsePos));
	}
}


//-------------------------------------------------------------------
// Adds pulse increments for one interpolation period to the inc move queue
//-------------------------------------------------------------------
BOOL Ros_MotionServer_AddPulseIncPointToQ(Controller* controller, int groupNo, Incremental_data* dataToEnQ)
{	
	int index;
	
	// Set pointer to specified queue
	Incremental_q* q = &controller->ctrlGroups[groupNo]->inc_q;

	while( q->cnt >= Q_SIZE ) //queue is full
	{
		//wait for items to be removed from the queue
		Ros_Sleep(controller->interpolPeriod);
		
		//make sure we don't get stuck in infinite loop
		if (!Ros_Controller_IsMotionReady(controller))  //<- they probably pressed HOLD or ESTOP or stop motion request
		{
			return FALSE;
		}
	}
	
	// Lock the q before manipulating it
	if(mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
	{
		// Get the index of the end of the queue
		index = Q_OFFSET_IDX( q->idx, q->cnt , Q_SIZE );
		// Copy data at the end of the queue
		q->data[index] = *dataToEnQ;
		// increase the count of elements in the queue
		q->cnt++;
		
		// Unlock the q
		mpSemGive(q->q_lock);
	}
	else
	{
		printf("ERROR: Unable to add point to queue.  Queue is locked up!\r\n");
		return FALSE;
	}
	
	return TRUE;
}


//-------------------------------------------------------------------
// Clears the inc move queue
//-------------------------------------------------------------------
BOOL Ros_MotionServer_ClearQ(Controller* controller, int groupNo)
{
	Incremental_q* q;

	// Check group number valid
	if(!Ros_Controller_IsValidGroupNo(controller, groupNo))
		return FALSE;

	// Stop addtional items from being added to the queue
	controller->ctrlGroups[groupNo]->hasDataToProcess = FALSE;

	// Set pointer to specified queue
	q = &controller->ctrlGroups[groupNo]->inc_q;

	// Lock the q before manipulating it
	if(mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
	{
		// Reset the queue.  No need to modify index or delete data
		q->cnt = 0;
		
		// Unlock the q
		mpSemGive(q->q_lock);
		
		return TRUE;
	}

	return FALSE;
}


//-------------------------------------------------------------------
// Clears the inc move queue
//-------------------------------------------------------------------
BOOL Ros_MotionServer_ClearQ_All(Controller* controller)
{
	int groupNo;
	BOOL bRet = TRUE;
	
	for(groupNo=0; groupNo<controller->numGroup; groupNo++)
	{
		bRet &= Ros_MotionServer_ClearQ(controller, groupNo);
	}
#ifdef DEBUG
	printf("All buffers cleared\r\n");
#endif
	return bRet;
}


//-------------------------------------------------------------------
// Check the number of inc_move currently in the specified queue
//-------------------------------------------------------------------
int Ros_MotionServer_GetQueueCnt(Controller* controller, int groupNo)
{
	Incremental_q* q;
	int count;
	
	// Check group number valid
	if(!Ros_Controller_IsValidGroupNo(controller, groupNo))
		return -1;

	// Set pointer to specified queue
	q = &controller->ctrlGroups[groupNo]->inc_q;
	
	// Lock the q before manipulating it
	if(mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
	{			
		count = q->cnt;
			
		// Unlock the q
		mpSemGive(q->q_lock);
		
		return count;
	}
		
	printf("ERROR: Unable to access queue count.  Queue is locked up!\r\n");
	return ERROR;
}



//-------------------------------------------------------------------
// Check that at least one control group of the controller has data in queue
//-------------------------------------------------------------------
BOOL Ros_MotionServer_HasDataInQueue(Controller* controller)
{
	int groupNo;
	int qCnt;
	
	for(groupNo=0; groupNo<controller->numGroup; groupNo++)
	{
		qCnt = Ros_MotionServer_GetQueueCnt(controller, groupNo);
		if (qCnt > 0)
			return TRUE;
		else if (qCnt == ERROR)
			return ERROR;
	}
		
	return FALSE;
}


//-------------------------------------------------------------------
// Task to move the robot at each interpolation increment
// 06/05/13: Modified to always send information for all defined groups even if the inc_q is empty
//-------------------------------------------------------------------
void Ros_MotionServer_IncMoveLoopStart(Controller* controller) //<-- IP_CLK priority task
{
#if DX100
	MP_POS_DATA moveData;
#else
	MP_EXPOS_DATA moveData;
#endif

	Incremental_q* q;
	int i;
	int ret;
	LONG time;
	LONG q_time;
	int axis;
	//BOOL bNoData = TRUE;  // for testing
	MP_CTRL_GRP_SEND_DATA ctrlGrpData;
	MP_PULSE_POS_RSP_DATA prevPulsePosData[MAX_CONTROLLABLE_GROUPS];
	MP_PULSE_POS_RSP_DATA pulsePosData;
	
	// --- FSU Speed Limit related ---
	// When FSU speed limitation is active, some pulses for an interpolation cycle may not be processed by the controller.
	// To track the true amount of pulses processed, we keep track of the command position and by substracting the
	// the previous position from the current one, we can confirm the amount if pulses precessed.  
	// If the amount processed doesn't match the amount sent at the previous cycle, we resend the unprocessed pulses amount.
	// To keep the motion smooth, the 'maximum speed' (max pulses per cycle) is tracked and used to skip reading 
	// more pulse increment from the queue if the amount of unprocessed pulses is larger than the detected speed.
	// The 'maximum speed' is also used to prevent exceeding the commanded speed once the FSU Speed Limit is removed. 
	//  
	// The following set of variables are used to track FSU speed limitation.
	LONG newPulseInc[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];		// Pulse increments that we just retrieved from the incQueue
	LONG toProcessPulses[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];	// Total pulses that still need to be sent to the command
	LONG processedPulses[MP_GRP_AXES_NUM];							// Amount of pulses from the last command that were actually processed (accepted)   
	LONG maxSpeed[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];		// ROS speed (amount of pulses for one cycle from the data queue) that should not be exceeded  
	LONG maxSpeedRemain[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];	// Number of pulses (absolute) that remains to be processed at the 'maxSpeed'
	LONG prevMaxSpeed[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];		// Previous data queue reading 'maxSpeed'
	LONG prevMaxSpeedRemain[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];  // Previous data queue reading 'maxSpeedRemain'
	BOOL skipReadingQ[MAX_CONTROLLABLE_GROUPS];						// Flag indicating to skip reading more data from the increment queue (there is enough unprocessed from previous cycles remaining) 
	BOOL queueRead[MAX_CONTROLLABLE_GROUPS];						// Flag indicating that new increment data was retrieve from the queue on this cycle. 
	BOOL isMissingPulse;											// Flag that there are pulses send in last cycle that are missing from the command (pulses were not processed)  
	BOOL hasUnprocessedData;										// Flag that at least one axis (any group) still has unprecessed data. (Used to continue sending data after the queue is empty.)

	memset(newPulseInc, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
	memset(toProcessPulses, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
	memset(processedPulses, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM);
	memset(prevMaxSpeed, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
	memset(prevMaxSpeedRemain, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
	memset(maxSpeed, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
	memset(maxSpeedRemain, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
	memset(skipReadingQ, 0x00, sizeof(BOOL) * MAX_CONTROLLABLE_GROUPS);
	memset(queueRead, 0x00, sizeof(BOOL) * MAX_CONTROLLABLE_GROUPS);

	isMissingPulse = FALSE;
	hasUnprocessedData = FALSE;

	printf("IncMoveTask Started\r\n");
	
	memset(&moveData, 0x00, sizeof(moveData));

	for(i=0; i<controller->numGroup; i++)
	{
		moveData.ctrl_grp |= (0x01 << i); 
		moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(controller->ctrlGroups[i]);
		ctrlGrpData.sCtrlGrp = controller->ctrlGroups[i]->groupId;
		mpGetPulsePos(&ctrlGrpData, &prevPulsePosData[i]);
	}

	FOREVER
	{
		mpClkAnnounce(MP_INTERPOLATION_CLK);
		
		if (Ros_Controller_IsMotionReady(controller) 
			&& (Ros_MotionServer_HasDataInQueue(controller) || hasUnprocessedData)
			&& !controller->bStopMotion)
		{
			//bNoData = FALSE;   // for testing
			
			// For each control group, retrieve the new pulse increments for this cycle
			for (i = 0; i < controller->numGroup; i++)
			{
				queueRead[i] = FALSE;

				if (skipReadingQ[i])
				{
					// Reset skip flag and set position increment to 0
					skipReadingQ[i] = FALSE;
					memset(&moveData.grp_pos_info[i].pos, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM);
				}
				else
				{
					// Retrieve position increment from the queue.
					q = &controller->ctrlGroups[i]->inc_q;

					// Lock the q before manipulating it
					if (mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
					{
						if (q->cnt > 0)
						{
							// Initialize moveData with the next data from the queue
							time = q->data[q->idx].time;
							q_time = controller->ctrlGroups[i]->q_time;
							moveData.grp_pos_info[i].pos_tag.data[2] = q->data[q->idx].tool;
							moveData.grp_pos_info[i].pos_tag.data[3] = q->data[q->idx].frame;
							moveData.grp_pos_info[i].pos_tag.data[4] = q->data[q->idx].user;

							memcpy(&moveData.grp_pos_info[i].pos, &q->data[q->idx].inc, sizeof(LONG) * MP_GRP_AXES_NUM);
							queueRead[i] = TRUE;
#ifdef DEBUG
							Debug_BroadcastMsg("New Inc from Buffer: %d, %d, %d, %d, %d, %d\r\n", 
								moveData.grp_pos_info[i].pos[0], moveData.grp_pos_info[i].pos[1], moveData.grp_pos_info[i].pos[2],
								moveData.grp_pos_info[i].pos[3], moveData.grp_pos_info[i].pos[4], moveData.grp_pos_info[i].pos[5]);
#endif
							// increment index in the queue and decrease the count
							q->idx = Q_OFFSET_IDX(q->idx, 1, Q_SIZE);
							q->cnt--;

							// Check if complet interpolation period covered.
							// (Because time period of data received from ROS may not be a multiple of the 
							// controller interpolation clock period, the queue may contain partiel period and 
							// more than one queue increment maybe required to complete the interpolation period)
							while (q->cnt > 0)
							{
								if ((q_time <= q->data[q->idx].time)
									&& (q->data[q->idx].time - q_time <= controller->interpolPeriod))
								{
									// next incMove is part of same interpolation period

									// check that information is in the same format
									if ((moveData.grp_pos_info[i].pos_tag.data[2] != q->data[q->idx].tool)
										|| (moveData.grp_pos_info[i].pos_tag.data[3] != q->data[q->idx].frame)
										|| (moveData.grp_pos_info[i].pos_tag.data[4] != q->data[q->idx].user))
									{
										// Different format can't combine information
										break;
									}

									// add next incMove to current incMove
									for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
										moveData.grp_pos_info[i].pos[axis] += q->data[q->idx].inc[axis];
									time = q->data[q->idx].time;

									// increment index in the queue and decrease the count
									q->idx = Q_OFFSET_IDX(q->idx, 1, Q_SIZE);
									q->cnt--;
								}
								else
								{
									// interpolation period complet
									break;
								}
							}

							controller->ctrlGroups[i]->q_time = time;
						}
						else
						{
							// Queue is empty, initialize to 0 pulse increment
							moveData.grp_pos_info[i].pos_tag.data[2] = 0;
							moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
							moveData.grp_pos_info[i].pos_tag.data[4] = 0;
							memset(&moveData.grp_pos_info[i].pos, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM);
						}

						// Unlock the q					
						mpSemGive(q->q_lock);
					}
					else
					{
						printf("ERROR: Can't get data from queue. Queue is locked up.\r\n");
						memset(&moveData.grp_pos_info[i].pos, 0x00, sizeof(LONG) * MP_GRP_AXES_NUM);
						continue;
					}
				}
			}


			// --- FSU Speed Limit Check ---
			hasUnprocessedData = FALSE;

			for (i = 0; i < controller->numGroup; i++) 
			{
				memcpy(newPulseInc[i], moveData.grp_pos_info[i].pos, sizeof(LONG)* MP_GRP_AXES_NUM);

				// record the speed associate with the next amount of pulses
				if (queueRead[i])
				{
					for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
					{
						maxSpeed[i][axis] = abs(newPulseInc[i][axis]);
						maxSpeedRemain[i][axis] = abs(newPulseInc[i][axis]);
					}
				}

				// Check if pulses are missing from last increment.
				// Get the current controller command position and substract the previous command position
				// and check if it matches the amount if increment sent last cycle.  If it doesn't then
				// some pulses are missing and the amount of unprocessed pulses needs to be added to this cycle.
				ctrlGrpData.sCtrlGrp = controller->ctrlGroups[i]->groupId;
				mpGetPulsePos(&ctrlGrpData, &pulsePosData);
				isMissingPulse = FALSE;
				for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
				{
					// Check how many pulses we processed from last increment
					processedPulses[axis] = pulsePosData.lPos[axis] - prevPulsePosData[i].lPos[axis];
					prevPulsePosData[i].lPos[axis] = pulsePosData.lPos[axis];
					
					// Remove those pulses from the amount to process.  
					// If everything was processed, then there should by 0 pulses left. Otherwise FSU Speed limit prevented processing
					toProcessPulses[i][axis] -= processedPulses[axis];
					if (toProcessPulses[i][axis] != 0)
						isMissingPulse = TRUE;
					
					// Add the new pulses to be processed for this iteration 
					toProcessPulses[i][axis] += newPulseInc[i][axis];

					if (toProcessPulses[i][axis] != 0)
						hasUnprocessedData = TRUE;
				}

				// Check if pulses are missing which means that the FSU speed limit is enabled
				if (isMissingPulse) 
				{
					LONG max_inc;

					// Prevent going faster than original requested speed once speed limit turns off
					// Check if the speed (inc) of previous interation should be considered by checking 
					// if the unprocessed pulses from that speed setting still remains.
					// If all the pulses of previous increment were processed, then transfer the current 
					// speed and process the next increment from the increment queue.
					for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
					{
						// Check if has pulses to process
						if (toProcessPulses[i][axis] == 0)
							prevMaxSpeedRemain[i][axis] = 0;
						else
							prevMaxSpeedRemain[i][axis] = abs(prevMaxSpeedRemain[i][axis]) - abs(processedPulses[axis]);
					}

					// Check if still have data to process from previous iteration
					skipReadingQ[i] = FALSE;
					for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
					{
						if (prevMaxSpeedRemain[i][axis] > 0)
							skipReadingQ[i] = TRUE;
					}

					if (!skipReadingQ[i]) {
						for (axis = 0; axis < MP_GRP_AXES_NUM; axis++) {
							// Transfer the current speed as the new prevSpeed
							prevMaxSpeed[i][axis] = maxSpeed[i][axis];
							prevMaxSpeedRemain[i][axis] += maxSpeedRemain[i][axis];
						}
					}

					// Set the number of pulse that can be sent without exceeding speed
					for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
					{
						// Check if has pulses to process
						if (toProcessPulses[i][axis] == 0)
							continue;

						// Maximum inc that should be send ()
						if (prevMaxSpeed[i][axis] > 0)
							// if previous speed is defined use it
							max_inc = prevMaxSpeed[i][axis]; 
						else
						{
							if (maxSpeed[i][axis] > 0)
								// else fallback on current speed if defined
								max_inc = maxSpeed[i][axis];
							else if (newPulseInc[i][axis] != 0)
								// use the current speed if none zero.
								max_inc = abs(newPulseInc[i][axis]);
							else
								// otherwise use the axis max speed
								max_inc = controller->ctrlGroups[i]->maxInc.maxIncrement[axis];
#ifdef DEBUG
							Debug_BroadcastMsg("Warning undefined speed: Axis%d Defaulting Max Inc: %d (prevSpeed: %d curSpeed %d)\r\n", 
								axis, max_inc, prevMaxSpeed[i][axis], maxSpeed[i][axis]);
#endif
						}

						// Set new increment and recalculate unsent pulses
						if (abs(toProcessPulses[i][axis]) <= max_inc)
						{
							// Pulses to send is small than max, so send everything
							moveData.grp_pos_info[i].pos[axis] = toProcessPulses[i][axis];
						}
						else {
							// Pulses to send is too high, so send the amount matching the maximum speed
							if (toProcessPulses[i][axis] >= 0)
								moveData.grp_pos_info[i].pos[axis] = max_inc;
							else
								moveData.grp_pos_info[i].pos[axis] = -max_inc;
						}
					}
				}
				else
				{
					// No PFL Speed Limit detected
					for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
					{
						prevMaxSpeed[i][axis] = abs(moveData.grp_pos_info[i].pos[axis]);
						prevMaxSpeedRemain[i][axis] = abs(moveData.grp_pos_info[i].pos[axis]);
					}
				}
			}

#if DX100
			// first robot
			if (controller->bIsDx100Sda)
				moveData.ctrl_grp = 1 | (1 << 2); //R1 + B1
			else
				moveData.ctrl_grp = 1; //R1 only
			ret = mpMeiIncrementMove(MP_SL_ID1, &moveData);
			if (ret != 0)
			{
				controller->bMpIncMoveError = TRUE;
				Ros_MotionServer_StopMotion(controller);
				if (ret == -3)
					printf("mpMeiIncrementMove returned: %d (ctrl_grp = %d)\r\n", ret, moveData.ctrl_grp);
				else
					printf("mpMeiIncrementMove returned: %d\r\n", ret);
			}
			// if second robot
			moveData.ctrl_grp = 2; //R2 only
			if (controller->numRobot > 1)
			{
				ret = mpMeiIncrementMove(MP_SL_ID2, &moveData);
				if (ret != 0)
				{
					controller->bMpIncMoveError = TRUE;
					Ros_MotionServer_StopMotion(controller);
					if (ret == -3)
						printf("mpMeiIncrementMove returned: %d (ctrl_grp = %d)\r\n", ret, moveData.ctrl_grp);
					else
						printf("mpMeiIncrementMove returned: %d\r\n", ret);
				}
			}
#else
			// Send pulse increment to the controller command position
			ret = mpExRcsIncrementMove(&moveData);
			if(ret != 0)
			{
				// Failure: command rejected by controller
				// Update controller status to help identify cause
				Ros_Controller_StatusUpdate(controller);

				if(ret == E_EXRCS_CTRL_GRP)
					printf("mpExRcsIncrementMove returned: %d (ctrl_grp = %d)\r\n", ret, moveData.ctrl_grp);
#if (YRC1000||YRC1000u)
				else if (ret == E_EXRCS_IMOV_UNREADY && controller->bPFLEnabled )
				{
					// Check if this is caused by a known cause (E-Stop, Hold, Alarm, Error)
					if ( !Ros_Controller_IsEStop(controller) && !Ros_Controller_IsHold(controller) 
						&& !Ros_Controller_IsAlarm(controller) && !Ros_Controller_IsError(controller) ) {
						printf("mpExRcsIncrementMove returned UNREADY: %d (Could be PFL Active)\r\n", E_EXRCS_IMOV_UNREADY);
						controller->bPFLduringRosMove = TRUE;
					}
				}
				else if (ret == E_EXRCS_PFL_FUNC_BUSY && controller->bPFLEnabled)
				{
					printf("mpExRcsIncrementMove returned PFL Active\r\n");
					controller->bPFLduringRosMove = TRUE;	
				}
				else if (ret == E_EXRCS_UNDER_ENERGY_SAVING)
				{
					// retry until servos turn on and motion is accepted
					int checkCount;
					for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
					{
						ret = mpExRcsIncrementMove(&moveData);
						if (ret != E_EXRCS_UNDER_ENERGY_SAVING) 
							break;

						Ros_Sleep(MOTION_START_CHECK_PERIOD);
					}
					if(controller->bMpIncMoveError)
						printf("mpExRcsIncrementMove returned Eco mode enabled\r\n");
				}
#endif
				else
					printf("mpExRcsIncrementMove returned: %d\r\n", ret);

				// Stop motion if motion was rejected
				if (ret != 0) 
				{
					// Flag to prevent further motion until Trajectory mode is reenabled
					controller->bMpIncMoveError = TRUE;
					Ros_MotionServer_StopMotion(controller);
				}
			}
#endif

#ifdef DEBUG
			i = 0;
			for (axis = 0; axis < 6; axis++)

				Debug_BroadcastMsg("Axis %d: New: %d ToProcess: %d Sent: %d MaxSpeed: %d for remaining %d  Skip: %d\r\n",
					axis, newPulseInc[i][axis], toProcessPulses[i][axis],
					moveData.grp_pos_info[i].pos[axis], prevMaxSpeed[i][axis], prevMaxSpeedRemain[i][axis], skipReadingQ[i]);
#endif		
		}
		else  
		{
			// Reset previous position in case the robot is moved externally
			memset(toProcessPulses, 0x00, sizeof(LONG)* MP_GRP_AXES_NUM* MAX_CONTROLLABLE_GROUPS);
			hasUnprocessedData = FALSE;
			for (i = 0; i < controller->numGroup; i++)
			{
				ctrlGrpData.sCtrlGrp = controller->ctrlGroups[i]->groupId;
				mpGetPulsePos(&ctrlGrpData, &prevPulsePosData[i]);
			}
		}
	}
}



//-----------------------------------------------------------------------
// Convert a JointTrajData message to a JointMotionData of a control group
//-----------------------------------------------------------------------
void Ros_MotionServer_ConvertToJointMotionData(SmBodyJointTrajPtFull* jointTrajData, JointMotionData* jointMotionData)
{
	int i, maxAxes;

	memset(jointMotionData, 0x00, sizeof(JointMotionData));

	maxAxes = min(ROS_MAX_JOINT, MP_GRP_AXES_NUM);
	
	jointMotionData->flag = jointTrajData->validFields;
	jointMotionData->time = (int)(jointTrajData->time * 1000);
	
	for(i=0; i<maxAxes; i++)
	{
		jointMotionData->pos[i] = jointTrajData->pos[i];
		jointMotionData->vel[i] = jointTrajData->vel[i];
		jointMotionData->acc[i] = jointTrajData->acc[i];
	}
}

void Ros_MotionServer_PrintError(USHORT err_no, char* msgPrefix)
{
	char errMsg[ERROR_MSG_MAX_SIZE];
	memset(errMsg, 0x00, ERROR_MSG_MAX_SIZE);
	Ros_Controller_ErrNo_ToString(err_no, errMsg, ERROR_MSG_MAX_SIZE);
	printf("%s %s\r\n", msgPrefix, errMsg);
}

STATUS Ros_MotionServer_DisableEcoMode(Controller* controller)
{
	MP_SERVO_POWER_SEND_DATA sServoData;
	MP_STD_RSP_DATA rData;
	int ret;

#ifdef DUMMY_SERVO_MODE
	return OK;
#endif

	if (Ros_Controller_IsEcoMode(controller) == TRUE)
	{
		//toggle servos to disable energy-savings mode
		sServoData.sServoPower = 0;  // OFF
		memset(&sServoData, 0x00, sizeof(sServoData));
		memset(&rData, 0x00, sizeof(rData));
		ret = mpSetServoPower(&sServoData, &rData);
		if ((ret == 0) && (rData.err_no == 0))
		{
			// wait for the Servo/Eco OFF confirmation
			int checkCount;
			for (checkCount = 0; checkCount<MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsEcoMode(controller) == FALSE)
					break;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
		}
		else
		{
			Ros_MotionServer_PrintError(rData.err_no, "Can't disable energy-savings mode because:");
			return NG;
		}
	}

	if (Ros_Controller_IsEcoMode(controller) == FALSE)
		return OK;
	else
		return NG;
}

int Ros_MotionServer_GetDhParameters(Controller* controller, SimpleMsg* replyMsg)
{
	int i; 
	STATUS apiRet = OK;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoGetDhParameters);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_GET_DH_PARAMETERS;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	replyMsg->header.replyType = ROS_REPLY_SUCCESS;

	for (i = 0; i < controller->numGroup; i += 1)
	{
		if (controller->ctrlGroups[i] != NULL && replyMsg->header.replyType == ROS_REPLY_SUCCESS)
		{
			apiRet = GP_getDhParameters(i, &replyMsg->body.dhParameters.dhParameters[i]);

			if (apiRet == OK)
				replyMsg->header.replyType = ROS_REPLY_SUCCESS;
			else
				replyMsg->header.replyType = ROS_REPLY_FAILURE;
		}
	}

	return apiRet;
}


int Ros_MotionServer_SetSelectedTool(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
#ifndef FS100
	MP_SET_TOOL_NO_SEND_DATA setToolData;
	MP_STD_RSP_DATA responseData;
#endif

	int groupNo = receiveMsg->body.selectTool.groupNo;
	int tool = receiveMsg->body.selectTool.tool;

	if (groupNo >= 0 && groupNo < controller->numRobot)
	{	
		if (tool >= MIN_VALID_TOOL_INDEX && tool <= MAX_VALID_TOOL_INDEX)
		{
			//set tool that will be used by motion API
			controller->ctrlGroups[receiveMsg->body.selectTool.groupNo]->tool = tool;

#ifndef FS100
			//set jogging tool on the pendant
			setToolData.sRobotNo = controller->ctrlGroups[receiveMsg->body.selectTool.groupNo]->groupId;
			setToolData.sToolNo = tool;
			mpSetToolNo(&setToolData, &responseData);
#endif

			//We don't care if mpSetToolNo fails. It won't affect the actual motion.
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, groupNo);
		}
		else
			Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_DATA_TOOLNO, replyMsg, groupNo);
	}
	else
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_GROUPNO, replyMsg, groupNo);

	return 0;
}

void Ros_MotionServer_EnsureEcoModeIsDisabled(Controller* controller)
{
    //It's possible for energy-saving mode to activate between /robot_enable and the first trajectory point.
    //If eco-mode is active, then automatically invoke a /robot_enable.
    if (Ros_Controller_IsEcoMode(controller))
    {
        printf("Energy Saving Function is active. Clearing with automatic robot_enable.\n");

        Ros_MotionServer_StopTrajMode(controller);
        Ros_Sleep(100); //give time for Ros_Controller_StatusUpdate on other task
        Ros_MotionServer_StartTrajMode(controller);

        //Give time for the controller to recognize that the INFORM cursor is sitting on
        //a WAIT instructions. Without this delay, the first call to mpExRcsIncrementMove
        //will fail with a (-1).
        Ros_Sleep(100);
    }
}
