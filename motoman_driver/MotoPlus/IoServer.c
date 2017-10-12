// IoServer.c
//
// History:
// 10/12/2017: Refactor IO server into separate task
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Yaskawa America, Inc.
* Copyright (c) 2017, Delft University of Technology
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

#include "MotoPlus.h"
#include "SimpleMessage.h"
#include "Controller.h"
#include "IoServer.h"

//-----------------------
// Function Declarations
//-----------------------
// Main Task: 
void Ros_IoServer_StartNewConnection(Controller* controller, int sd);
void Ros_IoServer_StopConnection(Controller* controller, int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_IoServer_WaitForSimpleMsg(Controller* controller, int connectionIndex);
BOOL Ros_IoServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, int byteSize, SimpleMsg* replyMsg);

// IO functions:
int Ros_IoServer_ReadIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_IoServer_WriteIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_IoServer_ReadIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_IoServer_WriteIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);


//-----------------------
// Function implementation
//-----------------------

//-----------------------------------------------------------------------
// Start the tasks for a new io server connection:
// - WaitForSimpleMsg: Task that waits to receive new SimpleMessage
//-----------------------------------------------------------------------
void Ros_IoServer_StartNewConnection(Controller* controller, int sd)
{
	int connectionIndex;

	//look for next available connection slot
	for (connectionIndex = 0; connectionIndex < MAX_IO_CONNECTIONS; connectionIndex++)
	{
		if (controller->sdIoConnections[connectionIndex] == INVALID_SOCKET)
		{
			controller->sdIoConnections[connectionIndex] = sd;
			break;
		}
	}
	
	if (connectionIndex == MAX_IO_CONNECTIONS)
	{
		puts("IO server already connected... not accepting last attempt.");
		mpClose(sd);
		return;
	}

	if (controller->tidIoConnections[connectionIndex] == INVALID_TASK)
	{
#ifdef DEBUG
		printf("Creating new task: tidIoConnections (connectionIndex = %d)\n", connectionIndex);
#endif
			
		//start new task for this specific connection
		controller->tidIoConnections[connectionIndex] = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
											(FUNCPTR)Ros_IoServer_WaitForSimpleMsg,
											(int)controller, connectionIndex, 0, 0, 0, 0, 0, 0, 0, 0);
	
		if (controller->tidIoConnections[connectionIndex] != ERROR)
		{
			//set feedback signal indicating success
			Ros_Controller_SetIOState(IO_FEEDBACK_IOSERVERCONNECTED, TRUE);
		}
		else
		{
			puts("Could not create new task in the IO server.  Check robot parameters.");
			mpClose(sd);
			controller->sdIoConnections[connectionIndex] = INVALID_SOCKET;
			controller->tidIoConnections[connectionIndex] = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			return;
		}
	}
}


//-----------------------------------------------------------------------
// Close a connection along with all its associated task
//-----------------------------------------------------------------------
void Ros_IoServer_StopConnection(Controller* controller, int connectionIndex)
{   
	int tid;
	
	printf("Closing IO Server Connection\r\n");
	
	//close this connection
	mpClose(controller->sdIoConnections[connectionIndex]);
	//mark connection as invalid
	controller->sdIoConnections[connectionIndex] = INVALID_SOCKET;

	// Stop message receiption task
	tid = controller->tidIoConnections[connectionIndex];
	controller->tidIoConnections[connectionIndex] = INVALID_TASK;
	printf("IO Server Connection Closed\r\n");
	
	mpDeleteTask(tid);
}



//-----------------------------------------------------------------------
// Task that waits to receive new SimpleMessage and then processes it
//-----------------------------------------------------------------------
void Ros_IoServer_WaitForSimpleMsg(Controller* controller, int connectionIndex)
{
	SimpleMsg receiveMsg;
	SimpleMsg replyMsg;
	int byteSize = 0, byteSizeResponse = 0;
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;
	int ret = 0;
	BOOL bDisconnect = FALSE;
	BOOL bHasPreviousData = FALSE; // if true, then receiveMsg is already filled with valid data
	BOOL bInvalidMsgType = FALSE;

	while(!bDisconnect) //keep accepting messages until connection closes
	{
		Ros_Sleep(IO_SERVER_MIN_PERIOD);
		
		if (!bHasPreviousData)
		{
			//Receive message from the PC
			memset(&receiveMsg, 0x00, sizeof(receiveMsg));
			byteSize = mpRecv(controller->sdIoConnections[connectionIndex], (char*)(&receiveMsg), sizeof(receiveMsg), 0);
			if (byteSize <= 0)
				break; //end connection
		}
		
		bInvalidMsgType = FALSE;

		// Determine the expected size of the message
		expectedSize = -1;
		if(byteSize >= minSize)
		{
			switch(receiveMsg.header.msgType)
			{
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
					if (byteSize >= (minSize + sizeof(int))) //make sure I can at least get to [numberOfGroups] field
					{
						expectedSize = minSize + (sizeof(int) * 2);
						expectedSize += (sizeof(SmBodyJointTrajPtExData) * receiveMsg.body.jointTrajDataEx.numberOfValidGroups); //check the number of groups to determine size of data
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
				default:
					bInvalidMsgType = TRUE;
					break;
			}
		}

		bHasPreviousData = FALSE;
		// Check message size
		if(byteSize >= expectedSize && expectedSize <= sizeof(SimpleMsg))
		{
			// Process the simple message
			ret = Ros_IoServer_SimpleMsgProcess(controller, &receiveMsg, expectedSize, &replyMsg);
			if(ret == 1) 
			{
				bDisconnect = TRUE;
			}
			else if( byteSize > expectedSize ) 
			{
				printf("MessageReceived(%d bytes): expectedSize=%d, processing rest of bytes (%d, %d, %d)\r\n", byteSize,  expectedSize, sizeof(receiveMsg), receiveMsg.body.jointTrajData.sequence, ((int*)((char*)&receiveMsg +  expectedSize))[5]);
				memmove(&receiveMsg, (char*)&receiveMsg + expectedSize, byteSize-expectedSize);
				byteSize -= expectedSize;
				bHasPreviousData = TRUE;
			}
		}
		else if (bInvalidMsgType)
		{
			printf("Unknown Message Received(%d)\r\n", receiveMsg.header.msgType);
			Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGTYPE, &replyMsg, 0);			
		}
		else
		{
			printf("MessageReceived(%d bytes): expectedSize=%d\r\n", byteSize,  expectedSize);
			Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
			// Note: If messages are being combine together because of network transmission protocol
			// we may need to add code to store unused portion of the received buff that would be part of the next message
		}

		//Send reply message
		byteSizeResponse = mpSend(controller->sdIoConnections[connectionIndex], (char*)(&replyMsg), replyMsg.prefix.length + sizeof(SmPrefix), 0);        
		if (byteSizeResponse <= 0)
			break;	// Close the connection
	}
	
	Ros_Sleep(50);	// Just in case other associated task need time to clean-up.  Don't if necessary... but it doesn't hurt
	
	//close this connection
	Ros_IoServer_StopConnection(controller, connectionIndex);
}


//-----------------------------------------------------------------------
// Checks the type of message and processes it accordingly
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_IoServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, 
										int byteSize, SimpleMsg* replyMsg)
{
	int ret = 0;
	int expectedBytes = sizeof(SmPrefix) + sizeof(SmHeader);
	int invalidSubcode = 0;
	
	//printf("In SimpleMsgProcess\r\n");
	
	switch(receiveMsg->header.msgType)
	{
	//-----------------------
	case ROS_MSG_MOTO_READ_IO_BIT:
		// Check that the appropriate message size was received
		expectedBytes += sizeof(SmBodyMotoReadIOBit);
		if(expectedBytes == byteSize)
			ret = Ros_IoServer_ReadIOBit(receiveMsg, replyMsg);
		else
			invalidSubcode = ROS_RESULT_INVALID_MSGSIZE;
		break;

	//-----------------------
	case ROS_MSG_MOTO_WRITE_IO_BIT:
		// Check that the appropriate message size was received
		expectedBytes += sizeof(SmBodyMotoWriteIOBit);
		if(expectedBytes == byteSize)
			ret = Ros_IoServer_WriteIOBit(receiveMsg, replyMsg);
		else
			invalidSubcode = ROS_RESULT_INVALID_MSGSIZE;
		break;


	//-----------------------
	case ROS_MSG_MOTO_READ_IO_GROUP:
		// Check that the appropriate message size was received
		expectedBytes += sizeof(SmBodyMotoReadIOGroup);
		if (expectedBytes == byteSize)
			ret = Ros_IoServer_ReadIOGroup(receiveMsg, replyMsg);
		else
			invalidSubcode = ROS_RESULT_INVALID_MSGSIZE;
		break;

	//-----------------------
	case ROS_MSG_MOTO_WRITE_IO_GROUP:
		// Check that the appropriate message size was received
		expectedBytes += sizeof(SmBodyMotoWriteIOGroup);
		if (expectedBytes == byteSize)
			ret = Ros_IoServer_WriteIOGroup(receiveMsg, replyMsg);
		else
			invalidSubcode = ROS_RESULT_INVALID_MSGSIZE;
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
		ret = -1;
	}
		
	return ret;
}

int Ros_IoServer_ReadIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_INFO ioReadInfo;
	USHORT ioValue;
	int resultCode;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoReadIOBitReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_READ_IO_BIT_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	
	ioReadInfo.ulAddr = receiveMsg->body.readIOBit.ioAddress;
	apiRet = mpReadIO(&ioReadInfo, &ioValue, 1);

	if (apiRet == OK)
		resultCode = ROS_REPLY_SUCCESS;
	else
		resultCode = ROS_REPLY_FAILURE;

	replyMsg->body.readIOBitReply.value = ioValue;
	replyMsg->body.readIOBitReply.resultCode = resultCode;
	replyMsg->header.replyType = (SmReplyType)resultCode;
	return OK;
}

int Ros_IoServer_ReadIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_INFO ioReadInfo[8];
	USHORT ioValue[8];
	int resultCode;
	int resultValue = 0;
	int i;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoReadIOGroupReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_READ_IO_GROUP_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;

	for (i = 0; i < 8; i += 1)
	{
		ioReadInfo[i].ulAddr = (receiveMsg->body.readIOGroup.ioAddress * 10) + i;
	}
	apiRet = mpReadIO(ioReadInfo, ioValue, 8);

	resultValue = 0;
	for (i = 0; i < 8; i += 1)
	{
		resultValue |= (ioValue[i] << i);
	}

	if (apiRet == OK)
		resultCode = ROS_REPLY_SUCCESS;
	else
		resultCode = ROS_REPLY_FAILURE;

	replyMsg->body.readIOGroupReply.value = resultValue;
	replyMsg->body.readIOGroupReply.resultCode = resultCode;
	replyMsg->header.replyType = (SmReplyType)resultCode;
	return OK;
}

int Ros_IoServer_WriteIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{	
	int apiRet;
	MP_IO_DATA ioWriteData;
	int resultCode;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoWriteIOBitReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_WRITE_IO_BIT_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	
	ioWriteData.ulAddr = receiveMsg->body.writeIOBit.ioAddress;
	ioWriteData.ulValue = receiveMsg->body.writeIOBit.ioValue;
	apiRet = mpWriteIO(&ioWriteData, 1);

	if (apiRet == OK)
		resultCode = ROS_REPLY_SUCCESS;
	else
		resultCode = ROS_REPLY_FAILURE;

	replyMsg->body.writeIOBitReply.resultCode = resultCode;
	replyMsg->header.replyType = (SmReplyType)resultCode;
	return OK;
}

int Ros_IoServer_WriteIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_DATA ioWriteData[8];
	int resultCode;
	int i;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoWriteIOGroupReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_WRITE_IO_GROUP_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;

	for (i = 0; i < 8; i += 1)
	{
		ioWriteData[i].ulAddr = (receiveMsg->body.writeIOGroup.ioAddress * 10) + i;
		ioWriteData[i].ulValue = (receiveMsg->body.writeIOGroup.ioValue & (1 << i)) >> i;
	}
	apiRet = mpWriteIO(ioWriteData, 8);

	if (apiRet == OK)
		resultCode = ROS_REPLY_SUCCESS;
	else
		resultCode = ROS_REPLY_FAILURE;

	replyMsg->body.writeIOGroupReply.resultCode = resultCode;
	replyMsg->header.replyType = (SmReplyType)resultCode;
	return OK;
}
