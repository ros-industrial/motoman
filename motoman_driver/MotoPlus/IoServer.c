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

#include "MotoROS.h"

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
	int sockOpt;

	printf("Starting new connection to the IO Server\r\n");

ATTEMPT_IO_CONNECTION:
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
		if (Ros_MotionServer_HasDataInQueue(controller)) //another client is actively controlling motion; likely controlling I/O too
		{
			puts("IO server already connected... not accepting last attempt.");
			mpClose(sd);
			return;
		}
		else
		{
			puts("IO server already connected... closing old connection.");
			Ros_IoServer_StopConnection(controller, 0); //close socket, cleanup resources, and delete tasks
			goto ATTEMPT_IO_CONNECTION;
		}
	}

	//This timeout detection takes two hours. So, it's not terribly useful. But, it still serves a purpose
	sockOpt = 1;
	mpSetsockopt(sd, SOL_SOCKET, SO_KEEPALIVE, (char*)&sockOpt, sizeof(sockOpt));

	if (controller->tidIoConnections[connectionIndex] == INVALID_TASK)
	{
		printf("Creating new task: tidIoConnections (connectionIndex = %d)\n", connectionIndex);
			
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
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 7);
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

int Ros_IoServer_GetExpectedByteSizeForMessageType(SimpleMsg* receiveMsg)
{
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;

	switch (receiveMsg->header.msgType)
	{
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
	case ROS_MSG_MOTO_READ_MREGISTER:
		expectedSize = minSize + sizeof(SmBodyMotoReadIOMRegister);
		break;
	case ROS_MSG_MOTO_WRITE_MREGISTER:
		expectedSize = minSize + sizeof(SmBodyMotoWriteIOMRegister);
		break;
	default: //invalid message type
		return -1;
	}
	return expectedSize;
}

//-----------------------------------------------------------------------
// Checks the type of message and processes it accordingly
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_IoServer_SimpleMsgProcess(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int ret = 0;
	int invalidSubcode = 0;

	switch (receiveMsg->header.msgType)
	{
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
	case ROS_MSG_MOTO_READ_MREGISTER:
		ret = Ros_IoServer_ReadIORegister(receiveMsg, replyMsg);
		break;

		//-----------------------
	case ROS_MSG_MOTO_WRITE_MREGISTER:
		ret = Ros_IoServer_WriteIORegister(receiveMsg, replyMsg);
		break;

		//-----------------------
	default:
		printf("Invalid message type: %d\n", receiveMsg->header.msgType);
		invalidSubcode = ROS_RESULT_INVALID_MSGTYPE;
		break;
	}

	// Check Invalid Case
	if (invalidSubcode != 0)
	{
		Ros_SimpleMsg_IoReply(ROS_RESULT_INVALID, invalidSubcode, replyMsg);
		ret = -1;
	}

	return ret;
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
	int partialMsgByteCount = 0;
	BOOL bSkipNetworkRecv = FALSE;

	while(!bDisconnect) //keep accepting messages until connection closes
	{
		Ros_Sleep(0); //allow other tasks to run, if needed
		
		if (!bSkipNetworkRecv)
		{
			//Receive message from the PC
			memset((&receiveMsg) + partialMsgByteCount, 0x00, sizeof(SimpleMsg) - partialMsgByteCount);
			byteSize = mpRecv(controller->sdIoConnections[connectionIndex], (char*)((&receiveMsg) + partialMsgByteCount), sizeof(SimpleMsg) - partialMsgByteCount, 0);
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
		if (byteSize >= minSize)
		{
			expectedSize = Ros_IoServer_GetExpectedByteSizeForMessageType(&receiveMsg);

			if (expectedSize == -1)
			{
				printf("Unknown Message Received (%d)\r\n", receiveMsg.header.msgType);
				Ros_SimpleMsg_IoReply(ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGTYPE, &replyMsg);
			}
			else if (byteSize >= expectedSize) // Check message size
			{
				// Process the simple message
				ret = Ros_IoServer_SimpleMsgProcess(&receiveMsg, &replyMsg);
				if (ret != OK) //error during processing
				{
					bDisconnect = TRUE;
				}
				else if (byteSize > expectedSize) // Received extra data in single message
				{
					// Preserve the remaining bytes and treat them as the start of a new message
					Db_Print("MessageReceived(%d bytes): expectedSize=%d, processing rest of bytes (%d, %d, %d)\r\n", byteSize, expectedSize, sizeof(receiveMsg), receiveMsg.body.jointTrajData.sequence, ((int*)((char*)&receiveMsg + expectedSize))[5]);
					partialMsgByteCount = byteSize - expectedSize;
					memmove(&receiveMsg, (char*)&receiveMsg + expectedSize, partialMsgByteCount);

					//Did I receive multiple full messages at once that all need to be processed before listening for new data?
					if (partialMsgByteCount >= minSize)
					{
						expectedSize = Ros_IoServer_GetExpectedByteSizeForMessageType(&receiveMsg);
						bSkipNetworkRecv = (partialMsgByteCount >= expectedSize); //does my modified receiveMsg buffer contain a full message to process?
					}
				}
				else // All good
					partialMsgByteCount = 0;
			}
			else // Not enough data to process the command
			{
				Db_Print("MessageReceived(%d bytes): expectedSize=%d\r\n", byteSize, expectedSize);
				Ros_SimpleMsg_IoReply(ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg);
			}
		}
		else // Didn't even receive a command ID
		{
			Db_Print("Unknown Data Received (%d bytes)\r\n", byteSize);
			Ros_SimpleMsg_IoReply(ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg);
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

int Ros_IoServer_ReadIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_INFO ioReadInfo;
	USHORT ioValue = 0;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoReadIOBitReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_READ_IO_BIT_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	
	if (Ros_IoServer_IsValidReadAddress(receiveMsg->body.readIOBit.ioAddress, IO_ACCESS_BIT))
	{
		ioReadInfo.ulAddr = receiveMsg->body.readIOBit.ioAddress;
		apiRet = mpReadIO(&ioReadInfo, &ioValue, QUANTITY_BIT);

		replyMsg->body.readIOBitReply.value = ioValue;
		replyMsg->body.readIOBitReply.resultCode = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_READ_API_ERROR;
		replyMsg->header.replyType = (apiRet == OK) ? ROS_REPLY_SUCCESS : ROS_REPLY_FAILURE;
	}
	else
	{
		replyMsg->body.readIOBitReply.value = 0;
		replyMsg->body.readIOBitReply.resultCode = IO_RESULT_READ_ADDRESS_INVALID;
		replyMsg->header.replyType = ROS_REPLY_FAILURE;
	}

	return OK; //keep connection alive regardless of any error code
}

int Ros_IoServer_ReadIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_INFO ioReadInfo[8];
	USHORT ioValue[8];
	int resultValue = 0;
	int i;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoReadIOGroupReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_READ_IO_GROUP_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;

	if (Ros_IoServer_IsValidReadAddress(receiveMsg->body.readIOGroup.ioAddress, IO_ACCESS_GROUP))
	{
		for (i = 0; i < 8; i += 1)
		{
			ioReadInfo[i].ulAddr = (receiveMsg->body.readIOGroup.ioAddress * 10) + i;
		}
		apiRet = mpReadIO(ioReadInfo, ioValue, QUANTITY_BYTE);

		resultValue = 0;
		for (i = 0; i < 8; i += 1)
		{
			resultValue |= (ioValue[i] << i);
		}

		replyMsg->body.readIOGroupReply.value = resultValue;
		replyMsg->body.readIOGroupReply.resultCode = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_READ_API_ERROR;
		replyMsg->header.replyType = (apiRet == OK) ? ROS_REPLY_SUCCESS : ROS_REPLY_FAILURE;
	}
	else
	{
		replyMsg->body.readIOGroupReply.value = 0;
		replyMsg->body.readIOGroupReply.resultCode = IO_RESULT_READ_ADDRESS_INVALID;
		replyMsg->header.replyType = ROS_REPLY_FAILURE;
	}

	return OK; //keep connection alive regardless of any error code
}

int Ros_IoServer_WriteIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{	
	int apiRet;
	MP_IO_DATA ioWriteData;
	BOOL bAddressOk;
	BOOL bValueOk;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoWriteIOBitReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_WRITE_IO_BIT_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	
	bAddressOk = Ros_IoServer_IsValidWriteAddress(receiveMsg->body.writeIOBit.ioAddress, IO_ACCESS_BIT);
	bValueOk = Ros_IoServer_IsValidWriteValue(receiveMsg->body.writeIOBit.ioValue, IO_ACCESS_BIT);

	if (bAddressOk && bValueOk)
	{
		ioWriteData.ulAddr = receiveMsg->body.writeIOBit.ioAddress;
		ioWriteData.ulValue = receiveMsg->body.writeIOBit.ioValue;
		apiRet = mpWriteIO(&ioWriteData, QUANTITY_BIT);

		replyMsg->body.writeIOBitReply.resultCode = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_WRITE_API_ERROR;
		replyMsg->header.replyType = (apiRet == OK) ? ROS_REPLY_SUCCESS : ROS_REPLY_FAILURE;
	}
	else
	{
		replyMsg->header.replyType = ROS_REPLY_FAILURE;

		if (!bAddressOk)
			replyMsg->body.writeIOBitReply.resultCode = IO_RESULT_WRITE_ADDRESS_INVALID;
		else if (!bValueOk)
			replyMsg->body.writeIOBitReply.resultCode = IO_RESULT_WRITE_VALUE_INVALID;
	}

	return OK; //keep connection alive regardless of any error code
}

int Ros_IoServer_WriteIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_DATA ioWriteData[8];
	int i;
	BOOL bAddressOk;
	BOOL bValueOk;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoWriteIOGroupReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_WRITE_IO_GROUP_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;

	bAddressOk = Ros_IoServer_IsValidWriteAddress(receiveMsg->body.writeIOGroup.ioAddress, IO_ACCESS_GROUP);
	bValueOk = Ros_IoServer_IsValidWriteValue(receiveMsg->body.writeIOGroup.ioValue, IO_ACCESS_GROUP);

	if (bAddressOk && bValueOk)
	{
		for (i = 0; i < QUANTITY_BYTE; i += 1)
		{
			ioWriteData[i].ulAddr = (receiveMsg->body.writeIOGroup.ioAddress * 10) + i;
			ioWriteData[i].ulValue = (receiveMsg->body.writeIOGroup.ioValue & (1 << i)) >> i;
		}
		apiRet = mpWriteIO(ioWriteData, QUANTITY_BYTE);

		replyMsg->body.writeIOGroupReply.resultCode = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_WRITE_API_ERROR;
		replyMsg->header.replyType = (apiRet == OK) ? ROS_REPLY_SUCCESS : ROS_REPLY_FAILURE;
	}
	else
	{
		replyMsg->header.replyType = ROS_REPLY_FAILURE;

		if (!bAddressOk)
			replyMsg->body.writeIOGroupReply.resultCode = IO_RESULT_WRITE_ADDRESS_INVALID;
		else if (!bValueOk)
			replyMsg->body.writeIOGroupReply.resultCode = IO_RESULT_WRITE_VALUE_INVALID;
	}

	return OK; //keep connection alive regardless of any error code
}

int Ros_IoServer_ReadIORegister(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_INFO ioReadInfo;
	USHORT ioValue = 0;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoReadIOMRegisterReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_READ_MREGISTER;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;

	if (receiveMsg->body.readRegister.registerNumber < 1000000)
		receiveMsg->body.readRegister.registerNumber += 1000000;

	if (Ros_IoServer_IsValidReadAddress(receiveMsg->body.readRegister.registerNumber, IO_ACCESS_REGISTER))
	{
		ioReadInfo.ulAddr = receiveMsg->body.readRegister.registerNumber;
		apiRet = mpReadIO(&ioReadInfo, &ioValue, QUANTITY_BIT);

		replyMsg->body.readRegisterReply.value = ioValue;
		replyMsg->body.readRegisterReply.resultCode = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_READ_API_ERROR;
		replyMsg->header.replyType = (apiRet == OK) ? ROS_REPLY_SUCCESS : ROS_REPLY_FAILURE;
	}
	else
	{
		replyMsg->body.readRegisterReply.value = 0;
		replyMsg->body.readRegisterReply.resultCode = IO_RESULT_READ_ADDRESS_INVALID;
		replyMsg->header.replyType = ROS_REPLY_FAILURE;
	}

	return OK; //keep connection alive regardless of any error code
}

int Ros_IoServer_WriteIORegister(SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int apiRet;
	MP_IO_DATA ioWriteData;
	BOOL bAddressOk;
	BOOL bValueOk;

	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoWriteIOMRegisterReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_WRITE_MREGISTER;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;

	if (receiveMsg->body.writeRegister.registerNumber < 1000000)
		receiveMsg->body.writeRegister.registerNumber += 1000000;

	bAddressOk = Ros_IoServer_IsValidWriteAddress(receiveMsg->body.writeRegister.registerNumber, IO_ACCESS_REGISTER);
	bValueOk = Ros_IoServer_IsValidWriteValue(receiveMsg->body.writeRegister.value, IO_ACCESS_REGISTER);

	if (bAddressOk && bValueOk)
	{
		ioWriteData.ulAddr = receiveMsg->body.writeRegister.registerNumber;
		ioWriteData.ulValue = receiveMsg->body.writeRegister.value;
		apiRet = mpWriteIO(&ioWriteData, QUANTITY_BIT);

		replyMsg->body.writeRegisterReply.resultCode = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_WRITE_API_ERROR;
		replyMsg->header.replyType = (apiRet == OK) ? ROS_REPLY_SUCCESS : ROS_REPLY_FAILURE;
	}
	else
	{
		replyMsg->header.replyType = ROS_REPLY_FAILURE;

		if (!bAddressOk)
			replyMsg->body.writeRegisterReply.resultCode = IO_RESULT_WRITE_ADDRESS_INVALID;
		else if (!bValueOk)
			replyMsg->body.writeRegisterReply.resultCode = IO_RESULT_WRITE_VALUE_INVALID;
	}

	return OK; //keep connection alive regardless of any error code
}

BOOL Ros_IoServer_IsValidReadAddress(UINT32 address, IoAccessSize size)
{
	int mod = 0;

	if (size == IO_ACCESS_GROUP)
		address *= 10;

	mod = address % 10;

	//last digit cannot end in 8 or 9, unless it is an M Register
	if (size != IO_ACCESS_REGISTER && mod > 7)
		return FALSE;
	
	if ((address >= GENERALINMIN && address <= GENERALINMAX) ||
		(address >= GENERALOUTMIN && address <= GENERALOUTMAX) ||
		(address >= EXTERNALINMIN && address <= EXTERNALINMAX) ||
		(address >= NETWORKINMIN && address <= NETWORKINMAX) ||
		(address >= NETWORKOUTMIN && address <= NETWORKOUTMAX) ||
		(address >= EXTERNALOUTMIN && address <= EXTERNALOUTMAX) ||
		(address >= SPECIFICINMIN && address <= SPECIFICINMAX) ||
		(address >= SPECIFICOUTMIN && address <= SPECIFICOUTMAX) ||
		(address >= IFPANELMIN && address <= IFPANELMAX) ||
		(address >= AUXRELAYMIN && address <= AUXRELAYMAX) ||
		(address >= CONTROLSTATUSMIN && address <= CONTROLSTATUSMAX) ||
		(address >= PSEUDOINPUTMIN && address <= PSEUDOINPUTMAX) ||
		(address >= REGISTERMIN && address <= REGISTERMAX_READ))
	{
		return TRUE;
	}
	else
		return FALSE;
}

BOOL Ros_IoServer_IsValidWriteAddress(UINT32 address, IoAccessSize size)
{
	int mod = 0;

	if (size == IO_ACCESS_GROUP)
		address *= 10;

	mod = address % 10;

	//last digit cannot end in 8 or 9, unless it is an M Register
	if (size != IO_ACCESS_REGISTER && mod > 7)
		return FALSE;

	if ((address >= GENERALOUTMIN && address <= GENERALOUTMAX) ||
		(address >= NETWORKINMIN && address <= NETWORKINMAX) ||
		(address >= IFPANELMIN && address <= IFPANELMAX) ||
		(address >= REGISTERMIN && address <= REGISTERMAX_WRITE))
	{
		return TRUE;
	}
	else
		return FALSE;
}

BOOL Ros_IoServer_IsValidWriteValue(UINT32 value, IoAccessSize size)
{
	if (size == IO_ACCESS_REGISTER && value > 0xFFFF)
		return FALSE;
	
	if (size == IO_ACCESS_GROUP && value > 0xFF)
		return FALSE;

	if (size == IO_ACCESS_BIT && value > 1)
		return FALSE;

	return TRUE;
}