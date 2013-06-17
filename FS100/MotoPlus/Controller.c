// Controller.c
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

#include "MotoPlus.h"
#include "ParameterExtraction.h"
#include "CtrlGroup.h"
#include "SimpleMessage.h"
#include "Controller.h"
#include "MotionServer.h"
#include "StateServer.h"

extern STATUS setsockopt
    (
    int    s,                 /* target socket */
    int    level,             /* protocol level of option */
    int    optname,           /* option name */
    char * optval,            /* pointer to option value */
    int    optlen             /* option length */
    );

//-----------------------
// Function Declarations
//-----------------------
void Ros_Controller_Init(Controller* controller);
BOOL Ros_Controller_IsValidGroupNo(Controller* controller, int groupNo);
int Ros_Controller_OpenSocket(int tcpPort);
void Ros_Controller_ConnectionServer_Start(Controller* controller);
// Status related
void Ros_Controller_StatusInit(Controller* controller);
BOOL Ros_Controller_IsAlarm(Controller* controller);
BOOL Ros_Controller_IsError(Controller* controller);
BOOL Ros_Controller_IsPlay(Controller* controller);
BOOL Ros_Controller_IsTeach(Controller* controller);
BOOL Ros_Controller_IsRemote(Controller* controller);
BOOL Ros_Controller_IsOperating(Controller* controller);
BOOL Ros_Controller_IsHold(Controller* controller);
BOOL Ros_Controller_IsServoOn(Controller* controller);
BOOL Ros_Controller_IsEStop(Controller* controller);
BOOL Ros_Controller_IsWaitingRos(Controller* controller);
int Ros_Controller_GetNotReadySubcode(Controller* controller);
int Ros_Controller_StatusToMsg(Controller* controller, SimpleMsg* sendMsg);
BOOL Ros_Controller_StatusRead(Controller* controller, USHORT ioStatus[IOSTATUS_MAX]);
BOOL Ros_Controller_StatusUpdate(Controller* controller);
// Wrapper around MPFunctions
BOOL Ros_Controller_GetIOState(ULONG signal);
void Ros_Controller_SetIOState(ULONG signal, BOOL status);
int Ros_Controller_GetAlarmCode(Controller* controller);
void Ros_Controller_ErrNo_ToString(int errNo, char errMsg[ERROR_MSG_MAX_SIZE], int errMsgSize);

//-----------------------
// Function implementation
//-----------------------

//-------------------------------------------------------------------
// Initialize the controller structure
// This should be done before the controller is used for anything
//------------------------------------------------------------------- 
void Ros_Controller_Init(Controller* controller)
{
	int grpNo;
	int i;
	
	// Get the interpolation clock
	GP_getInterpolationPeriod(&controller->interpolPeriod);
	
	// Check for each group
	for(grpNo=0; grpNo<MP_GRP_NUM; grpNo++)
	{
		// Determine if specific group exists and allocate memory for it
		controller->ctrlGroups[grpNo] = Ros_CtrlGroup_Create(grpNo, controller->interpolPeriod);
		
		// Check if group was created and update the number of group accordingly
		if(controller->ctrlGroups[grpNo] != NULL)
		{
			controller->numGroup = grpNo + 1;
		}
	}
	
	// Initialize Thread ID and Socket to invalid value
	controller->tidConnectionSrv = INVALID_TASK;

    controller->tidStateSendState = INVALID_TASK;
    for (i = 0; i < MAX_STATE_CONNECTIONS; i++)
    {
    	controller->sdStateConnections[i] = INVALID_SOCKET;
    }

	for (i = 0; i < MAX_MOTION_CONNECTIONS; i++)
    {
    	controller->sdMotionConnections[i] = INVALID_SOCKET;
    	controller->tidMotionConnections[i] = INVALID_TASK;
    }
	controller->tidIncMoveThread = INVALID_TASK;

	
	// Other variables
	controller->bRobotJobReady = FALSE;
	controller->bRobotJobReadyRaised = FALSE;
	controller->bStopMotion = FALSE;
	Ros_Controller_StatusInit(controller);
	Ros_Controller_StatusRead(controller, controller->ioStatus);
}


//-------------------------------------------------------------------
// Check the number of inc_move currently in the specified queue
//-------------------------------------------------------------------
BOOL Ros_Controller_IsValidGroupNo(Controller* controller, int groupNo)
{
	if((groupNo >= 0) && (groupNo < controller->numGroup))
		return TRUE;
	else
	{
		printf("ERROR: Attempt to access invalid Group No. (%d) \r\n", groupNo);
		return FALSE;
	}
}


//-------------------------------------------------------------------
// Open a socket to listen for incomming connection on specified port
// return: <0  : Error
// 		   >=0 : socket descriptor
//-------------------------------------------------------------------
int Ros_Controller_OpenSocket(int tcpPort)
{
	int sd;  // socket descriptor
	struct sockaddr_in	serverSockAddr;
	int ret;

	// Open the socket
   	sd = mpSocket(AF_INET, SOCK_STREAM, 0);
   	if (sd < 0)
       	return -1;
    
    // Set structure
   	memset(&serverSockAddr, 0, sizeof(struct sockaddr_in));
   	serverSockAddr.sin_family = AF_INET;
   	serverSockAddr.sin_addr.s_addr = INADDR_ANY;
	serverSockAddr.sin_port = mpHtons(tcpPort);

	//bind to network interface
   	ret = mpBind(sd, (struct sockaddr *)&serverSockAddr, sizeof(struct sockaddr_in)); 
   	if (ret < 0)
       	goto closeSockHandle;
       	
	//prepare to accept connections
   	ret = mpListen(sd, SOMAXCONN);
   	if (ret < 0)
       	goto closeSockHandle;
       	
    return  sd;
     
closeSockHandle:	
	printf("Error in Ros_Controller_OpenSocket\r\n");

	if(sd >= 0)
    	mpClose(sd);

	return -2;       	
}


//-----------------------------------------------------------------------
// Main Connection Server Task that listens for new connections
//-----------------------------------------------------------------------
void Ros_Controller_ConnectionServer_Start(Controller* controller)
{
    int		sdMotionServer = INVALID_SOCKET;
    int		sdStateServer = INVALID_SOCKET;
    fd_set  fds;
    struct  timeval tv;
    int     sdAccepted = INVALID_SOCKET;;
    struct  sockaddr_in     clientSockAddr;
    int     sizeofSockAddr;
	int 	useNoDelay = 1;

	//Set feedback signal (Application is installed and running)
	Ros_Controller_SetIOState(IO_FEEDBACK_CONNECTSERVERRUNNING, TRUE);

    printf("Controller connection server running\r\n");

	//New sockets for server listening to multiple port
	sdMotionServer = Ros_Controller_OpenSocket(TCP_PORT_MOTION);
	if(sdMotionServer < 0)
		goto closeSockHandle;
	
	sdStateServer = Ros_Controller_OpenSocket(TCP_PORT_STATE);
	if(sdStateServer < 0)
		goto closeSockHandle;
	
	tv.tv_sec = 0;
	tv.tv_usec = 0;

    FOREVER //Continue to accept multiple connections forever
    {
		FD_ZERO(&fds);
		FD_SET(sdMotionServer, &fds); 
		FD_SET(sdStateServer, &fds); 
		
		if(mpSelect(sdStateServer+1, &fds, NULL, NULL, NULL) > 0)
		{
	        memset(&clientSockAddr, 0, sizeof(clientSockAddr));
	        sizeofSockAddr = sizeof(clientSockAddr);
			
			//Accept the connection and get a new socket handle
			if(FD_ISSET(sdMotionServer, &fds))
	        	sdAccepted = mpAccept(sdMotionServer, (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
			else if(FD_ISSET(sdStateServer, &fds))
				sdAccepted = mpAccept(sdStateServer, (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
			else
				continue;
				
	        if (sdAccepted < 0)
	            break;
	        
	        printf("Accepted connection from client PC\r\n");
	          
    		STATUS s = setsockopt(sdAccepted, IPPROTO_TCP, TCP_NODELAY, (char*)&useNoDelay, sizeof (int));
    		if( OK != s )
    		{
      			printf("Failed to set TCP_NODELAY.\r\n");
    		}
	        
			if(FD_ISSET(sdMotionServer, &fds))
				Ros_MotionServer_StartNewConnection(controller, sdAccepted);
			else if(FD_ISSET(sdStateServer, &fds))
				Ros_StateServer_StartNewConnection(controller, sdAccepted);
			else
				mpClose(sdAccepted); 			        
        }
	}
	
closeSockHandle:	
	printf("Error!?... Connection Server is aborting.  Reboot the controller.\r\n");

	if(sdMotionServer >= 0)
    	mpClose(sdMotionServer);
 	if(sdStateServer >= 0)
    	mpClose(sdStateServer);
   
    //disable feedback signal
	Ros_Controller_SetIOState(IO_FEEDBACK_CONNECTSERVERRUNNING, FALSE);
}



/**** Controller Status functions ****/

//-------------------------------------------------------------------
// Initialize list of Specific Input to keep track of
//-------------------------------------------------------------------
void Ros_Controller_StatusInit(Controller* controller)
{
	controller->ioStatusAddr[IOSTATUS_ALARM_MAJOR].ulAddr = 50010;		// Alarm
	controller->ioStatusAddr[IOSTATUS_ALARM_MINOR].ulAddr = 50011;		// Alarm
	controller->ioStatusAddr[IOSTATUS_ALARM_SYSTEM].ulAddr = 50012;		// Alarm
	controller->ioStatusAddr[IOSTATUS_ALARM_USER].ulAddr = 50013;		// Alarm
	controller->ioStatusAddr[IOSTATUS_ERROR].ulAddr = 50014;			// Error
	controller->ioStatusAddr[IOSTATUS_PLAY].ulAddr = 50054;				// Play
	controller->ioStatusAddr[IOSTATUS_TEACH].ulAddr = 50053;			// Teach
	controller->ioStatusAddr[IOSTATUS_REMOTE].ulAddr = 50056;			// Remote
	controller->ioStatusAddr[IOSTATUS_OPERATING].ulAddr = 50070;		// Operating
	controller->ioStatusAddr[IOSTATUS_HOLD].ulAddr = 50071;				// Hold
	controller->ioStatusAddr[IOSTATUS_SERVO].ulAddr = 50073;   			// Servo ON
	controller->ioStatusAddr[IOSTATUS_ESTOP_EX].ulAddr = 80025;   		// External E-Stop
	controller->ioStatusAddr[IOSTATUS_ESTOP_PP].ulAddr = 80026;   		// Pendant E-Stop
	controller->ioStatusAddr[IOSTATUS_ESTOP_CTRL].ulAddr = 80027;   	// Controller E-Stop
	controller->ioStatusAddr[IOSTATUS_WAITING_ROS].ulAddr = IO_FEEDBACK_WAITING_MP_INCMOVE; // Job input signaling ready for external motion
	controller->alarmCode = 0;
}


BOOL Ros_Controller_IsAlarm(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_ALARM_MAJOR]!=0) 
		|| (controller->ioStatus[IOSTATUS_ALARM_MINOR]!=0)
		|| (controller->ioStatus[IOSTATUS_ALARM_SYSTEM]!=0)
		|| (controller->ioStatus[IOSTATUS_ALARM_USER]!=0) );
}

BOOL Ros_Controller_IsError(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_ERROR]!=0));
}

BOOL Ros_Controller_IsPlay(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_PLAY]!=0));
}

BOOL Ros_Controller_IsTeach(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_TEACH]!=0));
}

BOOL Ros_Controller_IsRemote(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_REMOTE]!=0));
}

BOOL Ros_Controller_IsOperating(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_OPERATING]!=0));
}

BOOL Ros_Controller_IsHold(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_HOLD]!=0));
}

BOOL Ros_Controller_IsServoOn(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_SERVO]!=0));
}

BOOL Ros_Controller_IsEStop(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_ESTOP_EX]==0) 
		|| (controller->ioStatus[IOSTATUS_ESTOP_PP]==0)
		|| (controller->ioStatus[IOSTATUS_ESTOP_CTRL]==0) );
}

BOOL Ros_Controller_IsWaitingRos(Controller* controller)
{
	return ((controller->ioStatus[IOSTATUS_WAITING_ROS]!=0));
}

int Ros_Controller_GetNotReadySubcode(Controller* controller)
{
	// Check alarm
	if(Ros_Controller_IsAlarm(controller))
		return ROS_RESULT_NOT_READY_ALARM;
	
	// Check error
	if(Ros_Controller_IsError(controller))
		return ROS_RESULT_NOT_READY_ERROR;
	
	// Check e-stop
	if(Ros_Controller_IsEStop(controller))
		return ROS_RESULT_NOT_READY_ESTOP;

	// Check play mode
	if(!Ros_Controller_IsPlay(controller))
		return ROS_RESULT_NOT_READY_NOT_PLAY;
	
	// Check remote
	if(!Ros_Controller_IsRemote(controller))
		return ROS_RESULT_NOT_READY_NOT_REMOTE;
	
	// Check servo power
	if(!Ros_Controller_IsServoOn(controller))
		return ROS_RESULT_NOT_READY_SERVO_OFF;

	// Check hold
	if(Ros_Controller_IsHold(controller))
		return ROS_RESULT_NOT_READY_HOLD;

	// Check operating
	if(!Ros_Controller_IsOperating(controller))
		return ROS_RESULT_NOT_READY_NOT_STARTED;
	
	// Check ready I/O signal (should confirm wait)
	if(!Ros_Controller_IsWaitingRos(controller))
		return ROS_RESULT_NOT_READY_WAITING_ROS;
		
	return ROS_RESULT_NOT_READY_UNSPECIFIED;
}


// Creates a simple message of type: ROS_MSG_ROBOT_STATUS = 13
// Simple message containing the current state of the controller
int Ros_Controller_StatusToMsg(Controller* controller, SimpleMsg* sendMsg)
{
	//initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyRobotStatus);
	
	// set header information
	sendMsg->header.msgType = ROS_MSG_ROBOT_STATUS;
	sendMsg->header.commType = ROS_COMM_TOPIC;
	sendMsg->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	sendMsg->body.robotStatus.drives_powered = (int)(Ros_Controller_IsServoOn(controller));
	sendMsg->body.robotStatus.e_stopped = (int)(Ros_Controller_IsEStop(controller));
	sendMsg->body.robotStatus.error_code = controller->alarmCode;
	sendMsg->body.robotStatus.in_error = (int)Ros_Controller_IsAlarm(controller);
	sendMsg->body.robotStatus.in_motion = (int)(Ros_MotionServer_HasDataInQueue(controller));
	if(Ros_Controller_IsPlay(controller))
		sendMsg->body.robotStatus.mode = 2;
	else
		sendMsg->body.robotStatus.mode = 1;
	sendMsg->body.robotStatus.motion_possible = (int)(controller->bRobotJobReady);
	
	return(sendMsg->prefix.length + sizeof(SmPrefix));
}

//-------------------------------------------------------------------
// Get I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_StatusRead(Controller* controller, USHORT ioStatus[IOSTATUS_MAX])
{
	return (mpReadIO(controller->ioStatusAddr, ioStatus, IOSTATUS_MAX) == 0);
}

//-------------------------------------------------------------------
// Update I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_StatusUpdate(Controller* controller)
{
	USHORT ioStatus[IOSTATUS_MAX];
	int i;
	
	if(Ros_Controller_StatusRead(controller, ioStatus))
	{
		// Check for change of state and potentially react to the change
		for(i=0; i<IOSTATUS_MAX; i++)
		{
			if(controller->ioStatus[i] != ioStatus[i])
			{
				//printf("Change of ioStatus[%d]\r\n", i);
				
				controller->ioStatus[i] = ioStatus[i];
				switch(i)
				{
					case IOSTATUS_ALARM_MAJOR: // alarm
					case IOSTATUS_ALARM_MINOR: // alarm
					case IOSTATUS_ALARM_SYSTEM: // alarm
					case IOSTATUS_ALARM_USER: // alarm
					{
						if(ioStatus[i] == 0)
							controller->alarmCode = 0;
						else
							controller->alarmCode = Ros_Controller_GetAlarmCode(controller);
					}
					//case IOSTATUS_ERROR: // error
					//		if(ioStatus[i] != 0)
					//		{
					//			// Take action for alarm/error handling
					//		}
					//	break;
					case IOSTATUS_REMOTE: // remote
					case IOSTATUS_OPERATING: // operating
					case IOSTATUS_WAITING_ROS: // Job input signaling ready for external motion
					{
						if(ioStatus[i] == 0)  // signal turned OFF
						{
							// Job execution stopped take action
							controller->bRobotJobReady = FALSE;
							controller->bRobotJobReadyRaised = FALSE;
							Ros_MotionServer_ClearQ_All(controller);
						}
						else // signal turned ON
						{
							if(i==IOSTATUS_WAITING_ROS)
								controller->bRobotJobReadyRaised = TRUE;
								
							if(controller->bRobotJobReadyRaised
							 	&& (Ros_Controller_IsOperating(controller))
								&& (Ros_Controller_IsRemote(controller)) )
							{
								controller->bRobotJobReady = TRUE;
								printf("Robot job is ready for ROS commands.\r\n");
							}
						}
						break;
					}
				}
			}
		}
		return TRUE;
	}
	return FALSE;
}



/**** Wrappers on MP standard function ****/

//-------------------------------------------------------------------
// Get I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_GetIOState(ULONG signal)
{
    MP_IO_INFO ioInfo;
    USHORT ioState;
    int ret;
    
	//set feedback signal
	ioInfo.ulAddr = signal;
	ret = mpReadIO(&ioInfo, &ioState, 1);
	if(ret != 0)
		printf("mpReadIO failure (%d)\r\n", ret);
		
	return (ioState != 0);
}


//-------------------------------------------------------------------
// Set I/O state on the controller
//-------------------------------------------------------------------
void Ros_Controller_SetIOState(ULONG signal, BOOL status)
{
    MP_IO_DATA ioData;
    int ret;
    
	//set feedback signal
	ioData.ulAddr = signal;
	ioData.ulValue = status;
	ret = mpWriteIO(&ioData, 1);
	if(ret != 0)
		printf("mpWriteIO failure (%d)\r\n", ret);
}


//-------------------------------------------------------------------
// Get the code of the first alarm on the controller
//-------------------------------------------------------------------
int Ros_Controller_GetAlarmCode(Controller* controller)
{
	MP_ALARM_CODE_RSP_DATA alarmData;
	memset(&alarmData, 0x00, sizeof(alarmData));
	if(mpGetAlarmCode(&alarmData) == 0)
	{
		if(alarmData.usAlarmNum > 0)
			return(alarmData.AlarmData.usAlarmNo[0]);
		else
			return 0;
	}
	return -1;
}


//-------------------------------------------------------------------
// Convert error code to string
//-------------------------------------------------------------------
void Ros_Controller_ErrNo_ToString(int errNo, char errMsg[ERROR_MSG_MAX_SIZE], int errMsgSize)
{
	switch(errNo)
	{
		case 0x2010: memcpy(errMsg, "Robot is in operation", errMsgSize); break;
		case 0x2030: memcpy(errMsg, "In HOLD status (PP)", errMsgSize); break;
		case 0x2040: memcpy(errMsg, "In HOLD status (External)", errMsgSize); break;
		case 0x2050: memcpy(errMsg, "In HOLD status (Command)", errMsgSize); break;
		case 0x2060: memcpy(errMsg, "In ERROR/ALARM status", errMsgSize); break;
		case 0x2070: memcpy(errMsg, "In SERVO OFF status", errMsgSize); break;
		case 0x2080: memcpy(errMsg, "Wrong operation mode", errMsgSize); break;
		case 0x3040: memcpy(errMsg, "The home position is not registered", errMsgSize); break;
		case 0x3050: memcpy(errMsg, "Out of range (ABSO data", errMsgSize); break;
		case 0x3400: memcpy(errMsg, "Cannot operate MASTER JOB", errMsgSize); break;
		case 0x3410: memcpy(errMsg, "The JOB name is already registered in another task", errMsgSize); break;
		case 0x4040: memcpy(errMsg, "Specified JOB not found", errMsgSize); break;
		case 0x5200: memcpy(errMsg, "Over data range", errMsgSize); break;
		default: memcpy(errMsg, "Unspecified reason", errMsgSize); break;
	}
}

