//Controller.h
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

#ifndef CONTROLLER_H
#define CONTROLLER_H

//#define DX100          // Define to compile for DX100 controller  

#define TCP_PORT_MOTION	50240
#define TCP_PORT_STATE	50241

#define IO_FEEDBACK_WAITING_MP_INCMOVE	  11120  //output# 889 
#define IO_FEEDBACK_MP_INCMOVE_DONE	  	  11121  //output# 890 
#define IO_FEEDBACK_INITIALIZATION_DONE	  11122  //output# 891 
#define IO_FEEDBACK_CONNECTSERVERRUNNING  11123	 //output# 892 
#define IO_FEEDBACK_MOTIONSERVERCONNECTED 11124	 //output# 893 
#define IO_FEEDBACK_STATESERVERCONNECTED  11125	 //output# 894 
#define IO_FEEDBACK_FAILURE				  11127  //output# 896

#define IO_

#define MAX_MOTION_CONNECTIONS	1
#define MAX_STATE_CONNECTIONS	4

#define INVALID_SOCKET -1
#define INVALID_TASK -1
#define IPPROTO_TCP  6

#define ERROR_MSG_MAX_SIZE 64

#define START_MAX_PULSE_DEVIATION 10

#define CONTROLLER_STATUS_UPDATE_PERIOD 10

typedef enum 
{
	IOSTATUS_ALARM_MAJOR = 0,
	IOSTATUS_ALARM_MINOR,
	IOSTATUS_ALARM_SYSTEM,
	IOSTATUS_ALARM_USER,
	IOSTATUS_ERROR,
	IOSTATUS_PLAY,
	IOSTATUS_TEACH,
	IOSTATUS_REMOTE,
	IOSTATUS_OPERATING,
	IOSTATUS_HOLD,
	IOSTATUS_SERVO,
	IOSTATUS_ESTOP_EX,
	IOSTATUS_ESTOP_PP,
	IOSTATUS_ESTOP_CTRL,
	IOSTATUS_WAITING_ROS,
	IOSTATUS_MAX
} IoStatusIndex;
 
typedef struct
{
	UINT16 interpolPeriod;									// Interpolation period of the controller
	int numGroup;											// Actual number of defined group
	int numRobot;											// Actual number of defined robot
	CtrlGroup* ctrlGroups[MP_GRP_NUM];						// Array of the controller control group
	
	// Controller Status
	MP_IO_INFO ioStatusAddr[IOSTATUS_MAX];					// Array of Specific Input Address representing the I/O status
    USHORT ioStatus[IOSTATUS_MAX];							// Array storing the current status of the controller
    int alarmCode;											// Alarm number currently active
	BOOL bRobotJobReady;									// Boolean indicating that the controller is ready for increment move
    BOOL bRobotJobReadyRaised;								// Indicates that the signal was raised since operating was resumed
    BOOL bStopMotion;										// Flag to stop motion

	// Connection Server
	int tidConnectionSrv;
	
	// State Server Connection
	int tidStateSendState;  								// ThreadId of thread sending the controller state
	int	sdStateConnections[MAX_STATE_CONNECTIONS];			// Socket Descriptor array for State Server

	// Motion Server Connection
	int	sdMotionConnections[MAX_MOTION_CONNECTIONS];		// Socket Descriptor array for Motion Server
	int	tidMotionConnections[MAX_MOTION_CONNECTIONS];  		// ThreadId array for Motion Server
	int tidIncMoveThread;  									// ThreadId for sending the incremental move to the controller

#ifdef DX100
	BOOL bSkillMotionReady[2];								// Boolean indicating that the SKILL command required for DX100 is active
	int RosListenForSkillID[2];								// ThreadId for listening to SkillSend command
#endif
	
} Controller;

extern BOOL Ros_Controller_Init(Controller* controller);
extern BOOL Ros_Controller_IsValidGroupNo(Controller* controller, int groupNo);
extern void Ros_Controller_ConnectionServer_Start(Controller* controller);

extern void Ros_Controller_StatusInit(Controller* controller);
extern BOOL Ros_Controller_StatusRead(Controller* controller, USHORT ioStatus[IOSTATUS_MAX]);
extern BOOL Ros_Controller_StatusUpdate(Controller* controller);
extern BOOL Ros_Controller_IsAlarm(Controller* controller);
extern BOOL Ros_Controller_IsError(Controller* controller);
extern BOOL Ros_Controller_IsPlay(Controller* controller);
extern BOOL Ros_Controller_IsTeach(Controller* controller);
extern BOOL Ros_Controller_IsRemote(Controller* controller);
extern BOOL Ros_Controller_IsOperating(Controller* controller);
extern BOOL Ros_Controller_IsHold(Controller* controller);
extern BOOL Ros_Controller_IsServoOn(Controller* controller);
extern BOOL Ros_Controller_IsEStop(Controller* controller);
extern BOOL Ros_Controller_IsWaitingRos(Controller* controller);
extern BOOL Ros_Controller_IsMotionReady(Controller* controller);
extern int Ros_Controller_GetNotReadySubcode(Controller* controller);
extern int Ros_Controller_StatusToMsg(Controller* controller, SimpleMsg* sendMsg);

extern BOOL Ros_Controller_GetIOState(ULONG signal);
extern void Ros_Controller_SetIOState(ULONG signal, BOOL status);
extern void Ros_Controller_ErrNo_ToString(int errNo, char errMsg[ERROR_MSG_MAX_SIZE], int errMsgSize);

#ifdef DX100
extern void Ros_Controller_ListenForSkill(Controller* controller, int sl);
#endif

#endif
