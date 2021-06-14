// SimpleMessage.h
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

#ifndef SIMPLE_MSG_H
#define SIMPLE_MSG_H

#define ROS_MAX_JOINT 10
#define MOT_MAX_GR	4


//----------------
// Prefix Section
//----------------

typedef struct
{
	int length;
} SmPrefix;

//----------------
// Header Section
//----------------

typedef enum
{
	ROS_MSG_PING = 1,
	ROS_MSG_GET_VERSION = 2,
	ROS_MSG_ROBOT_STATUS = 13,

	ROS_MSG_JOINT_TRAJ_PT_FULL = 14,
	ROS_MSG_JOINT_FEEDBACK = 15,

	ROS_MSG_MOTO_MOTION_CTRL = 2001,
	ROS_MSG_MOTO_MOTION_REPLY = 2002,

	ROS_MSG_MOTO_READ_IO_BIT = 2003,
	ROS_MSG_MOTO_READ_IO_BIT_REPLY = 2004,
	ROS_MSG_MOTO_WRITE_IO_BIT = 2005,
	ROS_MSG_MOTO_WRITE_IO_BIT_REPLY = 2006,
	ROS_MSG_MOTO_READ_IO_GROUP = 2007,
	ROS_MSG_MOTO_READ_IO_GROUP_REPLY = 2008,
	ROS_MSG_MOTO_WRITE_IO_GROUP = 2009,
	ROS_MSG_MOTO_WRITE_IO_GROUP_REPLY = 2010,
	ROS_MSG_MOTO_IOCTRL_REPLY = 2011,
	ROS_MSG_MOTO_READ_MREGISTER = 2012,
	ROS_MSG_MOTO_WRITE_MREGISTER = 2013,

	ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX = 2016,
	ROS_MSG_MOTO_JOINT_FEEDBACK_EX = 2017,
	ROS_MSG_MOTO_SELECT_TOOL = 2018,

	ROS_MSG_MOTO_GET_DH_PARAMETERS = 2020
} SmMsgType;


typedef enum
{
	ROS_COMM_INVALID = 0,
	ROS_COMM_TOPIC = 1,
	ROS_COMM_SERVICE_REQUEST = 2,
	ROS_COMM_SERVICE_REPLY = 3
} SmCommType;


typedef enum 
{
	ROS_REPLY_INVALID = 0,
	ROS_REPLY_SUCCESS = 1,
	ROS_REPLY_FAILURE = 2,
} SmReplyType;


typedef enum
{
	ROS_CMD_CHECK_MOTION_READY = 200101,
	ROS_CMD_CHECK_QUEUE_CNT = 200102,
	ROS_CMD_STOP_MOTION = 200111,
	ROS_CMD_START_SERVOS = 200112, // starts the servo motors
	ROS_CMD_STOP_SERVOS = 200113, // stops the servo motors and motion
	ROS_CMD_RESET_ALARM = 200114, // clears the error in the current controller
	ROS_CMD_START_TRAJ_MODE = 200121,
	ROS_CMD_STOP_TRAJ_MODE = 200122,
	ROS_CMD_DISCONNECT = 200130
} SmCommandType;


typedef enum
{
	ROS_RESULT_SUCCESS = 0,
	ROS_RESULT_TRUE = 0,
	ROS_RESULT_BUSY = 1,
	ROS_RESULT_FAILURE = 2,
	ROS_RESULT_FALSE = 2,
	ROS_RESULT_INVALID = 3,
	ROS_RESULT_ALARM = 4,
	ROS_RESULT_NOT_READY = 5,
	ROS_RESULT_MP_FAILURE = 6
} SmResultType;


typedef enum
{
	ROS_RESULT_INVALID_UNSPECIFIED = 3000,
	ROS_RESULT_INVALID_MSGSIZE,
	ROS_RESULT_INVALID_MSGHEADER,
	ROS_RESULT_INVALID_MSGTYPE,
	ROS_RESULT_INVALID_GROUPNO,
	ROS_RESULT_INVALID_SEQUENCE,
	ROS_RESULT_INVALID_COMMAND,
	ROS_RESULT_INVALID_DATA = 3010,
	ROS_RESULT_INVALID_DATA_START_POS,
	ROS_RESULT_INVALID_DATA_POSITION,
	ROS_RESULT_INVALID_DATA_SPEED,
	ROS_RESULT_INVALID_DATA_ACCEL,
	ROS_RESULT_INVALID_DATA_INSUFFICIENT,
	ROS_RESULT_INVALID_DATA_TIME,
	ROS_RESULT_INVALID_DATA_TOOLNO
} SmInvalidSubCode;


typedef enum
{
	ROS_RESULT_NOT_READY_UNSPECIFIED = 5000,
	ROS_RESULT_NOT_READY_ALARM,
	ROS_RESULT_NOT_READY_ERROR,
	ROS_RESULT_NOT_READY_ESTOP,
	ROS_RESULT_NOT_READY_NOT_PLAY,
	ROS_RESULT_NOT_READY_NOT_REMOTE,
	ROS_RESULT_NOT_READY_SERVO_OFF,
	ROS_RESULT_NOT_READY_HOLD,
	ROS_RESULT_NOT_READY_NOT_STARTED,
	ROS_RESULT_NOT_READY_WAITING_ROS,
	ROS_RESULT_NOT_READY_SKILLSEND,
	ROS_RESULT_NOT_READY_PFL_ACTIVE,
	ROS_RESULT_NOT_READY_INC_MOVE_ERROR
} SmNotReadySubcode;


struct _SmHeader
{
	SmMsgType msgType;
	SmCommType commType;
	SmReplyType replyType;
} __attribute__((__packed__));
typedef struct _SmHeader SmHeader;

typedef enum
{
	Valid_Time = 1,
	Valid_Position = 2,
	Valid_Velocity = 4,
	Valid_Acceleration = 8
} FlagsValidFields;

//--------------
// Body Section
//--------------

struct _SmBodyRobotStatus		// ROS_MSG_ROBOT_STATUS = 13
{
	int drives_powered;			// Servo Power: -1=Unknown, 1=ON, 0=OFF
	int e_stopped;				// Controller E-Stop state: -1=Unknown, 1=True(ON), 0=False(OFF)
	int error_code;				// Alarm code
	int in_error;				// Is there an alarm:   -1=Unknown, 1=True, 0=False 
	int in_motion;				// Is currently executing a motion command:  -1=Unknown, 1=True, 0=False 
	int mode;  					// Controller/Pendant mode: -1=Unknown, 1=Manual(TEACH), 2=Auto(PLAY)
	int motion_possible;		// Is the controller ready to receive motion: -1=Unknown, 1=ENABLED, 0=DISABLED 
} __attribute__((__packed__));
typedef struct _SmBodyRobotStatus SmBodyRobotStatus;

struct _SmBodyJointTrajPtFull	// ROS_MSG_JOINT_TRAJ_PT_FULL = 14
{
	int groupNo;  				// Robot/group ID;  0 = 1st robot 
	int sequence;				// Index of point in trajectory; 0 = Initial trajectory point, which should match the robot current position.
	FlagsValidFields validFields;	// Bit-mask indicating which “optional” fields are filled with data. 1=time, 2=position, 4=velocity, 8=acceleration
	float time;					// Timestamp associated with this trajectory point; Units: in seconds 
	float pos[ROS_MAX_JOINT];	// Desired joint positions in radian.  Base to Tool joint order  
	float vel[ROS_MAX_JOINT];	// Desired joint velocities in radian/sec.  
	float acc[ROS_MAX_JOINT];	// Desired joint accelerations in radian/sec^2.
} __attribute__((__packed__));
typedef struct _SmBodyJointTrajPtFull SmBodyJointTrajPtFull;

struct _SmBodyJointFeedback		// ROS_MSG_JOINT_FEEDBACK = 15
{
	int groupNo;  				// Robot/group ID;  0 = 1st robot 
	FlagsValidFields validFields;	// Bit-mask indicating which “optional” fields are filled with data. 1=time, 2=position, 4=velocity, 8=acceleration
	float time;					// Timestamp associated with this trajectory point; Units: in seconds 
	float pos[ROS_MAX_JOINT];	// Feedback joint positions in radian.  Base to Tool joint order  
	float vel[ROS_MAX_JOINT];	// Feedback joint velocities in radian/sec.  
	float acc[ROS_MAX_JOINT];	// Feedback joint accelerations in radian/sec^2.
} __attribute__((__packed__));
typedef struct _SmBodyJointFeedback SmBodyJointFeedback;


struct _SmBodyMotoMotionCtrl	// ROS_MSG_MOTO_MOTION_CTRL = 2001
{
	int groupNo;  				// Robot/group ID;  0 = 1st robot 
	int sequence;				// Optional message tracking number that will be echoed back in the response.
	SmCommandType command;		// Desired command
	float data[ROS_MAX_JOINT];	// Command data - for future use  
} __attribute__((__packed__));
typedef struct _SmBodyMotoMotionCtrl SmBodyMotoMotionCtrl;


struct _SmBodyMotoMotionReply	// ROS_MSG_MOTO_MOTION_REPLY = 2002
{
	int groupNo;  				// Robot/group ID;  0 = 1st robot 
	int sequence;				// Optional message tracking number that will be echoed back in the response.
	int command;				// Reference to the received message command or type
	SmResultType result;		// High level result code
	int subcode;				// More detailed result code (optional)
	float data[ROS_MAX_JOINT];	// Reply data - for future use 
} __attribute__((__packed__));
typedef struct _SmBodyMotoMotionReply SmBodyMotoMotionReply;

struct _SmBodyJointTrajPtExData
{
	int groupNo;  				// Robot/group ID;  0 = 1st robot 
	FlagsValidFields validFields;	// Bit-mask indicating which “optional” fields are filled with data. 1=time, 2=position, 4=velocity, 8=acceleration
	float time;					// Timestamp associated with this trajectory point; Units: in seconds 
	float pos[ROS_MAX_JOINT];	// Desired joint positions in radian.  Base to Tool joint order  
	float vel[ROS_MAX_JOINT];	// Desired joint velocities in radian/sec.  
	float acc[ROS_MAX_JOINT];	// Desired joint accelerations in radian/sec^2.
} __attribute__((__packed__));
typedef struct _SmBodyJointTrajPtExData SmBodyJointTrajPtExData;

struct _SmBodyJointTrajPtFullEx
{
	int numberOfValidGroups;
	int sequence;
	SmBodyJointTrajPtExData	jointTrajPtData[MOT_MAX_GR];
} __attribute__((__packed__));
typedef struct _SmBodyJointTrajPtFullEx SmBodyJointTrajPtFullEx;


struct _SmBodyJointFeedbackEx
{
	int numberOfValidGroups;
	SmBodyJointFeedback	jointTrajPtData[MOT_MAX_GR];
} __attribute__((__packed__));
typedef struct _SmBodyJointFeedbackEx SmBodyJointFeedbackEx;

struct _SmBodySelectTool
{
	int groupNo;  				// Robot/group ID;  0 = 1st robot 
	int tool;					// Tool no for the selected group
	int sequence;				// Optional message tracking number that will be echoed back in the response.
} __attribute__((__packed__));
typedef struct _SmBodySelectTool SmBodySelectTool;

//--------------
// IO Commands
//--------------

typedef enum
{
	IO_RESULT_OK = 0,
	IO_RESULT_READ_ADDRESS_INVALID = 1001,	//The ioAddress cannot be read on this controller
	IO_RESULT_WRITE_ADDRESS_INVALID,		//The ioAddress cannot be written to on this controller
	IO_RESULT_WRITE_VALUE_INVALID,			//The value supplied is not a valid value for the addressed IO element
	IO_RESULT_READ_API_ERROR,				//mpReadIO return -1
	IO_RESULT_WRITE_API_ERROR				//mpWriteIO returned -1
} IoResultCodes;

struct _SmBodyMotoReadIOBit
{
	UINT32 ioAddress;
} __attribute__((__packed__));
typedef struct _SmBodyMotoReadIOBit SmBodyMotoReadIOBit;

struct _SmBodyMotoReadIOBitReply
{
	UINT32 value;
	IoResultCodes resultCode;
} __attribute__((__packed__));
typedef struct _SmBodyMotoReadIOBitReply SmBodyMotoReadIOBitReply;

struct _SmBodyMotoWriteIOBit
{
	UINT32 ioAddress;
	UINT32 ioValue;
} __attribute__((__packed__));
typedef struct _SmBodyMotoWriteIOBit SmBodyMotoWriteIOBit;

struct _SmBodyMotoWriteIOBitReply
{
	IoResultCodes resultCode;
} __attribute__((__packed__));
typedef struct _SmBodyMotoWriteIOBitReply SmBodyMotoWriteIOBitReply;

struct _SmBodyMotoReadIOGroup
{
	UINT32 ioAddress; //Group address. Example: '1001' will read OG#1 (Bits 10010 through 10017)
} __attribute__((__packed__));
typedef struct _SmBodyMotoReadIOGroup SmBodyMotoReadIOGroup;

struct _SmBodyMotoReadIOGroupReply
{
	UINT32 value;
	IoResultCodes resultCode;
} __attribute__((__packed__));
typedef struct _SmBodyMotoReadIOGroupReply SmBodyMotoReadIOGroupReply;

struct _SmBodyMotoWriteIOGroup
{
	UINT32 ioAddress; //Group address. Example: '1001' will write OG#1 (Bits 10010 through 10017)
	UINT32 ioValue;
} __attribute__((__packed__));
typedef struct _SmBodyMotoWriteIOGroup SmBodyMotoWriteIOGroup;

struct _SmBodyMotoWriteIOGroupReply
{
	IoResultCodes resultCode;
} __attribute__((__packed__));
typedef struct _SmBodyMotoWriteIOGroupReply SmBodyMotoWriteIOGroupReply;

struct _SmBodyMotoIoCtrlReply	// ROS_MSG_MOTO_IOCTRL_REPLY = 2011
{
	SmResultType result;		// High level result code
	int subcode;				// More detailed result code (optional)
} __attribute__((__packed__));
typedef struct _SmBodyMotoIoCtrlReply SmBodyMotoIoCtrlReply;

struct _SmBodyMotoReadIOMRegister
{
	UINT32 registerNumber;	//Actual register index. Do not add 1000000 to this value. (Example: 123 = M123)
} __attribute__((__packed__));
typedef struct _SmBodyMotoReadIOMRegister SmBodyMotoReadIOMRegister;

struct _SmBodyMotoReadIOMRegisterReply
{
	UINT32 value;
	IoResultCodes resultCode;
} __attribute__((__packed__));
typedef struct _SmBodyMotoReadIOMRegisterReply SmBodyMotoReadIOMRegisterReply;

struct _SmBodyMotoWriteIOMRegister
{
	UINT32 registerNumber;	//Actual register index. Do not add 1000000 to this value. (Example: 123 = M123)
	UINT32 value;
} __attribute__((__packed__));
typedef struct _SmBodyMotoWriteIOMRegister SmBodyMotoWriteIOMRegister;

struct _SmBodyMotoWriteIOMRegisterReply
{
	IoResultCodes resultCode;
} __attribute__((__packed__));
typedef struct _SmBodyMotoWriteIOMRegisterReply SmBodyMotoWriteIOMRegisterReply;

//--------------
// DH Parameters
//--------------

struct _SmBodyMotoGetDhParameters
{
	DH_PARAMETERS dhParameters[MOT_MAX_GR];
} __attribute__((__packed__));
typedef struct _SmBodyMotoGetDhParameters SmBodyMotoGetDhParameters;

//--------------
// Body Union
//--------------

typedef union
{
	SmBodyRobotStatus robotStatus;
	SmBodyJointTrajPtFull jointTrajData;
	SmBodyJointFeedback	jointFeedback;
	SmBodyMotoMotionCtrl motionCtrl;
	SmBodyMotoMotionReply motionReply;
	SmBodyJointTrajPtFullEx jointTrajDataEx;
	SmBodyJointFeedbackEx jointFeedbackEx;
	SmBodySelectTool selectTool;
	SmBodyMotoReadIOBit readIOBit;
	SmBodyMotoReadIOBitReply readIOBitReply;
	SmBodyMotoWriteIOBit writeIOBit;
	SmBodyMotoWriteIOBitReply writeIOBitReply;
	SmBodyMotoReadIOGroup readIOGroup;
	SmBodyMotoReadIOGroupReply readIOGroupReply;
	SmBodyMotoWriteIOGroup writeIOGroup;
	SmBodyMotoWriteIOGroupReply writeIOGroupReply;
	SmBodyMotoIoCtrlReply ioCtrlReply;
	SmBodyMotoGetDhParameters dhParameters;
	SmBodyMotoReadIOMRegister readRegister;
	SmBodyMotoReadIOMRegisterReply readRegisterReply;
	SmBodyMotoWriteIOMRegister writeRegister;
	SmBodyMotoWriteIOMRegisterReply writeRegisterReply;
} SmBody;

//-------------------
// SimpleMsg Section
//-------------------
struct _SimpleMsg
{
	SmPrefix prefix;
	SmHeader header;
	SmBody body;
} __attribute__((__packed__));
typedef struct _SimpleMsg SimpleMsg;


//-------------------
// Function Section
//-------------------

extern int Ros_SimpleMsg_JointFeedback(CtrlGroup* ctrlGroup, SimpleMsg* sendMsg);
extern void Ros_SimpleMsg_JointFeedbackEx_Init(int numberOfGroups, SimpleMsg* sendMsg);
extern int Ros_SimpleMsg_JointFeedbackEx_Build(int groupIndex, SimpleMsg* src_msgFeedback, SimpleMsg* dst_msgExtendedFeedback);

extern int Ros_SimpleMsg_MotionReply(SimpleMsg* receiveMsg, int result, int subcode, SimpleMsg* replyMsg, int ctrlGrp);
extern int Ros_SimpleMsg_IoReply(int result, int subcode, SimpleMsg* replyMsg);

//Uncomment the DEBUG definition to enable debug-messages at runtime
//#define DEBUG  1

#ifdef DEBUG
#warning Dont forget to disable the DEBUG flag
// function to dump data structure for debugging
extern void Ros_SimpleMsg_DumpTrajPtFull(SmBodyJointTrajPtFull* data);
#endif

#endif
