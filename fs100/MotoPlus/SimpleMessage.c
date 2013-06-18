// SimpleMessage.c
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

//-----------------------
// Function Declarations
//-----------------------
int Ros_SimpleMsg_JointFeedback(CtrlGroup* ctrlGroup, SimpleMsg* sendMsg);
int Ros_SimpleMsg_MotionReply(SimpleMsg* receiveMsg, int result, int subcode, SimpleMsg* replyMsg);


//-----------------------
// Function implementation
//-----------------------

// Creates a simple message of type: ROS_MSG_JOINT_FEEDBACK = 15
// Simple message containing a the current joint position
// of the specified control group
int Ros_SimpleMsg_JointFeedback(CtrlGroup* ctrlGroup, SimpleMsg* sendMsg)
{
	int bRet;
	long pulsePos[MAX_PULSE_AXES];
	
	//initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyJointFeedback);
	
	// set header information
	sendMsg->header.msgType = ROS_MSG_JOINT_FEEDBACK;
	sendMsg->header.commType = ROS_COMM_TOPIC;
	sendMsg->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	sendMsg->body.jointFeedback.groupNo = ctrlGroup->groupNo;
	sendMsg->body.jointFeedback.validFields = 2;
	
	bRet = Ros_CtrlGroup_GetFBPulsePos(ctrlGroup, pulsePos);
	if(bRet!=TRUE)
		return 0;
				
	Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, pulsePos, sendMsg->body.jointFeedback.pos);

	// For testing
	//bRet = Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, pulsePos);
	//if(bRet!=TRUE)
	//	return 0;
	//	
	//Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, pulsePos, sendMsg->body.jointFeedback.vel);
	// End testing
	
	return(sendMsg->prefix.length + sizeof(SmPrefix));
}


// Creates a simple message of type MOTO_MOTION_REPLY to reply to a received message 
// result code and subcode indication result of the processing of the received message
int Ros_SimpleMsg_MotionReply(SimpleMsg* receiveMsg, int result, int subcode, SimpleMsg* replyMsg)
{
	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoMotionReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_MOTION_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	replyMsg->header.replyType = ROS_REPLY_SUCCESS;
	
	// set reply body
	if(receiveMsg->header.msgType == ROS_MSG_MOTO_MOTION_CTRL)
	{
		replyMsg->body.motionReply.groupNo = receiveMsg->body.motionCtrl.groupNo;
		replyMsg->body.motionReply.sequence = receiveMsg->body.motionCtrl.sequence;
		replyMsg->body.motionReply.command = receiveMsg->body.motionCtrl.command;
	}
	else if(receiveMsg->header.msgType == ROS_MSG_MOTO_MOTION_CTRL)
	{
		replyMsg->body.motionReply.groupNo = receiveMsg->body.motionCtrl.groupNo;
		replyMsg->body.motionReply.sequence = receiveMsg->body.motionCtrl.sequence;
		replyMsg->body.motionReply.command = ROS_MSG_MOTO_MOTION_CTRL;
	}
	else
	{
		replyMsg->body.motionReply.groupNo = -1;
		replyMsg->body.motionReply.sequence = -1;
		replyMsg->body.motionReply.command = receiveMsg->header.msgType;
	}

	replyMsg->body.motionReply.result = result;
	replyMsg->body.motionReply.subcode = subcode;
	
	return(replyMsg->prefix.length + sizeof(SmPrefix));
}

