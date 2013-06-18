/* mp_main.c - MotoPlus Test Application for Real Time Process */
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

#include "motoPlus.h"
#include "ParameterExtraction.h"
#include "CtrlGroup.h"
#include "SimpleMessage.h"
#include "Controller.h"
#include "StateServer.h"
#include "MotionServer.h"

void RosInitTask();

//GLOBAL DATA DEFINITIONS
int RosInitTaskID;

void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{	
	//Creates and starts a new task in a seperate thread of execution.
	//All arguments will be passed to the new task if the function
	//prototype will accept them.
	RosInitTaskID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)RosInitTask,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
	//Ends the initialization task.
	mpExitUsrRoot;
}

void RosInitTask()
{
	Controller ros_controller;

	Ros_Controller_Init(&ros_controller);

	ros_controller.tidConnectionSrv = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
						(FUNCPTR)Ros_Controller_ConnectionServer_Start,
						(int)&ros_controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
						
	// start loop to monitor controller state
	FOREVER
	{
		// Check controller status
		Ros_Controller_StatusUpdate(&ros_controller);
	
		mpTaskDelay(CONTROLLER_STATUS_UPDATE_PERIOD);
	}
}

