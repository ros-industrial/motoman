/* ParameterExtraction.h - Parameter Extraction definitions header file */

/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Yaskawa America, Inc.
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

#ifndef _INC_GETMOTOMANPARAMETERS_H
#define _INC_GETMOTOMANPARAMETERS_H

#include "ParameterTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* << 2 >>                                                              	  */
/* Function name : int  GP_getNumberOfGroups()								  */
/* Functionality : Retrieves the Number of Defined Groups					  */
/* Parameter	 : NONE														  */
/* Return value	 : Success = Number of Groups								  */
/*				 : Failure = -1												  */	
/******************************************************************************/
extern int  	GP_getNumberOfGroups();

/******************************************************************************/
/* << 3 >>                                                              	  */
/* Function name : int  GP_getNumberOfAxes()								  */
/* Functionality : Retrieves the Number of Axes								  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/* Return value	 : Success = Number of Axes									  */
/*				 : Failure = -1												  */	
/******************************************************************************/
extern int  	GP_getNumberOfAxes(int ctrlGrp);

/******************************************************************************/
/* << 4 >>                                                              	  */
/* Function name : STATUS GP_getPulseToRad()								  */
/* Functionality : Gets the Pulse to radians conversion factors				  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   GB_PULSE_TO_RAD *PulseToRad -array of conversion data [OUT]*/
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */	
/******************************************************************************/
extern STATUS 	GP_getPulseToRad(int ctrlGrp, PULSE_TO_RAD *PulseToRad);

/******************************************************************************/
/* << 11 >>                                                             	  */
/* Function name : STATUS GetFBPulseCorrection()							  */
/* Functionality : Get all the pulse correction data for required axes		  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data [IN]			  */
/*				   FB_PULSE_CORRECTION_DATA * correctionData[OUT]			  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_getFBPulseCorrection(int ctrlGrp, FB_PULSE_CORRECTION_DATA *correctionData);
	
/******************************************************************************/
/* << 12 >>                                                             	  */
/* Function name : STATUS GP_getQtyOfAllowedTasks()							  */
/* Functionality : No.of MotoPlus tasks that can be started concurrently  	  */
/* Parameter	 : TASK_QTY_INFO *taskInfo [OUT]				  			  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_getQtyOfAllowedTasks(TASK_QTY_INFO *taskInfo);

/******************************************************************************/
/* << 13 >>                                                             	  */
/* Function name : STATUS GP_getInterpolationPeriod()						  */
/* Functionality : No.of millisecs b/w each tick of the interpolation-clock	  */
/* Parameter	 : UINT16 *periodInMilliseconds [OUT]						  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */	
/******************************************************************************/
extern STATUS GP_getInterpolationPeriod(UINT16* periodInMilliseconds);

/******************************************************************************/
/* << 14 >>                                                             	  */
/* Function name : STATUS GP_getMaxIncPerIpCycle()							  */
/* Functionality : Max increment the arm is capable of(limited by governor)	  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data [IN]			  */
/*				   int interpolationPeriodInMilliseconds - obtained from GP_getInterpolationPeriod [IN] */
/*				   MAX_INCREMENT_INFO *mip [OUT]	   		 				  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */	
/******************************************************************************/
extern STATUS GP_getMaxIncPerIpCycle(int ctrlGrp, int interpolationPeriodInMilliseconds, MAX_INCREMENT_INFO *mip);

/******************************************************************************/
/* << 15 >>                                                             	  */
/* Function name : GP_getGovForIncMotion()									  */
/* Functionality : Percentage Limit of the max-increment					  */
/* Parameter	 : int ctrlGrp 				[IN]			  				  */
/* Return value	 : Success = percentage limit Of MaxSpeed					  */
/*				 : Failure = -1												  */	
/******************************************************************************/
extern float GP_getGovForIncMotion(int ctrlGrp);

/******************************************************************************/
/* << 16 >>                                                              	  */
/* Function name : STATUS GP_getJointPulseLimits()							  */
/* Functionality : Gets the Pulse to radians conversion factors				  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   GB_PULSE_TO_RAD *PulseToRad -array of conversion data [OUT]*/
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_getJointPulseLimits(int ctrlGrp, JOINT_PULSE_LIMITS* jointPulseLimits);

/******************************************************************************/
/* << 17 >>                                                              	  */
/* Function name : STATUS GP_getJointVelocityLimits()						  */
/* Functionality : Gets the velocity limit for each joint					  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   JOINT_ANGULAR_VELOCITY_LIMITS *GP_getJointAngularVelocityLimits (deg/sec) [OUT]*/
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_getJointAngularVelocityLimits(int ctrlGrp, JOINT_ANGULAR_VELOCITY_LIMITS* jointVelocityLimits);

/******************************************************************************/
/* << 18 >>                                                              	  */
/* Function name : STATUS GP_getAxisMotionType()							  */
/* Functionality : Gets the motion type of each axis in the group			  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   AXIS_MOTION_TYPE *axisType -array of data [OUT]			  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS 	GP_getAxisMotionType(int ctrlGrp, AXIS_MOTION_TYPE* axisType);

/******************************************************************************/
/* << 19 >>                                                              	  */
/* Function name : STATUS GP_getPulseToMeter()								  */
/* Functionality : Gets the Pulse to meter conversion factors				  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   PULSE_TO_METER *PulseToMeter -array of conversion data [OUT]*/
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS 	GP_getPulseToMeter(int ctrlGrp, PULSE_TO_METER* PulseToMeter);

/******************************************************************************/
/* << 20 >>                                                              	  */
/* Function name : STATUS GP_isBaxisSlave()									  */
/* Functionality : Determines if B axis is automatically moved relative to	  */
/*				   other axes.												  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   BOOL* bBaxisIsSlave - TRUE if b axis is slave [OUT]		  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS 	GP_isBaxisSlave(int ctrlGrp, BOOL* bBaxisIsSlave);

/******************************************************************************/
/* << 21 >>                                                              	  */
/* Function name : STATUS GP_getFeedbackSpeedMRegisterAddresses()			  */
/* Functionality : Obtains the MRegister CIO addresses that contain the 	  */
/*				   feedback speed for each axis. Optionally enables this      */
/*				   feature if not already enabled.						      */
/* Parameter	 : int ctrlGrp - Robot control group (zero based index) [IN]  */
/*				   BOOL bActivateIfNotEnabled - TRUE to enable feature [IN]   */
/*				   BOOL bForceRebootAfterActivation - TRUE to force the user  */
/*                 to reboot if this feature gets activated. Set to FALSE if  */
/*                 you plan to enable for additional control groups. [IN]     */
/*				   JOINT_FEEDBACK_SPEED_ADDRESSES* registerAddresses -		  */
/*				   Obtains the CIO register address for the feedback data [OUT]*/
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS	GP_getFeedbackSpeedMRegisterAddresses(int ctrlGrp, BOOL bActivateIfNotEnabled, BOOL bForceRebootAfterActivation, JOINT_FEEDBACK_SPEED_ADDRESSES* registerAddresses);

/******************************************************************************/
/* << 22 >>                                                              	  */
/* Function name : STATUS GP_isSdaRobot()									  */
/* Functionality : Determines if the robot is a dual-arm SDA.				  */
/* Parameter	 : BOOL* bIsSda - TRUE if robot is SDA [OUT]				  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS 	GP_isSdaRobot(BOOL* bIsSda);

/******************************************************************************/
/* << 23 >>                                                              	  */
/* Function name : STATUS GP_isSharedBaseAxis()								  */
/* Functionality : Determines if the robot is an SDA that has a base axis 	  */
/*					which is shared over multiple control groups.			  */
/* Parameter	 : BOOL* bIsSharedBaseAxis [OUT]				  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS 	GP_isSharedBaseAxis(BOOL* bIsSharedBaseAxis);

/******************************************************************************/
/* << 24 >>                                                              	  */
/* Function name : STATUS GP_getDhParameters()								  */
/* Functionality : Retrieves DH parameters for a given control group.		  */
/* Parameter	 : int ctrlGrp - Robot control group (zero based index) [IN]  */
/*					DH_PARAMETERS* dh - Value of the DH parameters [OUT]      */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS 	GP_getDhParameters(int ctrlGrp, DH_PARAMETERS* dh);

/******************************************************************************/
/* << 25 >>                                                              	  */
/* Function name : STATUS GP_isPflEnabled()								      */
/* Functionality : Determines if the robot is currently supporting the PFL    */
/* function			                                                          */
/* Parameter	 : BOOL* bIsPflEnabled [OUT]			                 	  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_isPflEnabled(BOOL* bIsPflEnabled);

#ifdef __cplusplus
}
#endif

#endif
