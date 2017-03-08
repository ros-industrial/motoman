/* ParameterTypes.h - Parameter Extraction type definitions header file */

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

#ifndef _INC_MOTOMANPARAMETER_TYPES_H
#define _INC_MOTOMANPARAMETER_TYPES_H

#include "MotoPlus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float PtoR[MAX_PULSE_AXES]; //Array to store PULSE TO RADIAN conversion factors for each axes
} PULSE_TO_RAD;

typedef struct
{
	float PtoM[MAX_PULSE_AXES]; //Array to store PULSE TO METER conversion factors for each axes
} PULSE_TO_METER;

typedef enum
{
	AXIS_ROTATION,
	AXIS_LINEAR,
	AXIS_INVALID
} AXIS_TYPE;

typedef struct
{
	AXIS_TYPE	type[MAX_PULSE_AXES];	//Array to store whether axis is rotational or linear
} AXIS_MOTION_TYPE;

typedef struct
{
	BOOL  bValid;			//TRUE if ulSourceAxis != 0
	INT32 ulSourceAxis;		
	INT32 ulCorrectionAxis;	 
	float fCorrectionRatio;	
} FB_AXIS_CORRECTION;

typedef struct
{
	FB_AXIS_CORRECTION  correction[MAX_PULSE_AXES];
} FB_PULSE_CORRECTION_DATA;

typedef struct
{
	UINT32	qtyOfOutFiles;				
	UINT32	qtyOfHighPriorityTasks;		
	UINT32	qtyOfNormalPriorityTasks;	
} TASK_QTY_INFO;
	
typedef struct
{
	UINT16 periodInMilliseconds;
} GP_INTERPOLATION_PERIOD;

typedef struct
{
	UINT32	maxIncrement[MAX_PULSE_AXES];
} MAX_INCREMENT_INFO;

typedef struct
{
	int ctrlGrp;				//Robot control group
	int IpCycleInMilliseconds;	//Interpolation Cycle in milliseconds
	MAX_INCREMENT_INFO info;	//Maximum increment per interpolation cycle
} MAX_INC_PIPC;

typedef struct
{
	INT32 maxLimit[MAX_PULSE_AXES];
	INT32 minLimit[MAX_PULSE_AXES];
} JOINT_PULSE_LIMITS;

typedef struct
{
	INT32 maxLimit[MAX_PULSE_AXES];
} JOINT_ANGULAR_VELOCITY_LIMITS;

#ifdef __cplusplus
}
#endif

#endif
