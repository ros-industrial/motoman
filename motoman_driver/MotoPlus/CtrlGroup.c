// CtrlGroup.c
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
MP_GRP_ID_TYPE Ros_CtrlGroup_FindGrpId(int groupNo);
CtrlGroup* Ros_CtrlGroup_Create(int groupNo, float interpolPeriod);
BOOL Ros_CtrlGroup_GetPulsePosCmd(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES]);
BOOL Ros_CtrlGroup_GetFBPulsePos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES]);
void Ros_CtrlGroup_ConvertToRosPos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES], 
									float rosPos[MAX_PULSE_AXES]);
void Ros_CtrlGroup_ConvertToMotoPos(CtrlGroup* ctrlGroup, float rosPos[MAX_PULSE_AXES],
									long pulsePos[MAX_PULSE_AXES]);
UCHAR Ros_CtrlGroup_GetAxisConfig(CtrlGroup* ctrlGroup);
BOOL Ros_CtrlGroup_IsRobot(CtrlGroup* ctrlGroup);

//-----------------------
// Function implementation
//-----------------------

//-------------------------------------------------------------------
// Search through the control group to find the GroupId that matches 
// the group number
//-------------------------------------------------------------------
MP_GRP_ID_TYPE Ros_CtrlGroup_FindGrpId(int groupNo)
{
	MP_GRP_ID_TYPE grp_id;
	
	for(grp_id = MP_R1_GID; grp_id < MP_S3_GID; grp_id++)
	{
		if(groupNo == mpCtrlGrpId2GrpNo(grp_id))
			return grp_id;
	}
	
	return -1;
}

//-------------------------------------------------------------------
// Create a CtrlGroup data structure for existing group otherwise 
// return NULL
//-------------------------------------------------------------------
CtrlGroup* Ros_CtrlGroup_Create(int groupNo, float interpolPeriod)
{
	CtrlGroup* ctrlGroup;
	int numAxes;
	int i;
#ifdef DX100		
	float speedCap;
#endif
	long maxSpeedPulse[MP_GRP_AXES_NUM];
	STATUS status;
	BOOL bInitOk;
	BOOL slaveAxis;
	
	// Check if group is defined
	numAxes = GP_getNumberOfAxes(groupNo);
#ifdef DEBUG
	printf("Group %d: Num Axes %d\n", groupNo, numAxes);
#endif
	if(numAxes > 0)
	{
		bInitOk = TRUE;
		// Allocate and initialize memory
		ctrlGroup = mpMalloc(sizeof(CtrlGroup));
		memset(ctrlGroup, 0x00, sizeof(CtrlGroup));
		
		// Populate values
		ctrlGroup->groupNo = groupNo;
		ctrlGroup->numAxes = numAxes;
		ctrlGroup->groupId = Ros_CtrlGroup_FindGrpId(groupNo);

		status = GP_getAxisMotionType(groupNo, &ctrlGroup->axisType);
		if (status != OK)
			bInitOk = FALSE;

		status = GP_getPulseToRad(groupNo, &ctrlGroup->pulseToRad);
		if (status != OK)
			bInitOk = FALSE;

		status = GP_getPulseToMeter(groupNo, &ctrlGroup->pulseToMeter);
		if (status != OK)
			bInitOk = FALSE;

		status = GP_getFBPulseCorrection(groupNo, &ctrlGroup->correctionData);
		if(status!=OK)
			bInitOk = FALSE;

		status = GP_getMaxIncPerIpCycle(groupNo, interpolPeriod, &ctrlGroup->maxInc);
		if (status != OK)
			bInitOk = FALSE;

		status = GP_isBaxisSlave(groupNo, &slaveAxis);
		if (status != OK)
			bInitOk = FALSE;

		ctrlGroup->bIsBaxisSlave = (numAxes == 5) && slaveAxis;

		//adjust the axisType field to account for robots with non-contiguous axes (such as delta or palletizing which use SLU--T axes)
		for (i = 0; i < MP_GRP_AXES_NUM; i += 1)
		{
			if (ctrlGroup->maxInc.maxIncrement[i] == 0) //if the axis can't move, then I assume it's invalid
				ctrlGroup->axisType.type[i] = AXIS_INVALID;
		}

		memset(&ctrlGroup->inc_q, 0x00, sizeof(Incremental_q));
		ctrlGroup->inc_q.q_lock = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);

#ifdef DX100		
		speedCap = GP_getGovForIncMotion(groupNo);
		if(speedCap != -1)
		{
			for(i=0; i<MP_GRP_AXES_NUM; i++)
				ctrlGroup->maxInc.maxIncrement[i] *= speedCap;
		}
		else
			bInitOk = FALSE;
#endif

		// Calculate maximum speed in radian per second
		memset(maxSpeedPulse, 0x00, sizeof(maxSpeedPulse));
		for(i=0; i<MP_GRP_AXES_NUM; i++)
			maxSpeedPulse[i] = ctrlGroup->maxInc.maxIncrement[i] * 1000.0 / interpolPeriod; 
		Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, maxSpeedPulse, ctrlGroup->maxSpeed); 

		printf("axisType[%d]: ", groupNo);
		for (i = 0; i < MP_GRP_AXES_NUM; i++)
		{
			if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
				printf("Lin\t");
			else if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
				printf("Rot\t");
			else
			{
				printf("---\t");
				//motoRosAssert(FALSE, SUBCODE_INVALID_AXIS_TYPE, "Grp%d Axs%d type invalid", groupNo, i);
			}
		}
		printf(";\r\n");

		printf("pulse->unit[%d]: ", groupNo);
		for (i = 0; i < MP_GRP_AXES_NUM; i++)
		{
			if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
				printf("%.4f\t", ctrlGroup->pulseToMeter.PtoM[i]);
			else if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
				printf("%.4f\t", ctrlGroup->pulseToRad.PtoR[i]);
			else
				printf("--\t");
		}
		printf(";\r\n");

		printf("maxInc[%d] (in motoman joint order): %d, %d, %d, %d, %d, %d, %d\r\n", 
			groupNo,
			ctrlGroup->maxInc.maxIncrement[0],ctrlGroup->maxInc.maxIncrement[1],ctrlGroup->maxInc.maxIncrement[2],
			ctrlGroup->maxInc.maxIncrement[3],ctrlGroup->maxInc.maxIncrement[4],ctrlGroup->maxInc.maxIncrement[5],
			ctrlGroup->maxInc.maxIncrement[6]);

		printf("maxSpeed[%d] (in ros joint order): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\r\n",
			groupNo,
			ctrlGroup->maxSpeed[0],ctrlGroup->maxSpeed[1],ctrlGroup->maxSpeed[2],
			ctrlGroup->maxSpeed[3],ctrlGroup->maxSpeed[4],ctrlGroup->maxSpeed[5],
			ctrlGroup->maxSpeed[6]);

		ctrlGroup->tidAddToIncQueue = INVALID_TASK;
		
		if(bInitOk == FALSE)
		{
			mpFree(ctrlGroup);
			ctrlGroup = NULL;
		}
	}
	else
	{
		ctrlGroup = NULL;
	}
	
	return ctrlGroup;
}


//-------------------------------------------------------------------
// Get the commanded pulse position in pulse (in motoman joint order)
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetPulsePosCmd(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES])
{
	LONG status = 0;
	MP_CTRL_GRP_SEND_DATA sData;
	MP_PULSE_POS_RSP_DATA pulse_data;
	int i;

	memset(pulsePos, 0, MAX_PULSE_AXES*sizeof(long));  // clear result, in case of error

	// Set the control group
	switch(ctrlGroup->groupId)
	{
		case MP_R1_GID: sData.sCtrlGrp = 0; break;
		case MP_R2_GID: sData.sCtrlGrp = 1; break;
		case MP_R3_GID: sData.sCtrlGrp = 2; break;
		case MP_R4_GID: sData.sCtrlGrp = 3; break;
		case MP_B1_GID: sData.sCtrlGrp = 8; break;
		case MP_B2_GID: sData.sCtrlGrp = 9; break;
		case MP_B3_GID: sData.sCtrlGrp = 10; break;
		case MP_B4_GID: sData.sCtrlGrp = 11; break;
		case MP_S1_GID: sData.sCtrlGrp = 16; break;
		case MP_S2_GID: sData.sCtrlGrp = 17; break;
		case MP_S3_GID: sData.sCtrlGrp = 18; break;
		default:
			printf("Failed to get pulse feedback position\nInvalid groupId: %d\n", ctrlGroup->groupId);
			return FALSE;
	}
	
	// get the command joint positions
	status = mpGetPulsePos (&sData,&pulse_data);
	if (0 != status)
	{
		printf("Failed to get pulse position (command): %u\n", status);
		return FALSE;
	}
	
	// assign return value
	for (i=0; i<MAX_PULSE_AXES; ++i)
		pulsePos[i] = pulse_data.lPos[i];

	return TRUE;
}


//-------------------------------------------------------------------
// Get the corrected feedback pulse position in pulse
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetFBPulsePos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES])
{
	LONG status = 0;
	MP_CTRL_GRP_SEND_DATA sData;
#ifndef DUMMY_SERVO_MODE
	MP_FB_PULSE_POS_RSP_DATA pulse_data;
#else
	MP_PULSE_POS_RSP_DATA pulse_data;
#endif
	int i;

	memset(pulsePos, 0, MAX_PULSE_AXES*sizeof(long));  // clear result, in case of error

	// Set the control group
	switch(ctrlGroup->groupId)
	{
		case MP_R1_GID: sData.sCtrlGrp = 0; break;
		case MP_R2_GID: sData.sCtrlGrp = 1; break;
		case MP_R3_GID: sData.sCtrlGrp = 2; break;
		case MP_R4_GID: sData.sCtrlGrp = 3; break;
		case MP_B1_GID: sData.sCtrlGrp = 8; break;
		case MP_B2_GID: sData.sCtrlGrp = 9; break;
		case MP_B3_GID: sData.sCtrlGrp = 10; break;
		case MP_B4_GID: sData.sCtrlGrp = 11; break;
		case MP_S1_GID: sData.sCtrlGrp = 16; break;
		case MP_S2_GID: sData.sCtrlGrp = 17; break;
		case MP_S3_GID: sData.sCtrlGrp = 18; break;
		default: 
			printf("Failed to get pulse feedback position\nInvalid groupId: %d\n", ctrlGroup->groupId);
			return FALSE;
	}
	
#ifndef DUMMY_SERVO_MODE
	// get raw (uncorrected/unscaled) joint positions
	status = mpGetFBPulsePos (&sData,&pulse_data);
	if (0 != status)
	{
		printf("Failed to get pulse feedback position: %u\n", status);
		return FALSE;
	}
	
	// apply correction to account for cross-axis coupling
	// Note: this is only required for feedback position
	// controller handles this correction internally when 
	// dealing with command positon.
	for (i=0; i<MAX_PULSE_AXES; ++i)
	{
		FB_AXIS_CORRECTION *corr = &ctrlGroup->correctionData.correction[i];
		if (corr->bValid)
		{
			int src_axis = corr->ulSourceAxis;
			int dest_axis = corr->ulCorrectionAxis;
			pulse_data.lPos[dest_axis] -= (int)(pulse_data.lPos[src_axis] * corr->fCorrectionRatio);
		}
	}
#else
	mpGetPulsePos(&sData, &pulse_data);
#endif
	
	// assign return value
	for (i=0; i<MAX_PULSE_AXES; ++i)
		pulsePos[i] = pulse_data.lPos[i];
	
	return TRUE;
}

//-------------------------------------------------------------------
// Retrieves the absolute value (Nm) of the maximum current servo torque.
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetTorque(CtrlGroup* ctrlGroup, double torqueValues[MAX_PULSE_AXES])
{
    MP_GRP_AXES_T dst_vel;
    MP_TRQ_CTL_VAL dst_trq;
  	LONG status = 0;
  	int i;

	memset(torqueValues, 0, sizeof(torqueValues)); // clear result, in case of error
	memset(dst_trq.data, 0, sizeof(MP_TRQCTL_DATA));
	dst_trq.unit = TRQ_NEWTON_METER; //request data in Nm

	status = mpSvsGetVelTrqFb(dst_vel, &dst_trq);
	if (status != OK)
		return FALSE;

	for (i = 0; i < MAX_PULSE_AXES; i += 1) 
	{
	    torqueValues[i] = (double)dst_trq.data[ctrlGroup->groupId][i] * 0.000001; //Use double.  Float only good for 6 sig digits.
	}
    
    return TRUE;
}

// Convert Motoman position in pulse to Ros position in radian/meters
// In the case of a 7, 4, or 5 axis robot, adjust the order to match 
// the physical axis sequence
//-------------------------------------------------------------------
void Ros_CtrlGroup_ConvertToRosPos(CtrlGroup* ctrlGroup, long motopulsePos[MAX_PULSE_AXES],
									float rosPos[MAX_PULSE_AXES])
{
	int i;
	float conversion = 1;
	int rpi = 0; //rosPos index
	int mpi = 0; //motopos index

	if ((ctrlGroup->numAxes == 7) && Ros_CtrlGroup_IsRobot(ctrlGroup)) //is robot, and is 7 axis
	{
		// Adjust joint order for 7 axis robot (SLURBTE > SLEURBT); All rotary axes

		for (i = 0; i < ctrlGroup->numAxes; i++)
		{
			if (i < 2)
				rosPos[i] = motopulsePos[i] / ctrlGroup->pulseToRad.PtoR[i];
			else if (i == 2)
				rosPos[2] = motopulsePos[6] / ctrlGroup->pulseToRad.PtoR[6];
			else
				rosPos[i] = motopulsePos[i - 1] / ctrlGroup->pulseToRad.PtoR[i - 1];
		}
	}
	else if (Ros_CtrlGroup_IsRobot(ctrlGroup) && ctrlGroup->numAxes < 6)
	{
		//Delta: (SLU--T- > SLUT---) All rotary axes
		//Scara: (SLUR--- > SLUR---) U-axis is linear
		//Large Palletizing: (SLU--T- > SLUT---) All rotary axes
		//High Speed Picking: (SLU-BT- > SLUBT--) All rotary axes

		for (i = rpi = mpi = 0; i < ctrlGroup->numAxes; i += 1, rpi += 1, mpi += 1)
		{
			while (ctrlGroup->axisType.type[mpi] == AXIS_INVALID)
			{
				mpi += 1;
				if (mpi >= MAX_PULSE_AXES)
					return;
			}

			if (ctrlGroup->axisType.type[mpi] == AXIS_ROTATION)
				conversion = ctrlGroup->pulseToRad.PtoR[mpi];
			else if (ctrlGroup->axisType.type[mpi] == AXIS_LINEAR)
				conversion = ctrlGroup->pulseToMeter.PtoM[mpi];
			else
				conversion = 1.0;

			rosPos[rpi] = motopulsePos[mpi] / conversion;
		}
	}
	else
	{
		for (i = 0; i < MAX_PULSE_AXES; i++)
		{
			if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
				conversion = ctrlGroup->pulseToRad.PtoR[i];
			else if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
				conversion = ctrlGroup->pulseToMeter.PtoM[i];
			else
				conversion = 1.0;

			rosPos[i] = motopulsePos[i] / conversion;
		}
	}
}

//-------------------------------------------------------------------
// Convert Ros position in radian to Motoman position in pulse
// In the case of a 7 or 4 axis robot, adjust the order to match 
// the motoman axis sequence
//-------------------------------------------------------------------
void Ros_CtrlGroup_ConvertToMotoPos(CtrlGroup* ctrlGroup, float radPos[MAX_PULSE_AXES], long motopulsePos[MAX_PULSE_AXES])
{
	int i;
	float conversion = 1;
	int rpi = 0; //radpos index
	int mpi = 0; //motopos index

	// Initialize memory space
	memset(motopulsePos, 0x00, sizeof(long)*MAX_PULSE_AXES);

	if ((ctrlGroup->numAxes == 7) && Ros_CtrlGroup_IsRobot(ctrlGroup))
	{
		// Adjust joint order for 7 axis robot (SLEURBT > SLURBTE); All rotary axes

		for (i = 0; i < ctrlGroup->numAxes; i++)
		{
			if (i < 2)
				motopulsePos[i] = (int)(radPos[i] * ctrlGroup->pulseToRad.PtoR[i]);
			else if (i == 2)
				motopulsePos[6] = (int)(radPos[2] * ctrlGroup->pulseToRad.PtoR[6]);
			else
				motopulsePos[i - 1] = (int)(radPos[i] * ctrlGroup->pulseToRad.PtoR[i - 1]);
		}
	}
	else if (Ros_CtrlGroup_IsRobot(ctrlGroup) && ctrlGroup->numAxes < 6)
	{
		//Delta: (SLUT--- > SLU--T-) All rotary axes
		//Scara: (SLUR--- > SLUR---) U-axis is linear
		//Large Palletizing: (SLUT--- > SLU--T-) All rotary axes
		//High Speed Picking: (SLUBT-- > SLU-BT-) All rotary axes

		for (i = rpi = mpi = 0; i < ctrlGroup->numAxes; i += 1, rpi += 1, mpi += 1)
		{
			while (ctrlGroup->axisType.type[mpi] == AXIS_INVALID)
			{
				mpi += 1;
				if (mpi >= MAX_PULSE_AXES)
					return;
			}

			if (ctrlGroup->axisType.type[mpi] == AXIS_ROTATION)
				conversion = ctrlGroup->pulseToRad.PtoR[mpi];
			else if (ctrlGroup->axisType.type[mpi] == AXIS_LINEAR)
				conversion = ctrlGroup->pulseToMeter.PtoM[mpi];
			else
				conversion = 1.0;

			motopulsePos[mpi] = (int)(radPos[rpi] * conversion);
		}
	}
	else
	{
		// Convert to pulse
		for (i = 0; i < MAX_PULSE_AXES; i++)
		{
			if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
				conversion = ctrlGroup->pulseToRad.PtoR[i];
			else if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
				conversion = ctrlGroup->pulseToMeter.PtoM[i];
			else
				conversion = 1.0;

			motopulsePos[i] = (int)(radPos[i] * conversion);
		}
	}
}

//-------------------------------------------------------------------
// Returns a bit wise axis configuration for the increment move API
//-------------------------------------------------------------------
UCHAR Ros_CtrlGroup_GetAxisConfig(CtrlGroup* ctrlGroup)
{
	int i;
	int axisConfig = 0;
	
	for (i = 0; i < MAX_PULSE_AXES; i++)
	{
		if (ctrlGroup->axisType.type[i] != AXIS_INVALID)
			axisConfig |= (0x01 << i);
	}
		
	return (UCHAR)axisConfig;
}

//-------------------------------------------------------------------
// Returns TRUE is the specified group is defined as a robot
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_IsRobot(CtrlGroup* ctrlGroup)
{
	return((ctrlGroup->groupId >= MP_R1_GID) && (ctrlGroup->groupId <= MP_R4_GID));
}
