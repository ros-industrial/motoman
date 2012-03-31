/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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
 
#ifdef ROS
#include "dx100/ros_conversion.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "ros_conversion.h"
#include "log_wrapper.h"
#endif

using namespace industrial::joint_data;

namespace motoman
{

namespace ros_conversion
{


// Pulse to radian conversion factors (initialized on startup)
float S_PULSE_TO_RAD	= 0;	    // pulses/rad
float L_PULSE_TO_RAD	= 0;	    // pulses/rad
float U_PULSE_TO_RAD	= 0;	    // pulses/rad
float R_PULSE_TO_RAD    = 0;	    // pulses/rad
float B_PULSE_TO_RAD	= 0;     	// pulses/rad
float T_PULSE_TO_RAD	= 0;	    // pulses/rad
float E_PULSE_TO_RAD	= 0;	    // pulses/rad




void initJointConversion(MotomanRobotModel model_number)
{
    
    LOG_INFO("Initializing joint conversion factors for: ");
    switch (model_number)
    {
    case MotomanRobotModels::SIA_10D:
        LOG_INFO("SIA_10D: %d", model_number);
        S_PULSE_TO_RAD	= 58670.87822;	    
        L_PULSE_TO_RAD	= 58670.87822;	 
        U_PULSE_TO_RAD	= 65841.76588;	    
        R_PULSE_TO_RAD  = 65841.76588;	  
        B_PULSE_TO_RAD	= 65841.76588;    
        T_PULSE_TO_RAD	= 33246.8329;	  
        E_PULSE_TO_RAD	= 65841.76588;	
        break;
        
    case MotomanRobotModels::SIA_20D:
        LOG_INFO("SIA_20D: %d", model_number);
        S_PULSE_TO_RAD	= 58670.87822;	    
        L_PULSE_TO_RAD	= 58670.87822;	 
        U_PULSE_TO_RAD	= 58670.87822;	    
        R_PULSE_TO_RAD  = 65841.76588;	  
        B_PULSE_TO_RAD	= 65841.76588;    
        T_PULSE_TO_RAD	= 33246.8329;	  
        E_PULSE_TO_RAD	= 58670.87822;	
        break;
    
    default:
        LOG_ERROR("Failed to initialize conversion factors for model: %d", model_number);
        break;
    }
}



float toPulses(float radians, MotomanJointIndex joint)
{
    float rtn = 0.0;
    switch (joint)
    {
      case MotomanJointIndexes::S:
         rtn = radians * S_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::L:
         rtn = radians * L_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::U:
         rtn = radians * U_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::R:
         rtn = radians * R_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::B:
         rtn = radians * B_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::T:
         rtn = radians * T_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::E:
         rtn = radians * E_PULSE_TO_RAD;
         break;
         
      default:
        rtn = radians;
    }
        return rtn;
}

float toRadians(float pulses, MotomanJointIndex joint)
{
    float rtn = 0.0;
    switch (joint)
    {
      case MotomanJointIndexes::S:
         rtn = pulses / S_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::L:
         rtn = pulses / L_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::U:
         rtn = pulses / U_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::R:
         rtn = pulses / R_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::B:
         rtn = pulses / B_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::T:
         rtn = pulses / T_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::E:
         rtn = pulses / E_PULSE_TO_RAD;
         break;
         
      default:
        rtn = pulses;
    }
        return rtn;
}

void toRosJoint(industrial::joint_data::JointData & mp_joints, 
				industrial::joint_data::JointData & ros_joints)
{

	ros_joints.copyFrom(mp_joints);
	// joints still in motoplus order for conversion
	for (int i = 0; i < ros_joints.getMaxNumJoints() ; i++)
	{
		ros_joints.setJoint(i, toRadians(ros_joints.getJoint(i), 
							(MotomanJointIndex)i));
	}
	toRosJointOrder(ros_joints);
}

void toMpJoint(industrial::joint_data::JointData & ros_joints,
				industrial::joint_data::JointData & mp_joints)
{

	mp_joints.copyFrom(ros_joints);
	toMotomanJointOrder(mp_joints);
	// joints in motoplus order for conversion
	for (int i = 0; i < mp_joints.getMaxNumJoints() ; i++)
	{
		ros_joints.setJoint(i, toPulses(mp_joints.getJoint(i), 
						(MotomanJointIndex)i));
	}
}

			
void toRosJointOrder(JointData & joints)
{
    //LOG_DEBUG("Swapping to ROS joint order");
    JointData swap;
    swap.setJoint(RosJointIndexes::S, joints.getJoint(MotomanJointIndexes::S));
    swap.setJoint(RosJointIndexes::L, joints.getJoint(MotomanJointIndexes::L));
    swap.setJoint(RosJointIndexes::U, joints.getJoint(MotomanJointIndexes::U));
    swap.setJoint(RosJointIndexes::R, joints.getJoint(MotomanJointIndexes::R));
    swap.setJoint(RosJointIndexes::B, joints.getJoint(MotomanJointIndexes::B));
    swap.setJoint(RosJointIndexes::T, joints.getJoint(MotomanJointIndexes::T));
    swap.setJoint(RosJointIndexes::E, joints.getJoint(MotomanJointIndexes::E));
    joints.copyFrom(swap);
}

void toMotomanJointOrder(JointData & joints)
{
    //LOG_DEBUG("Swapping to motoman joint order");
    JointData swap;
    swap.setJoint(MotomanJointIndexes::S, joints.getJoint(RosJointIndexes::S));
    swap.setJoint(MotomanJointIndexes::L, joints.getJoint(RosJointIndexes::L));
    swap.setJoint(MotomanJointIndexes::U, joints.getJoint(RosJointIndexes::U));
    swap.setJoint(MotomanJointIndexes::R, joints.getJoint(RosJointIndexes::R));
    swap.setJoint(MotomanJointIndexes::B, joints.getJoint(RosJointIndexes::B));
    swap.setJoint(MotomanJointIndexes::T, joints.getJoint(RosJointIndexes::T));
    swap.setJoint(MotomanJointIndexes::E, joints.getJoint(RosJointIndexes::E));
    joints.copyFrom(swap);
}

} //ros_conversion
} //motoman
